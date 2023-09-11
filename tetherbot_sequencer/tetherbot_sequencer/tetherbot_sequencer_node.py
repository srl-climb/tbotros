from __future__ import annotations

import rclpy
from abc import ABC, abstractmethod
from threading import Lock
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.client import Client
from custom_actions.action import MoveTetherbot, Empty, ExecuteSequence
from custom_srvs.srv import SetString
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tbotlib import TbTetherbot, CommandMoveArm, CommandMovePlatform, CommandPickGripper, CommandPlaceGripper, CommandList, TransformMatrix
from .handler import TbHandler, TbEmptyActionHandler, TbMoveActionHandler, TbSetStringServiceHandler

class SequencerNode(Node):

    def __init__(self):

        super().__init__(node_name ='sequencer')

        self.declare_parameter('desc_file', '/home/climb/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot_light.pkl')
        self._desc_file = self.get_parameter('desc_file').get_parameter_value().string_value

        self.declare_parameter('gripper_transform_source', value = 'hold')
        self._gripper_transform_source = self.get_parameter('gripper_transform_source').get_parameter_value().string_value

        self.declare_parameter('commands_file', '/home/climb/ros2_ws/commands/commands.pkl')
        self._commands_file = self.get_parameter('commands_file').get_parameter_value().string_value
        
        self._tbot: TbTetherbot = TbTetherbot.load(self._desc_file)
        self._rate = self.create_rate(4)
        self._busy_lock = Lock()
        self._busy = False
        self._commands = CommandList()
        self._commands_lock = Lock()

        # actions
        ActionServer(self, ExecuteSequence, self.get_name() + '/execute_sequence', 
                     cancel_callback = self.execute_sequence_cancel_callback, 
                     execute_callback = self.execute_sequence_execute_callback, 
                     goal_callback = self.execute_sequence_goal_callback)

        self._platform_move_client = ActionClient(self, MoveTetherbot, self._tbot.platform.name + '/platform_controller/move')
        self._arm_move_client = ActionClient(self, MoveTetherbot, self._tbot.platform.arm.name + '/arm_controller/move')
        self._gripper_open_clients:  list[ActionClient] = []
        self._gripper_close_clients: list[ActionClient] = []
        self._gripper_set_hold_clients: list[Client] = []
        self._gripper_set_transform_source_clients: list[Client] = []
        for gripper in self._tbot.grippers:
            self._gripper_close_clients.append(ActionClient(self, Empty, gripper.name + '/gripper_controller/close'))
            self._gripper_open_clients.append(ActionClient(self, Empty, gripper.name + '/gripper_controller/open'))
            self._gripper_set_hold_clients.append(self.create_client(SetString, gripper.name + '/gripper_state_publisher/set_hold'))
            self._gripper_set_transform_source_clients.append(self.create_client(SetString, gripper.name + '/gripper_state_publisher/set_transform_source'))
        self._docking_close_client = ActionClient(self, Empty, self._tbot.platform.arm.name + '/docking_controller/close')
        self._docking_open_client = ActionClient(self, Empty, self._tbot.platform.arm.name + '/docking_controller/open')
        # services
        self.create_service(SetString, self.get_name() + '/set_commands_file', self.set_commands_file_callback)
        self.create_service(Trigger, self.get_name() + '/load_commands', self.load_commands_callback)
        # publishers
        self._busy_pub = self.create_publisher(Bool, self.get_name() + '/busy',1)
        self._commands_file_pub = self.create_publisher(String, self.get_name() + '/commands_file',1)
        self._commands_loaded_pub = self.create_publisher(Bool, self.get_name() + '/commands_loaded',1)
        # timers
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):

        msg = Bool()
        msg.data = self._busy
        self._busy_pub.publish(msg)

        msg = String()
        msg.data = self._commands_file
        self._commands_file_pub.publish(msg)

        msg = Bool()
        msg.data = len(self._commands) > 0
        self._commands_loaded_pub.publish(msg)

    def set_commands_file_callback(self, request: SetString.Request, response: SetString.Response) -> SetString.Response:

        with self._commands_lock:
            self._commands_file = request.data
            self._commands = CommandList()

        return response

    def load_commands_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:

        with self._commands_lock:
            try:
                self._commands = CommandList.load(self._commands_file)
            except Exception as exc:
                response.success = False
                self.get_logger().error('Failed loading commands: ' + str(exc))
            else:
                response.success = True
                self.get_logger().info('Loaded commands: ' + self._commands_file)

        return response

    def execute_sequence_execute_callback(self, goal_handle: ServerGoalHandle):

        state = 0

        while True:
            self._rate.sleep()
            self.get_logger().info('Execute sequence: State ' + str(state))
            # initialize
            if state == 0:
                handlers: list[TbHandler] = self.parse_commands()
                current_handler: TbHandler = None
                length = len(handlers)
                if handlers:      
                    state = 1
                else:
                    goal_handle.abort()
                    self.get_logger().error('Execute sequence: Aborted, loaded sequence empty')
                    state = 99
            
            # start new action
            elif state == 1:
                if handlers:
                    current_handler = handlers.pop(0)
                    self.get_logger().info('Execute sequence: Executing ' + current_handler.info())
                    state = 2
                else:
                    current_handler = None
                    goal_handle.succeed()
                    self.get_logger().info('Execute sequence: Completed')
                    state = 99

            # execute handler
            elif state == 2:
                current_handler.execute()
                self.get_logger().info('state: ' + str(current_handler._state))
                self.get_logger().info('state: ' + str(current_handler.status))
                if current_handler.status > 4:
                        goal_handle.abort()
                        self.get_logger().error('Execute sequence: Aborted, action unexpectedly canceled/aborted')
                        state = 99
                elif current_handler.status == 4:
                        state = 1
                else:
                    state = 2

            # clean up and exit
            elif state == 99:
                if current_handler is not None:
                    current_handler.cancel()
                self.get_logger().info('Execute sequence: Done')
                break

            # exit conditions
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Execute sequence: Canceled')
                state = 99

            # publish feedback
            if length > 0:
                feedback = ExecuteSequence.Feedback()
                feedback.length = length
                feedback.current = length - len(handlers)
                feedback.progress = float((length - len(handlers)) / length)
                goal_handle.publish_feedback(feedback)

        with self._busy_lock:
            self._busy = False

        return ExecuteSequence.Result()

    def execute_sequence_goal_callback(self, _):
        
        with self._busy_lock:
            if self._busy:
                self.get_logger().info('Execute sequence: Goal rejected, busy')
                return GoalResponse.REJECT
            else:
                self._busy = True
                return GoalResponse.ACCEPT
            
    def execute_sequence_cancel_callback(self, _):

        return CancelResponse.ACCEPT

    def parse_commands(self) -> list[TbHandler]:

        with self._commands_lock:
            commands = self._commands

            handlers: list[TbHandler] = []

            if commands:
                # actions/services to conduct before executing the commands
                for client in self._gripper_set_transform_source_clients:
                    handlers.append(TbSetStringServiceHandler(client, self._gripper_transform_source))

                # actions/services to conduct for each command
                for command in commands:
                    # move platform
                    if type(command) is CommandMovePlatform:
                        handlers.append(TbMoveActionHandler(self._platform_move_client, command._targetposes))
                    # move arm
                    elif type(command) is CommandMoveArm:
                        print('hi')
                        handlers.append(TbMoveActionHandler(self._arm_move_client, command._targetposes))
                    # pick gripper
                    elif type(command) is CommandPickGripper:
                        handlers.append(TbEmptyActionHandler(self._docking_close_client))
                        handlers.append(TbEmptyActionHandler(self._gripper_open_clients[command._grip_idx])) 
                        handlers.append(TbSetStringServiceHandler(self._gripper_set_transform_source_clients[command._grip_idx], 'arm'))  
                    # place gripper
                    elif type(command) is CommandPlaceGripper:
                        handlers.append(TbEmptyActionHandler(self._gripper_close_clients[command._grip_idx]))
                        handlers.append(TbEmptyActionHandler(self._docking_open_client))
                        handlers.append(TbSetStringServiceHandler(self._gripper_set_hold_clients[command._grip_idx], str(command._hold_idx)))
                        handlers.append(TbSetStringServiceHandler(self._gripper_set_transform_source_clients[command._grip_idx], self._gripper_transform_source))  
                    else:
                        pass

        return handlers


def main(args = None):

    rclpy.init(args = args)
    try:
        node = SequencerNode()
        executor = MultiThreadedExecutor()
        try:
            rclpy.spin(node, executor = executor)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':

    main()

