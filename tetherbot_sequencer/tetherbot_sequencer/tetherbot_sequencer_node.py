from __future__ import annotations

import rclpy
from threading import Lock
from rclpy_wrapper.node import Node2
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.client import Client
from custom_actions.action import MoveTetherbot, Empty as EmptyAction, ExecuteSequence
from custom_srvs.srv import SetString, Tension
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty as EmptyService
from std_srvs.srv import Trigger
from tbotlib import TbTetherbot, CommandMoveArm, CommandMovePlatform, CommandPickGripper, CommandPlaceGripper, CommandList
from .handler import TbHandler, TbEmptyActionHandler, TbMoveActionHandler, TbSetStringServiceHandler, TbTensionServiceHandler, TbDelayHandler, TbEmptyServiceHandler

class SequencerNode(Node2):

    def __init__(self):

        super().__init__(node_name ='sequencer')

        self.declare_parameter('desc_file', '/home/climb/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot_light.pkl')
        self._desc_file = self.get_parameter('desc_file').get_parameter_value().string_value

        self.declare_parameter('gripper_transform_source', value = 'hold')
        self._gripper_transform_source = self.get_parameter('gripper_transform_source').get_parameter_value().string_value

        self.declare_parameter('commands_file', '/home/climb/ros2_ws/commands/commands.pkl')
        self._commands_file = self.get_parameter('commands_file').get_parameter_value().string_value
        
        self._tbot: TbTetherbot = TbTetherbot.load(self._desc_file)
        self._rate = self.create_rate(10)
        self._busy_lock = Lock()
        self._busy = False
        self._auto = False
        self._next = False
        self._commands = CommandList()
        self._commands_lock = Lock()

        # actions
        self.create_action_server(ExecuteSequence, self.get_name() + '/execute_sequence', 
                     cancel_callback = self.execute_sequence_cancel_callback, 
                     execute_callback = self.execute_sequence_execute_callback, 
                     goal_callback = self.execute_sequence_goal_callback)

        self._platform_move_client = self.create_action_client(MoveTetherbot, self._tbot.platform.name + '/platform_controller/move')
        self._arm_move_client = self.create_action_client(MoveTetherbot, self._tbot.platform.arm.name + '/arm_controller/move')
        self._gripper_open_clients:  list[ActionClient] = []
        self._gripper_close_clients: list[ActionClient] = []
        self._gripper_set_hold_clients: list[Client] = []
        self._gripper_set_transform_source_clients: list[Client] = []
        for gripper in self._tbot.grippers:
            self._gripper_close_clients.append(self.create_action_client(EmptyAction, gripper.name + '/gripper_controller/close'))
            self._gripper_open_clients.append(self.create_action_client(EmptyAction, gripper.name + '/gripper_controller/open'))
            self._gripper_set_hold_clients.append(self.create_client(SetString, gripper.name + '/gripper_state_publisher/set_hold'))
            self._gripper_set_transform_source_clients.append(self.create_client(SetString, gripper.name + '/gripper_state_publisher/set_transform_source'))
        self._docking_close_client = self.create_action_client(EmptyAction, self._tbot.platform.arm.name + '/docking_controller/close')
        self._docking_open_client = self.create_action_client(EmptyAction, self._tbot.platform.arm.name + '/docking_controller/open')
        
        # clients
        self._tension_tethers_client = self.create_client(Tension, self._tbot.platform.name + '/platform_controller/tension_gripper_tethers')
        self._enable_arm_control_client = self.create_client(Tension, self._tbot.platform.arm.name + '/arm_controller/enable_control')
        self._disable_arm_control_client = self.create_client(Tension, self._tbot.platform.arm.name + '/arm_controller/disable_control')
        self._enable_platform_control_client = self.create_client(Tension, self._tbot.platform.name + '/platform_controller/enable_control')
        self._disable_platform_control_client = self.create_client(Tension, self._tbot.platform.name + '/platform_controller/disable_control')

        # services
        self.create_service(SetString, self.get_name() + '/set_commands_file', self.set_commands_file_callback)
        self.create_service(Trigger, self.get_name() + '/load_commands', self.load_commands_callback)
        self.create_service(EmptyService, self.get_name() + '/enable_auto', self.enable_auto_callback)
        self.create_service(EmptyService, self.get_name() + '/disable_auto', self.disable_auto_callback)
        self.create_service(EmptyService, self.get_name() + '/next', self.next_callback)
        
        # publishers
        self._busy_pub = self.create_publisher(Bool, self.get_name() + '/busy', 1)
        self._auto_pub = self.create_publisher(Bool, self.get_name() + '/auto', 1)
        self._commands_file_pub = self.create_publisher(String, self.get_name() + '/commands_file', 1)
        self._commands_loaded_pub = self.create_publisher(Bool, self.get_name() + '/commands_loaded', 1)
        
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

        msg = Bool()
        msg.data = self._auto
        self._auto_pub.publish(msg)

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
    
    def enable_auto_callback(self, request: EmptyService.Request, response: EmptyService.Response) -> EmptyService.Response:

        self._auto = True

        return response
    
    def disable_auto_callback(self, request: EmptyService.Request, response: EmptyService.Response) -> EmptyService.Response:

        self._auto = False

        return response
    
    def next_callback(self, request: EmptyService.Request, response: EmptyService.Response) -> EmptyService.Response:

        self._next = True

        return response
    
    def execute_sequence_execute_callback(self, goal_handle: ServerGoalHandle):

        state = 0

        while True:
            # initialize
            if state == 0:
                handlers: list[TbHandler] = self.parse_commands()
                current_handler: TbHandler = None
                feedback = ExecuteSequence.Feedback()
                feedback.length = len(handlers)
                if handlers:
                    feedback.message = 'Start'
                    self.get_logger().info('Execute sequence: ' + feedback.message)    
                    self._next = False  
                    state = 1
                else:
                    goal_handle.abort()
                    feedback.message = 'Aborted, loaded sequence empty'
                    self.get_logger().error('Execute sequence: ' + feedback.message)
                    state = 99
            # start new action if auto or next is TRUE
            elif state == 1:
                if not handlers:
                    current_handler = None
                    goal_handle.succeed()
                    feedback.message = 'Succeeded'
                    self.get_logger().info('Execute sequence: ' + feedback.message)
                    state = 99
                elif handlers and (self._auto or self._next):
                    current_handler = handlers.pop(0)
                    feedback.message = 'Executing ' + current_handler.info()
                    self.get_logger().info('Execute sequence: ' + feedback.message)
                    state = 2
                else:
                    state = 1
            # execute handler
            elif state == 2:
                done, status = current_handler.execute()
                if done:
                    if status == TbHandler.SUCCEEDED:
                        feedback.message = current_handler.info().capitalize() + ' succeeded'
                        self.get_logger().info('Execute sequence: ' + feedback.message)
                        self._next = False
                        state = 1
                    elif status == TbHandler.CANCELED:
                        goal_handle.abort()
                        feedback.message = 'Aborted, ' + current_handler.info() + ' canceled'
                        self.get_logger().error('Execute sequence: ' + feedback.message)
                        state = 99
                    elif status == TbHandler.ABORTED:
                        goal_handle.abort()
                        feedback.message = 'Aborted, ' + current_handler.info() + ' aborted'
                        self.get_logger().error('Execute sequence: ' + feedback.message)
                        state = 99
                else:
                    state = 2
            # clean up and exit
            elif state == 99:
                if current_handler is not None:
                    current_handler.cancel()
                break
            # exit conditions
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                feedback.message = 'Canceled'
                self.get_logger().info('Execute sequence: ' + feedback.message)
                state = 99
            # publish feedback
            if feedback.length > 0:
                feedback.current = feedback.length - len(handlers)
                feedback.progress = float((feedback.length - len(handlers)) / feedback.length)
                goal_handle.publish_feedback(feedback)
            self._rate.sleep()
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
                        handlers.append(TbEmptyServiceHandler(self._enable_platform_control_client))
                        handlers.append(TbMoveActionHandler(self._platform_move_client, command._targetposes))
                        handlers.append(TbEmptyServiceHandler(self._disable_platform_control_client))
                        handlers.append(TbDelayHandler(3))
                    # move arm
                    elif type(command) is CommandMoveArm:
                        handlers.append(TbEmptyServiceHandler(self._enable_platform_control_client))
                        handlers.append(TbEmptyServiceHandler(self._enable_arm_control_client))
                        handlers.append(TbMoveActionHandler(self._arm_move_client, command._targetposes))
                        handlers.append(TbEmptyServiceHandler(self._disable_arm_control_client))
                        handlers.append(TbEmptyServiceHandler(self._disable_platform_control_client))
                        handlers.append(TbDelayHandler(3))
                    # pick gripper
                    elif type(command) is CommandPickGripper:
                        handlers.append(TbEmptyServiceHandler(self._enable_platform_control_client))
                        handlers.append(TbTensionServiceHandler(self._tension_tethers_client, command._grip_idx, 0))
                        handlers.append(TbDelayHandler(3))
                        handlers.append(TbEmptyServiceHandler(self._disable_platform_control_client))
                        handlers.append(TbEmptyActionHandler(self._docking_close_client))
                        handlers.append(TbEmptyActionHandler(self._gripper_open_clients[command._grip_idx])) 
                        handlers.append(TbSetStringServiceHandler(self._gripper_set_transform_source_clients[command._grip_idx], 'arm'))  
                        handlers.append(TbDelayHandler(3))
                    # place gripper
                    elif type(command) is CommandPlaceGripper:
                        handlers.append(TbEmptyActionHandler(self._gripper_close_clients[command._grip_idx]))
                        handlers.append(TbEmptyActionHandler(self._docking_open_client))
                        handlers.append(TbSetStringServiceHandler(self._gripper_set_hold_clients[command._grip_idx], str(command._hold_idx)))
                        handlers.append(TbSetStringServiceHandler(self._gripper_set_transform_source_clients[command._grip_idx], self._gripper_transform_source))  
                        handlers.append(TbEmptyServiceHandler(self._enable_platform_control_client))
                        handlers.append(TbTensionServiceHandler(self._tension_tethers_client, command._grip_idx, 1))
                        handlers.append(TbDelayHandler(3))
                        handlers.append(TbEmptyServiceHandler(self._disable_platform_control_client))
                        handlers.append(TbDelayHandler(3))
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

