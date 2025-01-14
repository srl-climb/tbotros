from __future__ import annotations

import rclpy
import numpy as np
import quaternion as qu
from typing import Tuple
from rclpy_wrapper.node import Node2
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from multiprocessing import Process, Queue as ProcessQueue
from threading import Lock, Event, Thread
from custom_actions.action import PlanTetherbot
from custom_msgs.msg import Float64Array
from custom_srvs.srv import SetString, DisplayCommands
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Empty, Trigger
from std_msgs.msg import Bool, Int64, String
from tbotlib import TbTetherbot, GlobalPlanner, CommandList, TransformMatrix, TetherbotVisualizer, yaml2planner
from time import time_ns, sleep

class PlannerNode(Node2):

    def __init__(self):

        super().__init__(node_name = 'planner')

        self.declare_parameter('desc_file', '/home/climb/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot.pkl')
        self._desc_file = self.get_parameter('desc_file').get_parameter_value().string_value

        self.declare_parameter('config_file', '/home/climb/ros2_ws/src/tbotros_config/tbotros_config/config/planner.yaml')
        self._config_file = self.get_parameter('config_file').get_parameter_value().string_value

        self.declare_parameter('commands_file', '/home/srl-orin/ros2_ws/command/command.pkl')
        self._commands_file = self.get_parameter('commands_file').get_parameter_value().string_value
        
        self._tbot: TbTetherbot = TbTetherbot.load(self._desc_file)
        self._tbot_light: TbTetherbot = TbTetherbot.load(self._desc_file)
        self._tbot_light.remove_all_geometries()

        # intermediary objects
        self._tbot_platform_arm_qs = self._tbot.platform.arm.qs
        self._tbot_platform_T_local = self._tbot.platform.T_local
        self._tbot_grippers_T_world: list[TransformMatrix] = [gripper.T_world for gripper in self._tbot.grippers]
        self._tbot_grippers_hold_name: list[str] = [self._tbot.wall.get_hold(gripper.parent.name).name for gripper in self._tbot.grippers]

        for gripper in self._tbot.grippers:
            self._tbot_grippers_T_world.append(gripper.T_world)
            self._tbot_grippers_hold_name.append('')

        self.load_planner_config()

        # actions
        self.create_action_server(PlanTetherbot, self.get_name() + '/plan', 
                     execute_callback = self.plan_action_execute_callback,
                     cancel_callback = self.plan_action_cancel_callback,
                     goal_callback = self.plan_action_goal_callback)
        # subscriptions for required transformations/joint states
        self.create_subscription(PoseStamped, self._tbot.platform.name + '/platform_state_publisher/pose', self.platform_pose_sub_callback, 1)
        self.create_subscription(Float64Array, self._tbot.platform.arm.name + '/arm_state_publisher/joint_states', self.arm_joint_states_sub_callback, 1)
        for i in range(self._tbot.k):
            self.create_subscription(PoseStamped, self._tbot.grippers[i].name + '/gripper_state_publisher/pose', lambda msg, i=i: self.gripper_pose_sub_callback(msg, i), 1)
            self.create_subscription(String, self._tbot.grippers[i].name + '/gripper_state_publisher/hold_name', lambda msg, i=i: self.gripper_hold_name_sub_callback(msg, i), 1)
        # services
        self.create_service(Empty, self.get_name() + '/display_state', self.display_state_srv_callback)
        self.create_service(DisplayCommands, self.get_name() + '/display_commands', self.display_commands_srv_callback)
        self.create_service(SetString, self.get_name() + '/set_commands_file', self.set_commands_file_srv_callback)
        self.create_service(Trigger, self.get_name() + '/save_commands', self.save_commands_srv_callback)
        # NOTE: Put set_commands and save_commands in same callback group to avoid changing the path while saving commands
        # publishers
        self._busy_pub = self.create_publisher(Bool, self.get_name() + '/busy',1)
        self._commands_file_pub = self.create_publisher(String, self.get_name() + '/commands_file',1)
        self._commands_saved_pub = self.create_publisher(Bool, self.get_name() + '/commands_saved',1)
        # timers
        self.create_timer(1, self.timer_callback)
        
        self._commands = CommandList()
        self._commands_saved = False
        self._commands_lock = Lock()
        self._busy_lock = Lock()
        self._busy = False
        self._rate = self.create_rate(4)

        # display thread objects
        self._stop_event = Event()

    def timer_callback(self):

        msg = Bool()
        msg.data = self._busy
        self._busy_pub.publish(msg)

        msg = String()
        msg.data = self._commands_file
        self._commands_file_pub.publish(msg)

        msg = Bool()
        msg.data = self._commands_saved
        self._commands_saved_pub.publish(msg)

    def update_tbot(self, tbot: TbTetherbot):

        tbot.platform.arm.qs = self._tbot_platform_arm_qs
        tbot.platform.T_local = self._tbot_platform_T_local
        self.get_logger().info(str(tbot.platform.T_local.decompose()))
        for i in range(tbot.k):
            tbot.grippers[i].T_world = tbot.grippers[i].T_world

        for hold_name, gripper_index in zip(self._tbot_grippers_hold_name, range(tbot.k)):
            hold_index = [hold.name for hold in tbot.wall.holds].index(hold_name)
            tbot.place(gripper_index, hold_index, correct_pose = True)

    def arm_joint_states_sub_callback(self, msg: Float64Array):
        
        self._tbot_platform_arm_qs = msg.data
        self._tbot_platform_arm_qs[0] = self._tbot_platform_arm_qs[0] * (np.pi/180)

    def platform_pose_sub_callback(self, msg: PoseStamped):

        self._tbot_platform_T_local = TransformMatrix(self.pose2mat(msg.pose))

    def gripper_pose_sub_callback(self, msg: PoseStamped, i: int):

        self._tbot_grippers_T_world[i] = TransformMatrix(self.pose2mat(msg.pose))

    def gripper_hold_name_sub_callback(self, msg: Int64, i: int):

        self._tbot_grippers_hold_name[i] = msg.data

    def set_commands_file_srv_callback(self, request: SetString.Request, response: SetString.Response) -> SetString.Response:

        with self._commands_lock:
            self._commands_file = request.data
            self._commands_saved = False

        return response

    def save_commands_srv_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        
        with self._commands_lock:
            try:
                self._commands.save(self._commands_file, overwrite = True)
            except Exception as exc:
                response.success = False
                self.get_logger().error('Failed saving commands: ' + str(exc))
            else:
                self._commands_saved = True
                response.success = True
                self.get_logger().info('Saved commands: ' + self._commands_file)

        return response

    def plan_action_execute_callback(self, goal_handle: ServerGoalHandle):

        state = 0
        while True:
            self._rate.sleep()
            # initialize
            if state == 0:
                request: PlanTetherbot.Goal = goal_handle.request
                process: Process = None
                queue: ProcessQueue = None
                state = 1
                # update tbot to current configuration
                self.update_tbot(self._tbot_light)
                if request.mode == 0: # plan platform
                    planner = self._platform2pose
                    self.get_logger().info(str(TransformMatrix(request.goal_pose).decompose()))
                    kwargs = {'pose': TransformMatrix(request.goal_pose)}
                elif request.mode == 1: # plan arm
                    planner = self._arm2pose
                    kwargs = {'pose': TransformMatrix(request.goal_pose)}
                elif request.mode == 2: # plan pick
                    planner = self._local_planner
                    kwargs = {'grip_idx': request.gripper_index, 
                              'hold_idx': 0, 
                              'start_state': 0, 
                              'goal_state': 5}
                elif request.mode == 3: # plan place
                    planner = self._local_planner
                    kwargs = {'grip_idx': 0, 
                              'hold_idx': request.hold_index, 
                              'start_state': 6, 
                              'goal_state': 10}
                elif request.mode == 4: # plan pick and place
                    planner = self._local_planner
                    kwargs = {'grip_idx': request.gripper_index, 
                              'hold_idx': request.hold_index, 
                              'start_state': 0, 
                              'goal_state': 10}
                elif request.mode == 5: # plan platform to configuration
                    planner = self._platform2configuration
                    kwargs = {'grip_idx': request.gripper_index}
                elif request.mode == 6: # plan global
                    planner = self._global_planner
                    kwargs = {'start': [self._tbot_light.wall.holds.index(gripper.parent) for gripper in self._tbot_light.grippers],
                              'goal': request.goal_configuration}
                else:
                    goal_handle.abort()
                    self.get_logger().error('Plan: Aborted, received unknown mode in goal request')
                    state = 99
                feedback = PlanTetherbot.Feedback()
                feedback.message = 'Planning path with planner "' + planner.__class__.__name__ + '" and arguments: ' + str(kwargs)
                goal_handle.publish_feedback(feedback)

            # start path planning process               
            elif state == 1:
                queue = ProcessQueue(1)
                process = Process(target = self.plan_process_func,
                                  args = (queue, self._tbot_light, planner, kwargs))
                process.start()
                state = 2

            # wait for path planning process
            elif state == 2:
                if not queue.empty():
                    process_result: Tuple[TbTetherbot, CommandList, bool] = queue.get()
                    process.join()
                    goal_handle.succeed() 
                    if process_result[2] is None or process_result[2] == False:
                        self.get_logger().info('Plan: Succeeded, no viable path found')
                    else:
                        self.get_logger().info('Plan: Succeeded, path found')
                        with self._commands_lock:
                            self._commands = process_result[1]
                            self._commands_saved = False
                    state = 99
                elif process.exitcode is not None:
                    self.get_logger().error('Plan: Abort, planning process terminated early: Exitcode ' + str(process.exitcode))
                    goal_handle.abort()
                    state = 99
                else:
                    state = 2

            elif state == 99:
                if process is not None:
                    if process.is_alive():
                        process.terminate()
                break

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                state = 99

        with self._busy_lock:
            self._busy = False

        return PlanTetherbot.Result()
        
    def plan_action_goal_callback(self, _):

        with self._busy_lock:
            if self._busy:
                self.get_logger().info('Plan: Goal rejected, busy')
                return GoalResponse.REJECT
            else:
                self._busy = True
                return GoalResponse.ACCEPT
            
    def plan_action_cancel_callback(self, _):

        return CancelResponse.ACCEPT
            
    def display_state_srv_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:

        self.update_tbot(self._tbot)
        
        thread = Thread(target = self.display_func, args = (self._tbot, self._stop_event, CommandList()))
        thread.start()
        # NOTE: The open3d functions are run in a seperate thread as using a timer_callback leads to warnings and lag

        return response
    
    def display_commands_srv_callback(self, request: DisplayCommands.Request, response: DisplayCommands.Response) -> DisplayCommands.Response:
        
        self.update_tbot(self._tbot)

        thread = Thread(target = self.display_func, args = (self._tbot, self._stop_event, self._commands, request.speed, self._simulation_dt))
        thread.start()
        # NOTE: The open3d functions are run in a seperate thread as using a timer_callback leads to warnings and lag
        
        return response
    
    @staticmethod
    def display_func(tbot: TbTetherbot, stop_event: Event, commands: CommandList, speed: int = 1, simulation_dt: float = 1):

        vi = TetherbotVisualizer(tbot)
        speed = max(speed, 1)
        state = 0
        i = 0

        while not stop_event.is_set() and vi.opened:
            if state == 0:
                if i < len(commands):
                    commands[i].reset()
                    start_time = time_ns() / 1000000000 * speed
                    state = 1
                else:
                    state = 4
            if state == 1:
                current_time = time_ns() / 1000000000 * speed
                # execute the simulation in real-time
                if (current_time-start_time) >= simulation_dt:
                    # calculate how many steps of dt to move forward in the display
                    step = ((current_time-start_time)//simulation_dt) 
                    # execute the command
                    done = commands[i].do(tetherbot=tbot, step=int(step))
                    if done:
                        i = i + 1
                        state = 0
                    else:
                        start_time = start_time + step * simulation_dt 
                        state = 1
                else:
                    state = 1
            elif state == 4:
                sleep(0.1)
            
            vi.update()  

        vi.close()
            
    @staticmethod
    def plan_process_func(queue: ProcessQueue, tbot: TbTetherbot, planner: GlobalPlanner, kwargs: dict):                    

        queue.put(planner.plan(tetherbot = tbot, commands = CommandList(), **kwargs))

    def load_planner_config(self):

        with open(self._config_file, "r") as stream:
            try: 
                self._simulation_dt, self._platform2pose, self._platform2configuration, self._arm2pose, self._local_planner, self._global_planner = yaml2planner(self._config_file)
            except Exception as exc:
                self.get_logger().error("Failed loading planner configuration file '" + self._config_file + ": '" + str(exc))
            else:
                self.get_logger().info("Loaded planner configuration file '" + self._config_file + "'")
        
    def pose2mat(self, pose: Pose) -> np.ndarray:

        T = np.eye(4)
        T[:3,:3] = qu.as_rotation_matrix(qu.from_float_array([pose.orientation.w, 
                                                              pose.orientation.x, 
                                                              pose.orientation.y,
                                                              pose.orientation.z]))
        T[:3,3]  = np.array([pose.position.x,
                             pose.position.y,
                             pose.position.z])
        
        return T

    def destroy_node(self) -> bool:

        self._stop_event.set()

        return super().destroy_node()
    

def main(args = None):

    rclpy.init(args = args)
    try:
        node = PlannerNode()
        executor = MultiThreadedExecutor()
        try:
            rclpy.spin(node, executor = executor)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()