from __future__ import annotations
import numpy as np
from rclpy.node import Publisher
from rclpy.time import Time
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Lock
from custom_msgs.msg import Float64Stamped
from custom_actions.action import MoveTetherbot
from std_srvs.srv import Empty
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Pose, PoseStamped
from .tetherbot_control_base_node import BaseNode


class BaseControllerNode(BaseNode):

    def __init__(self, default_motor_node_names: list[str] = None, **kwargs):

        super().__init__(**kwargs)

        # actions
        self.create_action_server(MoveTetherbot, self.get_name() + '/move', 
                     execute_callback = self.execute_move_callback,
                     goal_callback = self.goal_move_callback,
                     cancel_callback = self.cancel_move_callback,
                     callback_group = ReentrantCallbackGroup())
             
        # services
        self.create_service(Empty, self.get_name() + '/enable_control', self.enable_control_callback)
        self.create_service(Empty, self.get_name() + '/disable_control', self.disable_control_callback)

        # publishers
        self._target_pose_pub = self.create_publisher(PoseStamped, self.get_name() + '/target_pose', 1)
        self._control_enabled_pub = self.create_publisher(Bool, self.get_name() + '/control_enabled', 1)
        self._csp_target_position_pubs: list[Publisher] = []
        for name in default_motor_node_names:
            self._csp_target_position_pubs.append(self.create_publisher(Float64Stamped, name + '/csp_target_position', 1))

        # subscriptions
        self._motor_modes: list[int] = []
        self._motor_running_states: list[bool] = []
        for i in range(len(default_motor_node_names)):
            self._motor_modes.append(0)
            self._motor_running_states.append(False)
            self.create_subscription(Bool, default_motor_node_names[i] + '/running', lambda msg, i=i: self.motor_running_callback(msg, i), 1)
            self.create_subscription(Int8, default_motor_node_names[i] + '/mode', lambda msg, i=i: self.motor_mode_callback(msg, i), 1)

        # timers
        self.create_timer(0.02, self.control_loop, callback_group = MutuallyExclusiveCallbackGroup())
        self.create_timer(0.2, self.watchdog_loop)

        # rate for waiting and monitoring in the action server executor
        self._move_action_rate = self.create_rate(10, self.get_clock())

        # lock for protecting execute_move_callback
        self._move_action_lock = Lock()

        ### ADDITIONAL STUFF ###

        # variable used by execute_move_callback to determine if a new move was started
        self._current_move_id = int(-1)

        # parameters of the control loop
        self._target_pose: Pose = None
        self._target_pose_queue = []
        self._control_enabled = False

    def control_loop(self):

        self.lookup_tbot_transforms()

        if self._control_enabled and self._target_pose is not None:
            
            # get the next target pose if available
            if self._target_pose_queue:
                self._target_pose = self._target_pose_queue.pop(0)

            # calculate target positions of the motors
            target_positions = self.control_function(self._target_pose)

            # publish target positions to the motors
            msg = Float64Stamped()
            stamp = self.get_clock().now().to_msg()
            for csp_target_position_pub, q in zip(self._csp_target_position_pubs, target_positions):
                msg.data = q
                msg.stamp = stamp
                csp_target_position_pub.publish(msg)

            # publish target pose
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            msg.pose = self._target_pose
            self._target_pose_pub.publish(msg)

    def watchdog_loop(self):

        if self._control_enabled:
            if all(mode == 8 for mode in self._motor_modes) and all(self._motor_running_states):
                pass
            else:
                self.get_logger().warn('One or more motors stopped running/are in wrong mode, disabling control')
                self._control_enabled = False

        # publish control enable state
        msg = Bool()
        msg.data = self._control_enabled
        self._control_enabled_pub.publish(msg)

    def control_function(self, target_pose: Pose) -> np.ndarray:

        pass

    def enable_control_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        
        self.get_logger().info('Enabling control')
        self._control_enabled = True

        return response
    
    def disable_control_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:

        self.get_logger().info('Disabling control')
        self._control_enabled = False

        return response
    
    def motor_running_callback(self, msg: Bool, i: int):

        self._motor_running_states[i] = msg.data

    def motor_mode_callback(self, msg: Int8, i: int):

        self._motor_modes[i] = msg.data

    def execute_move_callback(self, goal_handle: ServerGoalHandle):
        
        move_id = self.get_move_id()

        with self._move_action_lock:
            self.get_logger().info('Move [%s]: Start' %move_id)
       
            # execute state machine
            state = 0
            
            while True:
                self._move_action_rate.sleep()

                if state == 0:
                    self._target_pose_queue = self.array2poses(goal_handle.request.poses)
                    state = 1

                # wait for controller to empty target pose queue
                elif state == 1:
                    if not self._target_pose_queue:
                        self.get_logger().info('Move [%s]: Succeeded' %move_id)
                        goal_handle.succeed()
                        state = 99
                    else:
                        state = 1

                # shutdown/clean up
                elif state == 99:
                    self._target_pose_queue = []
                    # leave the state machine
                    break

                # shutdown state conditions
                # is new move waiting for execution?
                if move_id != self._current_move_id:
                    goal_handle.abort()
                    self.get_logger().info('Move [%s]: Aborted, overwritten by new move' %move_id)
                    state = 99
                # is cancel requested?
                elif goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Move [%s]: Canceled' %move_id)
                    state = 99
                # is control enabled?
                elif not self._control_enabled:
                    goal_handle.abort()
                    self.get_logger().info('Move [%s]: Aborted, control disabled' %move_id)
                    state = 99
                  
        return MoveTetherbot.Result()
        
    def cancel_move_callback(self, _):

        return CancelResponse.ACCEPT
    
    def goal_move_callback(self, _):

        return GoalResponse.ACCEPT
    
    def get_move_id(self) -> int:
        
        self._current_move_id = self._current_move_id + 1
        
        return self._current_move_id

    def array2poses(self, array: np.ndarray) -> list[Pose]:

        array = np.reshape(array, (-1,7))
        poses = []

        for row in array:
            pose = Pose()
            pose.position.x = row[0]
            pose.position.y = row[1]
            pose.position.z = row[2]
            pose.orientation.w = row[3]
            pose.orientation.x = row[4]
            pose.orientation.y = row[5]
            pose.orientation.z = row[6]

            poses.append(pose)

        return poses

