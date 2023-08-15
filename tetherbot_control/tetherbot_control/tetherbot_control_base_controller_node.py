from __future__ import annotations

import uuid
import rclpy.task
import rclpy.executors
import numpy as np

from rclpy.node import Publisher
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Lock
from custom_msgs.msg import MotorPosition
from custom_actions.action import MoveMotor, MoveTetherbot
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from tbotlib import TbTetherbot
from .tetherbot_control_base_node import BaseNode


class BaseControllerNode(BaseNode):

    def __init__(self, default_motor_node_names: list[str] = None, **kwargs):

        super().__init__(**kwargs)

        ### LOAD CONFIG ###

        # tbotlib object for kinematics
        self._tbot: TbTetherbot = TbTetherbot.load(self._config_file)

        ### ROS OBJECTS ###

        # action server
        ActionServer(self, MoveTetherbot, self.get_name() + '/move', 
                     execute_callback = self.execute_move_callback,
                     goal_callback = self.goal_move_callback,
                     cancel_callback = self.cancel_move_callback,
                     callback_group = ReentrantCallbackGroup())
        
        # action clients
        self._motor_action_clis: list[ActionClient] = []
        for name in default_motor_node_names:
            self._motor_action_clis.append(ActionClient(self, MoveMotor, name + '/move', callback_group = ReentrantCallbackGroup()))
        
        # services
        self.create_service(Empty, self.get_name() + '/enable_control', self.enable_control_callback)
        self.create_service(Empty, self.get_name() + '/disable_control', self.disable_control_callback)

        # publishers
        self._target_pose_pub = self.create_publisher(Pose, self.get_name() + '/target_pose', 1)
        self._control_enabled_pub = self.create_publisher(Bool, self.get_name() + '/control_enabled', 1)
        self.create_timer(0.2, self.timer_callback)

        # control loop timer
        self._control_loop_rate = 0.02 # in seconds
        self.create_timer(self._control_loop_rate, self.control_loop, callback_group = MutuallyExclusiveCallbackGroup())

        # rate for waiting and monitoring in the action server executor
        self._rate = self.create_rate(10, self.get_clock())

        # lock for protecting execute_move_callback
        self._lock = Lock()

        ### ADDITIONAL STUFF ###

        # variable used by execute_move_callback to determine if a new move was started
        self._current_move_id = int(-1)

        # parameters of the control loop
        self._target_pose  = None
        self._target_pose_queue = []
        self._control_enabled = False

    def timer_callback(self):

        if self._target_pose is None:
            msg = Pose()
        else:
            msg = self._target_pose
        self._target_pose_pub.publish(msg)

        msg = Bool()
        msg.data = self._control_enabled
        self._control_enabled_pub.publish(msg)


    def control_loop(self):

        if self._control_enabled:
            # get the next target pose if available
            if self._target_pose_queue:
                self._target_pose = self._target_pose_queue.pop(0)

            if self._target_pose is not None:
                
                # calculate target position of the motors
                target_positions = self.control_function(self._target_pose)

                # publish target positions to the motors
                for target_position_pub, q in zip(self._target_position_pubs, target_positions):
                    msg = MotorPosition()
                    msg.target_position = q

                    target_position_pub.publish(msg)

    def control_function(self, target_pose: Pose) -> np.ndarray:

        pass

    def enable_control_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:

        self._control_enabled = True

        return response
    
    def disable_control_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:

        self._control_enabled = False

        return response

    def execute_move_callback(self, goal_handle: ServerGoalHandle):
        
        move_id = self.get_move_id()

        with self._lock:
            self.get_logger().info('Move [%s]: Start' %move_id)
       
            # execute state machine
            state = 0
            
            while True:
                self._rate.sleep()

                # initialize state machine variables
                if state == 0:
                    self._target_pose_queue = []
                    self._target_position_pubs: list[Publisher] = []
                    send_goal_futures: list[rclpy.Future] = []
                    state = 1

                # create temporary publishers to send positions
                elif state == 1:
                    for _ in self._motor_action_clis:
                        self._target_position_pubs.append(self.create_publisher(MotorPosition, 
                                                                                '/target_position_' + str(uuid.uuid4()).replace('-', ''), 
                                                                                100)) # use absolute topic name here!
                    state = 2

                # send goals to motor action servers 
                elif state == 2:
                    for motor_action_cli, target_position_pub in zip(self._motor_action_clis, self._target_position_pubs):
                        goal = MoveMotor.Goal()
                        goal.mode = 2
                        goal.target_position_topic = target_position_pub.topic_name             
                        send_goal_futures.append(motor_action_cli.send_goal_async(goal))
                    state = 3

                # wait for servers to become available
                elif state == 3:
                    if all([future.done() for future in send_goal_futures]):
                        state = 4
                    else:
                        state = 3

                # wait for motors to subscribe
                elif state == 4:
                    if all([pub.get_subscription_count() > 1 for pub in self._target_position_pubs]):
                        state = 5
                    else:
                        state = 4

                # create temporary subscription to receive poses
                elif state == 5:
                    self._target_pose_queue = self.array2poses(goal_handle.request.poses)
                    
                    state = 6
                    # NOTE: Subscription is created last to avoid receiving messages before they can be send to the motors

                # wait for controller to empty target pose queue
                elif state == 6:
                    if self._target_pose_queue:
                        state = 6
                    else:
                        state = 7

                # shutdown/clean up
                elif state == 7:
                    self._target_pose_queue = []
                    for future in send_goal_futures:
                        if future.done():
                            future.result().cancel_goal_async()
                        future.cancel()
                    for target_position_pub in self._target_position_pubs:
                        target_position_pub.destroy()
                    # leave the state machine
                    break

                # shutdown state conditions
                # is new move waiting for execution?
                if move_id != self._current_move_id:
                    goal_handle.abort()
                    self.get_logger().info('Move [%s]: Aborted' %move_id)
                    state = 7
                # is motor action terminated unexpectedly?
                for future in send_goal_futures:
                    if future.done():
                        if future.result().status > 3:
                            goal_handle.abort()
                            self.get_logger().error('Move [%s]: Aborted, motor action unexpectedly canceled/aborted' %move_id)
                            state = 7
                # is cancel requested?
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Move [%s]: Canceled' %move_id)
                    state = 7

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

        for a in array:
            pose = Pose()
            pose.position.x = a[0]
            pose.position.y = a[1]
            pose.position.z = a[2]
            pose.orientation.w = a[3]
            pose.orientation.x = a[4]
            pose.orientation.y = a[5]
            pose.orientation.z = a[6]

            poses.append(pose)

        return poses

