from __future__ import annotations

import rclpy
import rclpy.time
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from custom_srvs.srv import Tension
from custom_msgs.msg import BoolArray
from custom_actions.action import MoveMotor, Empty as EmptyAction
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Pose
from tbotlib import TransformMatrix
from threading import Lock
from .tetherbot_control_base_controller_node import BaseControllerNode

class PlatformControllerNode(BaseControllerNode):

    def __init__(self):

        super().__init__(node_name = 'platform_controller', 
                         default_motor_node_names = ['motor0/faulhaber_motor', 
                                                     'motor1/faulhaber_motor', 
                                                     'motor2/faulhaber_motor',
                                                     'motor3/faulhaber_motor',
                                                     'motor4/faulhaber_motor',
                                                     'motor5/faulhaber_motor',
                                                     'motor6/faulhaber_motor',
                                                     'motor7/faulhaber_motor',
                                                     'motor8/faulhaber_motor',
                                                     'motor9/faulhaber_motor',])
        
        # by default the gripper parent is the hold, but we set it to the world/map
        # this way T_local gets referenced to the world/map and not to the hold frame
        for gripper in self._tbot.grippers:
            gripper.parent = self._tbot

        # services
        self.create_service(Tension, self.get_name() + '/tension_gripper_tethers', self.tension_gripper_tethers_srv_callback)
        # publishers
        self._tether_tension_pub = self.create_publisher(BoolArray, self.get_name() + '/tether_tension', 1)
        # timers
        self.create_timer(0.5, self.pub_timer)
        # subscriptions
        for i in range(self._tbot.k):
            self.create_subscription(PoseStamped, self._tbot.grippers[i].name + '/gripper_state_publisher/pose', lambda msg, i=i: self.gripper_pose_sub_callback(msg, i), 1)
        # actions
        self.create_action_server(EmptyAction, self.get_name() + '/calibrate_tether_lengths', 
                                  execute_callback = self.calibrate_tether_lengths_execute_callback,
                                  cancel_callback = self.calibrate_tether_lengths_cancel_callback,
                                  goal_callback = self.calibrate_tether_lengths_goal_callback)
        self._move_motor_clis: list[ActionClient] = []
        for i in range(self._tbot.m):
            self._move_motor_clis.append(self.create_action_client(MoveMotor, 'motor' + str(i) + '/faulhaber_motor/move'))
        self._rate = self.create_rate(10)
        
        # length parameters of untensioned/tensioned tethers
        self._untension_length = 0.01 # in meter
        self._tension_length = -0.001

        # lock/flag to ensure only one action server runs at a given time
        self._lock = Lock()
        self._busy = False

    def control_function(self, target_pose: Pose) -> np.ndarray:

        # calculate joint states
        qs = self._tbot.ivk(TransformMatrix(self.pose2mat(target_pose)))
        qs = qs + self._untension_length * np.logical_not(self._tbot.tensioned) + self._tbot.tensioned * self._tension_length

        return qs
    
    def gripper_pose_sub_callback(self, msg: PoseStamped, i: int):

        self._tbot.grippers[i].T_local = TransformMatrix(self.pose2mat(msg.pose))
    
    def tension_gripper_tethers_srv_callback(self, request: Tension.Request, response = Tension.Response) -> Tension.Response:

        if request.value: # True -> tension tether
            self._tbot.tension(request.gripper_index, True)
            response.success = True
        else:
            if np.all(self._tbot.tensioned):
                self._tbot.tension(request.gripper_index, False)
                response.success = True
            else:
                self.get_logger().error('Tension gripper tethers: Tethers already slack')
                response.success = False

        return response

    def pub_timer(self):

        msg = BoolArray()
        msg.data = self._tbot.tensioned.astype(bool).tolist()

        self._tether_tension_pub.publish(msg)

    def calibrate_tether_lengths_execute_callback(self, goal_handle: ServerGoalHandle) -> Trigger.Response:

        state = 0

        while True:
            self._rate.sleep()

            # initialize
            if state == 0:
                cli_goal_handles: list[ClientGoalHandle] = []
                send_goal_futures: list[rclpy.Future] = []
                self.get_logger().info('Calibrate tether lengths: Start')
                self.get_logger().info('Calibrate tether lengths: Note: Transform source should not be fwk!')
                state = 1

            # wait for services and send goal
            elif state == 1:
                if all([cli.server_is_ready() for cli in self._move_motor_clis]):
                    qs = self._tbot.ivk(TransformMatrix(self.pose2mat(self._actual_pose))) 
                    for q, cli in zip(qs, self._move_motor_clis):
                        cli_goal = MoveMotor.Goal()
                        cli_goal.homing_offset = float(q)
                        cli_goal.mode = 4
                        send_goal_futures.append(cli.send_goal_async(cli_goal))
                    state = 2
                else:
                    state = 1

            # wait for goals to arrive
            elif state == 2:
                if all([future.done() for future in send_goal_futures]):
                    for future in send_goal_futures:
                        cli_goal_handle: ClientGoalHandle = future.result()
                        cli_goal_handles.append(cli_goal_handle)
                    state = 3
                else:
                    state = 2
            
            # wait for results
            elif state == 3:
                if all([cli_goal_handle.status == 4 for cli_goal_handle in cli_goal_handles]):
                    goal_handle.succeed()
                    self.get_logger().info('Calibrate tether lengths: Succeeded')
                    state = 99
                elif any([cli_goal_handle.status > 4 for cli_goal_handle in cli_goal_handles]):
                    goal_handle.abort()
                    self.get_logger().info('Calibrate tether lengths: Aborted, motor client canceled/aborted unexpectedly')
                    state = 99
                else:
                    state = 3

            # clean up
            elif state == 99:
                for future in send_goal_futures:
                    future.cancel()
                for cli_goal_handle in cli_goal_handles:
                    cli_goal_handle.cancel_goal_async()
                break

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Calibrate tether lengths: Canceled')
                goal_handle.canceled()
                state = 99
        
        with self._lock:
            self._busy = False

        return EmptyAction.Result()
    
    def calibrate_tether_lengths_cancel_callback(self, _):
        
        return CancelResponse.ACCEPT
    
    def calibrate_tether_lengths_goal_callback(self, _):

        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            else:
                self._busy = True
                return GoalResponse.ACCEPT

def main(args = None):

    rclpy.init(args = args)
    try:
        node = PlatformControllerNode()
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


""" if all([cli.server_is_ready() for cli in self._move_motor_clis]):
                    try:
                        tethervectors: list[Vector3] = []
                        for i in range(self._tbot.m):
                            tethervectors.append(self._tf_buffer.lookup_transform(
                                target_frame = self._tbot.tethers[i].anchorpoints[0].name,
                                source_frame = self._tbot.tethers[i].anchorpoints[1].name,
                                time = rclpy.time.Time()).transform.translation)
                    except:
                        self.get_logger().info('Callibrate tether lengths: Aborted, lookup transform failed')
                        goal_handle.abort()
                        state = 99
                    else:
                        for tethervector, cli in zip(tethervectors, self._move_motor_clis):
                            cli_goal = MoveMotor.Goal()
                            cli_goal.homing_offset = float(sqrt(tethervector.x**2 + tethervector.y**2 + tethervector.z**2))
                            cli_goal.mode = 4
                            send_goal_futures.append(cli.send_goal_async(cli_goal))
                        state = 2
                else:
                    state = 1 """