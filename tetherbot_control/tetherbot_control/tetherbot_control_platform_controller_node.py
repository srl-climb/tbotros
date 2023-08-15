from __future__ import annotations

import rclpy
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from custom_srvs.srv import Tension
from custom_msgs.msg import BoolArray
from geometry_msgs.msg import PoseStamped, Pose
from tbotlib import TransformMatrix
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

        # service for tensioning a grippers' tethers
        self.create_service(Tension, self.get_name() + '/tension_gripper_tethers', self.tension_gripper_tethers_srv_callback)

        # publisher for tethers tension
        self._tether_tension_pub = self.create_publisher(BoolArray, self.get_name() + '/tether_tension', 1)
        self.create_timer(0.5, self.pub_timer)

        # subscription to gripper poses
        for i in range(self._tbot.k):
            self.create_subscription(PoseStamped, self._tbot.grippers[i].name + '/gripper_state_publisher/pose', lambda msg: self.gripper_pose_sub_callback(msg, i), 1)

        # slack parameters of untensioned tethers
        self._slack_target = 0.01 # in meter
        self._slack_actual = 0 # in meter
        self._slack_velocity = 0.003 # in meter/s
        self._slack_enabled = False

    def control_function(self, target_pose: Pose) -> np.ndarray:

        # calculate motor positions
        return self.apply_slack(self._tbot.ivk(TransformMatrix(self.pose2mat(target_pose))))
    
    def gripper_pose_sub_callback(self, msg: PoseStamped, i: int):

        self._tbot.grippers[i].T_local = TransformMatrix(self.pose2mat(msg.pose))
    
    def tension_gripper_tethers_srv_callback(self, request: Tension.Request, response = Tension.Response) -> Tension.Response:

        if request.value: # True -> tension tether
            self._tbot.tension(request.gripper_index, True)
            self._slack_enabled = False
            response.success = True
        else:
            if np.all(self._tbot.tensioned):
                self._tbot.tension(request.gripper_index, False)
                self._slack_enabled = True
                response.success = True
            else:
                self.get_logger().error('Tension gripper tethers: Tethers already slack')
                response.success = False

        return response

    def pub_timer(self):

        msg = BoolArray()

        msg.data = self._tbot.tensioned.astype(bool).tolist()

        self._tether_tension_pub.publish(msg)

    def apply_slack(self, qs: np.ndarray):
        
        # ramp up/down actual slack value between 0 and slack_target
        if self._slack_enabled:
            self._slack_actual = np.clip(self._slack_actual + self._control_loop_rate * self._slack_velocity, 0, self._slack_target)
        else:
            self._slack_actual = np.clip(self._slack_actual - self._control_loop_rate * self._slack_velocity, 0, self._slack_target)

        qs = qs + self._slack_actual * np.logical_not(self._tbot.tensioned)


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


