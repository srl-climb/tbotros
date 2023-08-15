from __future__ import annotations

import rclpy
import rclpy.task
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Pose
from tbotlib import TransformMatrix
from .tetherbot_control_base_controller_node import BaseControllerNode

class ArmControllerNode(BaseControllerNode):

    def __init__(self):

        super().__init__(node_name = 'arm_controller', default_motor_node_names = ['motor10/faulhaber_motor', 'motor11/faulhaber_motor', 'motor12/faulhaber_motor'])

        # subscriptions for required transformations/joint states
        self.create_subscription(PoseStamped, self._tbot.platform.name + '/platform_state_publisher/pose', self.platform_pose_sub_callback, 1)
        
    def control_function(self, target_pose: Pose) -> np.ndarray:
  
        # calculate motor positions
        return self._tbot.platform.arm.ivk(TransformMatrix(self.pose2mat(target_pose)))
    
    def platform_pose_sub_callback(self, msg: PoseStamped):

        self._tbot.platform.T_local = TransformMatrix(self.pose2mat(msg.pose))

def main(args = None):

    rclpy.init(args = args)
    try:
        node = ArmControllerNode()
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


