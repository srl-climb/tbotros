from __future__ import annotations

import rclpy
import rclpy.task
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from custom_msgs.msg import Float64Array
from std_srvs.srv import Empty
from tbotlib import TransformMatrix
from .tetherbot_control_base_controller_node import BaseControllerNode

class ArmControllerNode(BaseControllerNode):

    def __init__(self):

        super().__init__(node_name = 'arm_controller', default_motor_node_names = ['motor10/faulhaber_motor', 'motor11/faulhaber_motor', 'motor12/faulhaber_motor'])

        # subscriber
        self.create_subscription(Float64Array, self.get_name() + '/joint_states', self.arm_joint_states_callback, 1)

    def control_function(self, target_pose: Pose) -> np.ndarray:

        # calculate joint states (deg, m, m)
        qs = self._tbot.platform.arm.ivk(TransformMatrix(self.pose2mat(target_pose)))
        qs[0] = qs[0] * (180/np.pi)                                               
        
        return qs 
    
    def enable_control_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:

        self._target_pose = self.mat2pose(self._tbot.platform.arm.links[-1].T_world.T)

        return super().enable_control_callback(request, response)
    
    def arm_joint_states_callback(self, msg: Float64Array):

        qs = np.array(msg.data)
        qs[0] = qs[0] * (np.pi/180) 

        self._tbot.platform.arm.qs = qs
        

def main(args = None):

    rclpy.init(args = args)
    try:
        node = ArmControllerNode()
        try:
            rclpy.spin(node, executor = MultiThreadedExecutor())
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


