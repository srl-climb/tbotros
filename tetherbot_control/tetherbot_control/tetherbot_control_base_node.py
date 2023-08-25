from __future__ import annotations

import numpy as np

from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose
import quaternion as qu

#sys.path.append(os.path.join(get_package_prefix('tetherbot_control'), 'lib/python3.8/site-packages/tetherbot_control'))
# NOTE: tbotlib's load function is based on pickle, which has to import tbotlib in order to function
#       we add the path to the tbotlib inside the tetherbot_control install to make tbotlib importable

class BaseNode(Node):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)

        self.declare_parameter('config_file', '/home/srl-orin/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot_light.pkl')
        self._config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.get_logger().info('config file: ' + str(self._config_file))
        # tf buffer
        self._tf_buffer = Buffer()
        # tf listener
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # tf broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)


    def relative_transform(self, transform0: Transform, transform1: Transform) -> Transform:
        # calculate the relative transform 0->1 of two transforms 0 and 1 in the same coordinate frame

        # convert to transformation matrix
        T0 = self.transform2mat(transform0)
        T1 = self.transform2mat(transform1)
        
        # invert T1
        T0[:3,:3] = T0[:3,:3].T
        T0[:3,3] = -T0[:3,:3] @ T0[:3,3]

        # calculate relative transform
        T = T0 @ T1
        
        # convert to Transform object
        transform = self.mat2transform(T)

        return transform
    
    def inverse_transform(self, transform: Transform):
        # calculate inverse transform

        T = self.transform2mat(transform)
        
        # invert T1
        T[:3,:3] = T[:3,:3].T
        T[:3,3] = -T[:3,:3] @ T[:3,3]

        inverse = self.mat2transform(T)

        return inverse

    def transform2mat(self, transform: Transform) -> np.ndarray:

        T = np.eye(4)
        T[:3,:3] = qu.as_rotation_matrix(qu.from_float_array([transform.rotation.w, 
                                                              transform.rotation.x, 
                                                              transform.rotation.y,
                                                              transform.rotation.z]))
        T[:3,3]  = np.array([transform.translation.x,
                             transform.translation.y,
                             transform.translation.z])
        
        return T
    
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

    
    def mat2transform(self, T: np.ndarray) -> Transform:

        q = qu.as_float_array(qu.from_rotation_matrix(T[:3,:3]))

        transform = Transform()
        transform.translation.x = T[0,3]
        transform.translation.y = T[1,3]
        transform.translation.z = T[2,3]
        transform.rotation.w = q[0]
        transform.rotation.x = q[1]
        transform.rotation.y = q[2]
        transform.rotation.z = q[3]

        return transform

    @staticmethod
    def array2quaternion(array: np.ndarray) -> Quaternion:

        quaternion = Quaternion()
        quaternion.w = array[0]
        quaternion.x = array[1]
        quaternion.y = array[2]
        quaternion.z = array[3]

        return quaternion
    
    @staticmethod
    def array2vector3(array: np.ndarray) -> Vector3:

        vector3 = Vector3()
        vector3.x = array[0]
        vector3.y = array[1]
        vector3.z = array[2]

        return vector3
