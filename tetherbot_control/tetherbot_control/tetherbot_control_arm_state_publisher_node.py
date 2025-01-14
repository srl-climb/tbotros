from __future__ import annotations

import rclpy
import rclpy.time
import numpy as np
from tbotlib import TbTetherbot
from geometry_msgs.msg import TransformStamped, PoseStamped
from custom_msgs.msg import Float64Stamped, Float64Array
from .tetherbot_control_base_state_publisher_node import BaseStatePublisherNode

class ArmStatePublisherNode(BaseStatePublisherNode):

    def __init__(self):

        super().__init__(node_name = 'arm_state_publisher')

        # tb arm object for forward kinematics
        self._arm = self._tbot.platform.arm
        self._joint_states = np.array([0,0,0], dtype=float) # in [deg, m, m]
        
        # subscriptions
        for i in range(3):
            self.create_subscription(Float64Stamped, 'motor' + str(i) + '/position', lambda msg, i=i: self.motor_position_sub_callback(msg, i), 1)
        # publishers
        self._pose_pub = self.create_publisher(PoseStamped, self.get_name() + '/pose', 1)
        self._joint_states_pub = self.create_publisher(Float64Array, self.get_name() + '/joint_states', 1)
        # timer
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        
        self._arm.qs = self._joint_states * [np.pi/180, 1, 1] # arm qs is in [rad, m, m]

        # publish joint states
        msg = Float64Array()
        msg.data = self._joint_states.astype(float).tolist()
        self._joint_states_pub.publish(msg)

        # publish transforms
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        # first joint
        tf.transform.rotation = self.array2quaternion(self._arm.links[0].T_local.q)
        tf.transform.translation = self.array2vector3(self._arm.links[0].T_local.r)
        tf.header.frame_id = self._arm.name
        tf.child_frame_id = self._arm.links[0].name
        self._tf_broadcaster.sendTransform(tf) 
        # second joint
        tf.transform.rotation = self.array2quaternion(self._arm.links[1].T_local.q)
        tf.transform.translation = self.array2vector3(self._arm.links[1].T_local.r)
        tf.header.frame_id = self._arm.links[0].name
        tf.child_frame_id = self._arm.links[1].name
        self._tf_broadcaster.sendTransform(tf) 
        # third joint 
        tf.transform.rotation = self.array2quaternion(self._arm.links[2].T_local.q)
        tf.transform.translation = self.array2vector3(self._arm.links[2].T_local.r)
        tf.header.frame_id = self._arm.links[1].name
        tf.child_frame_id = self._arm.links[2].name
        self._tf_broadcaster.sendTransform(tf) 

        # publish end effector pose
        pose = PoseStamped()
        try:  
            tf: TransformStamped = self._tf_buffer.lookup_transform(target_frame = 'map', source_frame = self._arm.links[2].name, time = rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn('Look up transform failed: ' + str(exc), skip_first = True, throttle_duration_sec = 3)
        else:   
            pose.header.frame_id = 'map'
            pose.header.stamp = tf.header.stamp
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation.w = tf.transform.rotation.w
            pose.pose.orientation.x = tf.transform.rotation.x
            pose.pose.orientation.y = tf.transform.rotation.y
            pose.pose.orientation.z = tf.transform.rotation.z
        self._pose_pub.publish(pose)

    def motor_position_sub_callback(self, msg: Float64Stamped, i: int):

        self._joint_states[i] = msg.data

    
def main(args = None):

    rclpy.init(args = args)
    try:
        node = ArmStatePublisherNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
