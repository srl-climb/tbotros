from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

'''
Transforms the origin of the tbot_center coordinate frame into
map coordinate frame and publishes it as a PoseStamped message into a topic
'''

class PosePublisherNode(Node):

    def __init__(self):
        super().__init__('pose_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.timer_callback)        
        self.platform_pose_publisher = self.create_publisher(PoseStamped, self.get_name() + '/platform_pose', 1)

        self.get_logger().info('Tetherbot Optitrack Pose is being published!')


    def timer_callback(self):

        try:
            tf = self.tf_buffer.lookup_transform('map', 'tbot_center', rclpy.time.Time())

            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = tf.transform.translation.x
            msg.pose.position.y = tf.transform.translation.y
            msg.pose.position.z = tf.transform.translation.z
            msg.pose.orientation.x = tf.transform.rotation.x
            msg.pose.orientation.y = tf.transform.rotation.y
            msg.pose.orientation.z = tf.transform.rotation.z
            msg.pose.orientation.w = tf.transform.rotation.w

            self.platform_pose_publisher.publish(msg)

        except TransformException:
            self.get_logger().warning('Transformation from map to tbot_center not available!', throttle_duration_sec = 3, skip_first = True)

def main():
    rclpy.init()
    node = PosePublisherNode()
    try:
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


