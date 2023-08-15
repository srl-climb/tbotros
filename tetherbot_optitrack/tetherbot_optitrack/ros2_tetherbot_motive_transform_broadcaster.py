import rclpy
from tf2_ros.transform_broadcaster import TransformBroadcaster
from mocap_msgs.msg import RigidBodies
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

'''
Transforms the Pose data of a Rigid Body sent by Motive into a coordinate system that represents the origin of the tetherbot in motive
'''

class ROS2TetherbotMotiveTransformBroadcasterNode(Node):

    def __init__(self):
        super().__init__('tetherbot_motive_transform_broadcaster')

        self.tether_pose_subscriber = self.create_subscription(RigidBodies, '/rigid_bodies', self.tbot_pose_callback, 1)

        self.transform_msg = TransformStamped()
        self.transform_msg.header.frame_id = 'motive_origin'
        self.transform_msg.child_frame_id = 'tbot_motive_origin'
    
    def tbot_pose_callback(self, msg):

        self.transform_msg.transform.translation.x = msg.rigidbodies[0].pose.position.x
        self.transform_msg.transform.translation.y = msg.rigidbodies[0].pose.position.y
        self.transform_msg.transform.translation.z = msg.rigidbodies[0].pose.position.z
        self.transform_msg.transform.rotation.w = msg.rigidbodies[0].pose.orientation.x
        self.transform_msg.transform.rotation.x = msg.rigidbodies[0].pose.orientation.y
        self.transform_msg.transform.rotation.y = msg.rigidbodies[0].pose.orientation.z
        self.transform_msg.transform.rotation.z = msg.rigidbodies[0].pose.orientation.w

        TransformBroadcaster(self).sendTransform(self.transform_msg)


def main():
    rclpy.init()
    node = ROS2TetherbotMotiveTransformBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
