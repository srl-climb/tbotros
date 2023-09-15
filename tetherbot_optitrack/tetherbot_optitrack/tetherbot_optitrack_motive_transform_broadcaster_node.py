import rclpy
from tf2_ros.transform_broadcaster import TransformBroadcaster
from mocap_msgs.msg import RigidBodies
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

'''
Transforms the Pose data of a Rigid Body sent by Motive into a coordinate system that represents the origin of the tetherbot in motive
'''

class MotiveTransformBroadcasterNode(Node):

    def __init__(self):
        super().__init__('motive_transform_broadcaster')

        self.create_subscription(RigidBodies, '/optitrack/rigid_bodies', self.tbot_pose_callback, 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.transform_msg = TransformStamped()
        self.transform_msg.header.frame_id = 'motive_origin'
        self.transform_msg.child_frame_id = 'tbot_motive_origin'
    
    def tbot_pose_callback(self, msg):
        
        try:
            self.transform_msg.header.stamp = self.get_clock().now().to_msg()
            self.transform_msg.transform.translation.x = msg.rigidbodies[0].pose.position.x
            self.transform_msg.transform.translation.y = msg.rigidbodies[0].pose.position.y
            self.transform_msg.transform.translation.z = msg.rigidbodies[0].pose.position.z
            self.transform_msg.transform.rotation.w = msg.rigidbodies[0].pose.orientation.x
            self.transform_msg.transform.rotation.x = msg.rigidbodies[0].pose.orientation.y
            self.transform_msg.transform.rotation.y = msg.rigidbodies[0].pose.orientation.z
            self.transform_msg.transform.rotation.z = msg.rigidbodies[0].pose.orientation.w

            self.tf_broadcaster.sendTransform(self.transform_msg)
        
        except:
            self.get_logger().warning('No rigid body available!', throttle_duration_sec=2)



def main():
    rclpy.init()
    node = MotiveTransformBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()
