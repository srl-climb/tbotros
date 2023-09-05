import rclpy
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory

'''
Calibrates the target tbot coordinate system for adaption of the offsets of estimated tbot cs
'''

class ROS2TetherbotCoordinateSystemCalibrationNode(Node):

    def __init__(self):
        super().__init__('tbot_center_correction_node')

        self.calib_srv = self.create_service(Empty, self.get_name() + '/tetherbot_coordinate_calibration_service', self.coordinate_calib_srv)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.offset_file = os.path.join(get_package_share_directory("tetherbot_optitrack"), "config/calibration_offsets.yaml")

        # Initialize Transform Message
        self.transform_msg = TransformStamped()
        self.transform_msg.header.frame_id = 'tbot_motive_origin'
        self.transform_msg.child_frame_id = 'tbot_center'

        # Load Offsets from file and save into tbot_center and tf msg
        self.load_offsets()

        # Send TF
        self.send_static_tf()

    def send_static_tf(self):
        self.tf_static_broadcaster.sendTransform(self.transform_msg)

    def load_offsets(self):

        try:
            with open(self.offset_file, "r") as file:
                self.tbot_center = yaml.safe_load(file.read())
                self.transform_msg.transform.translation.x = self.tbot_center["transform"]["translation"]["x"]
                self.transform_msg.transform.translation.y = self.tbot_center["transform"]["translation"]["y"]
                self.transform_msg.transform.translation.z = self.tbot_center["transform"]["translation"]["z"]
                self.transform_msg.transform.rotation.x = self.tbot_center["transform"]["rotation"]["x"]
                self.transform_msg.transform.rotation.y = self.tbot_center["transform"]["rotation"]["y"]
                self.transform_msg.transform.rotation.z = self.tbot_center["transform"]["rotation"]["z"]
                self.transform_msg.transform.rotation.w = self.tbot_center["transform"]["rotation"]["w"]

            self.get_logger().info('Calibration Offsets loaded!')

        except:
            self.get_logger().warning('Calibration Offsets could not be loaded! Using default values...')    

    def coordinate_calib_srv(self, request, response):
        tf = self.tf_buffer.lookup_transform('tbot_motive_origin', 'calibration_target', rclpy.time.Time())

        # Update Values for tbot_center

        try:
            self.transform_msg.transform.translation.x = tf.transform.translation.x
            self.transform_msg.transform.translation.y = tf.transform.translation.y
            self.transform_msg.transform.translation.z = tf.transform.translation.z
            self.transform_msg.transform.rotation.w = tf.transform.rotation.w
            self.transform_msg.transform.rotation.x = tf.transform.rotation.x
            self.transform_msg.transform.rotation.y = tf.transform.rotation.y
            self.transform_msg.transform.rotation.z = tf.transform.rotation.z

            self.send_static_tf()

        except:
            self.get_logger().warning("tbot_center could not be updated!")

        # Save new values to dictionary

        new_offsets = {
            'transform': {
                'translation': {
                    'x': tf.transform.translation.x,
                    'y': tf.transform.translation.y,
                    'z': tf.transform.translation.z
                },
                'rotation': {
                    'x': tf.transform.rotation.x,
                    'y': tf.transform.rotation.y,
                    'z': tf.transform.rotation.z,
                    'w': tf.transform.rotation.w
                }
            }
        }

        # Save dictionary to yaml file

        try:
            with open(self.offset_file, "w") as file:
                yaml.dump(new_offsets, file, default_flow_style=False)

            self.get_logger().info("Offset file has been updated!")

        except:
            self.get_logger().warning("Offset file could not be updated!")

        self.get_logger().info('Calibration Service finished!')
        return response
        

def main():
    rclpy.init()
    node = ROS2TetherbotCoordinateSystemCalibrationNode()

    try:
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
