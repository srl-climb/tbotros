from __future__ import annotations

import rclpy
import rclpy.time
from .tetherbot_control_base_state_publisher_node import BaseStatePublisherNode
from tbotlib import TbTetherbot
from geometry_msgs.msg import TransformStamped, PoseStamped
from custom_srvs.srv import SetString, GetPose
from std_msgs.msg import String

class GripperStatePublisherNode(BaseStatePublisherNode):

    def __init__(self):

        super().__init__(node_name='gripper_state_publisher')

        # declare parameters
        self.declare_parameter('gripper_id', '0')
        self.declare_parameter('hold_id', '0')
        self.declare_parameter('default_transform_source', 'hold')

        # set parameters
        self._tbot: TbTetherbot = TbTetherbot.load(self._config_file)
        self.set_gripper(self.get_parameter('gripper_id').get_parameter_value().string_value)
        self.set_hold(self.get_parameter('hold_id').get_parameter_value().string_value)
        self.set_transform_source(self.get_parameter('default_transform_source').get_parameter_value().string_value)

        # timer
        self.create_timer(1, self.timer_callback)
        # services
        self.create_service(SetString, self.get_name() + '/set_transform_source', self.set_transform_source_callback)
        self.create_service(SetString, self.get_name() + '/set_hold', self.set_hold_callback)
        self.create_service(GetPose, self.get_name() + '/get_pose', self.get_pose_callback)
        self.create_service(GetPose, self.get_name() + '/get_marker_pose', self.get_marker_pose_callback)
        # publishers
        self._pose_pub = self.create_publisher(PoseStamped, self.get_name() + '/pose', 1)
        self._transform_source_pub = self.create_publisher(String, self.get_name() + '/transform_source', 1)
        self._hold_name_pub = self.create_publisher(String, self.get_name() + '/hold_name', 1)
        # subscribers
        self.create_subscription(PoseStamped, 'marker_pose', self.marker_pose_callback, 1)

        self._tf_map_to_marker = TransformStamped()

    def timer_callback(self):

        # look up required transforms
        transform = self.get_transform()
    
        # publish transform
        if transform is not None:
            self._tf_broadcaster.sendTransform(transform)

        # publish pose
        pose = PoseStamped()
        if transform is not None:
            pose.header.frame_id = 'map'
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
        self._pose_pub.publish(pose)

        # publish transform source
        msg = String()
        msg.data = self._transform_source
        self._transform_source_pub.publish(msg)

        # publish hold name
        msg = String()
        msg.data = self._hold.name
        self._hold_name_pub.publish(msg)

    def marker_pose_callback(self, msg: PoseStamped):

        self._tf_map_to_marker.header.frame_id = 'map'
        self._tf_map_to_marker.child_frame_id = self._gripper.name
        self._tf_map_to_marker.header.stamp = msg.header.stamp
        self._tf_map_to_marker.transform.translation.x = msg.pose.position.x
        self._tf_map_to_marker.transform.translation.y = msg.pose.position.y
        self._tf_map_to_marker.transform.translation.z = msg.pose.position.z
        self._tf_map_to_marker.transform.rotation.w = msg.pose.orientation.w
        self._tf_map_to_marker.transform.rotation.x = msg.pose.orientation.x
        self._tf_map_to_marker.transform.rotation.y = msg.pose.orientation.y
        self._tf_map_to_marker.transform.rotation.z = msg.pose.orientation.z

    def set_transform_source_callback(self, request: SetString.Request, response: SetString.Response) -> SetString.Response:

        self.set_transform_source(request.data)

        return response
    
    def set_hold_callback(self, request: SetString.Request, response: SetString.Response) -> SetString.Response:

        self.set_hold(request.data)
        
        return response
    
    def get_pose_callback(self, request: GetPose.Request, response: GetPose.Response) -> GetPose.Response:

        transform = self.get_transform()

        if transform is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            response.succeeded = True
            response.pose = pose
        else:
            response.succeeded = False

        return response
    
    def get_marker_pose_callback(self, request: GetPose.Request, response: GetPose.Response) -> GetPose.Response:
        
        try:
            transform: TransformStamped = self._tf_buffer.lookup_transform(target_frame = 'map', 
                                                                           source_frame = self._gripper.marker.name,
                                                                           time = rclpy.time.Time())
        except Exception as exc:
            response.succeeded = False
            self.get_logger().error('Get marker pose failed: ' + str(exc))
        else:
            response.succeeded = True
            response.pose.header.frame_id = 'map'
            response.pose.header.stamp = transform.header.stamp
            response.pose.pose.position.x = transform.transform.translation.x
            response.pose.pose.position.y = transform.transform.translation.y
            response.pose.pose.position.z = transform.transform.translation.z
            response.pose.pose.orientation.w = transform.transform.rotation.w
            response.pose.pose.orientation.x = transform.transform.rotation.x
            response.pose.pose.orientation.y = transform.transform.rotation.y
            response.pose.pose.orientation.z = transform.transform.rotation.z

        return response

    def set_hold(self, value: str):

        try: 
            self._hold = self._tbot.wall.get_hold(value)
            self.get_logger().info('Setting hold to: ' + str(self._hold))
        except Exception as exc:
            self.get_logger().error('Failed setting hold: ' + str(exc))

    def set_gripper(self, value: str):

        try:
            self._gripper = self._tbot.get_gripper(value)
        except Exception as exc:
            self.get_logger().error('Failed setting gripper: ' + str(exc))

    def set_transform_source(self, value: str):

        if value in ['arm', 'marker', 'hold']:
            self._transform_source = value
        else:
            self.get_logger().error('Failed setting source type: ' + value)

    def get_transform(self) -> TransformStamped:

        # look up required transforms
        try:
            if self._transform_source == 'hold':           
                tf_map_to_source     = self._tf_listener.buffer.lookup_transform(target_frame = self._hold.grippoint.name,
                                                                                 source_frame = 'map',
                                                                                 time = rclpy.time.Time()).transform
                tf_gripper_to_source = self._tf_listener.buffer.lookup_transform(target_frame = self._gripper.grippoint.name,
                                                                                 source_frame = self._gripper.name,
                                                                                 time = rclpy.time.Time()).transform
            elif self._transform_source == 'arm':
                tf_map_to_source     = self._tf_listener.buffer.lookup_transform(target_frame = self._tbot.platform.arm.links[-1].name,
                                                                                 source_frame = 'map',
                                                                                 time = rclpy.time.Time()).transform
                tf_gripper_to_source = self._tf_listener.buffer.lookup_transform(target_frame = self._gripper.dockpoint.name,
                                                                                 source_frame = self._gripper.name,
                                                                                 time = rclpy.time.Time()).transform
            elif self._transform_source == 'marker':
                tf_map_to_source     = self.inverse_transform(self._tf_map_to_marker.transform)
                tf_gripper_to_source = self._tf_listener.buffer.lookup_transform(target_frame = self._gripper.marker.name,
                                                                                 source_frame = self._gripper.name,
                                                                                 time = rclpy.time.Time()).transform

            # calculate transformation of map to gripper
            # arm:    map -> gripper -> dock point = arm endeffector <- map
            # hold:   map -> gripper -> grip point = grip point <- hold <- map
            # marker: map -> gripper -> marker = marker <- map
            tf_map_to_gripper = self.relative_transform(tf_map_to_source, tf_gripper_to_source)

            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = self._gripper.name
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.transform = tf_map_to_gripper

            return transform
    
        except Exception as exc:
            self.get_logger().warn('Look up transform failed: ' + str(exc))

            return None
   
def main(args = None):

    rclpy.init(args = args)
    try:
        node = GripperStatePublisherNode()
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

