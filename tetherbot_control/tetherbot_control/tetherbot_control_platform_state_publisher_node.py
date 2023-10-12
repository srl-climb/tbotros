from __future__ import annotations

import rclpy
import rclpy.time
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float64
from custom_msgs.msg import Float64Stamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from zed_interfaces.srv import SetPose
from custom_msgs.msg import Float64Array, Float64Stamped
from custom_srvs.srv import SetString
from .tetherbot_control_base_state_publisher_node import BaseStatePublisherNode
from tbotlib import TbTetherbot, TransformMatrix

class PlatformStatePublisherNode(BaseStatePublisherNode):

    def __init__(self):

        super().__init__(node_name = 'platform_state_publisher')

        # declare parameters
        self.declare_parameter('default_transform_source', 'optitrack')
        self.declare_parameter('mode_2d', True)
        self.declare_parameter('fixed_z_value', 0.068)
        self.declare_parameter('optitrack_frame', 'none')
        self.declare_parameter('zed_frame', 'none')

        # set parameters
        self._joint_states = self._tbot.l
        self._zed_pose = PoseStamped()
        self._optitrack_pose = PoseStamped()
        self._transform_source = None
        self._mode_2d = self.get_parameter('mode_2d').get_parameter_value().bool_value
        self._fixed_z_value = self.get_parameter('fixed_z_value').get_parameter_value().double_value
        self._zed_frame = self.get_parameter('zed_frame').get_parameter_value().string_value
        self._optitrack_frame = self.get_parameter('optitrack_frame').get_parameter_value().string_value
        self.set_transform_source(self.get_parameter('default_transform_source').get_parameter_value().string_value)
        
        # by default the gripper parent is the hold, but we set it to the world/map
        # this way T_local gets referenced to the world/map and not to the hold frame
        for gripper in self._tbot.grippers:
            gripper.parent = self._tbot

        # subscriptions
        for i in range(self._tbot.m):
            self.create_subscription(Float64Stamped, 'motor' + str(i) + '/position', lambda msg, i=i: self.motor_position_sub_callback(msg, i), 1)

        # publishers
        self._pose_pub = self.create_publisher(PoseStamped, self.get_name() + '/pose', 1)
        self._transform_source_pub = self.create_publisher(String, self.get_name() + '/transform_source', 1)
        self._joint_states_pub = self.create_publisher(Float64Array, self.get_name() + '/joint_states', 1)
        self._stability_pub = self.create_publisher(Float64Stamped, self.get_name() + '/stability', 1)

        # services
        self.create_service(SetString, self.get_name() + '/set_transform_source', self.set_transform_source_callback)
        self.create_service(Trigger, self.get_name() + '/calibrate_zed_pose', self.calibrate_zed_pose_callback)

        # clients
        self._set_pose_cli = self.create_client(SetPose, 'set_pose', callback_group = ReentrantCallbackGroup())

        # timer
        self.create_timer(0.1, self.timer_callback)
        self.create_timer(0.5, self.slow_timer_callback)

        # rate
        self._rate = self.create_rate(10)

    def timer_callback(self):

        # publish joint states
        msg = Float64Array()
        msg.data = self._joint_states.astype(float).tolist()
        self._joint_states_pub.publish(msg)

        # calculate transform
        transform = TransformStamped()
        try:
            if self._transform_source == 'zed':
                transform = self._tf_buffer.lookup_transform(target_frame = 'map', source_frame = self._zed_frame, time = rclpy.time.Time())
            elif self._transform_source == 'optitrack':
                transform = self._tf_buffer.lookup_transform(target_frame = 'map', source_frame = self._optitrack_frame, time = rclpy.time.Time())
            elif self._transform_source == 'fwk':
                for gripper in self._tbot.grippers:
                    transform = self._tf_buffer.lookup_transform(target_frame = 'map', source_frame = gripper.name, time = rclpy.time.Time())
                    gripper.T_local = TransformMatrix(self.transform2mat(transform))
                transform_matrix = self._tbot.fwk(self._joint_states, self._tbot.platform.T_local) 
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.transform.translation.x = transform_matrix.x
                transform.transform.translation.y = transform_matrix.y
                transform.transform.translation.z = transform_matrix.z
                transform.transform.rotation.w = transform_matrix.q[0]
                transform.transform.rotation.x = transform_matrix.q[1]
                transform.transform.rotation.y = transform_matrix.q[2]
                transform.transform.rotation.z = transform_matrix.q[3]
            # 2d mode
            if self._mode_2d:
                transform_array = TransformMatrix(self.transform2mat(transform.transform)).decompose()
                transform_array[3] = 0 # set x rotation to 0
                transform_array[4] = 0 # set y rotation to 0
                transform_matrix = TransformMatrix(transform_array)

                transform.transform.translation.z = float(self._fixed_z_value)
                transform.transform.rotation.w = transform_matrix.q[0]
                transform.transform.rotation.x = transform_matrix.q[1]
                transform.transform.rotation.y = transform_matrix.q[2]
                transform.transform.rotation.z = transform_matrix.q[3]
            # set tetherbot platform to current transform
            self._tbot.platform.T_local = TransformMatrix(self.transform2mat(transform.transform))
        except Exception as exc:
            self.get_logger().error('Look up transform failed: ' + str(exc), skip_first = True, throttle_duration_sec = 3)

        transform.header.frame_id = 'map'
        transform.child_frame_id = self._tbot.platform.name
        self._tf_broadcaster.sendTransform(transform)

        # publish pose
        pose = PoseStamped()
        pose.pose.position.x = transform.transform.translation.x 
        pose.pose.position.y = transform.transform.translation.y 
        pose.pose.position.z = transform.transform.translation.z 
        pose.pose.orientation.w = transform.transform.rotation.w
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        self._pose_pub.publish(pose)

    def slow_timer_callback(self):

        # publish stability
        msg = Float64Stamped()
        msg.stamp = self.get_clock().now().to_msg()
        try:
            self.lookup_tbot_transforms()
            msg.data = float(self._tbot.stability(ignore_tether_lengths=True)[0])
        except Exception as exc:
            self.get_logger().error('Publish stability: ' + str(exc), throttle_duration_sec = 3, skip_first = True)
            msg.data = float(-1)
        self._stability_pub.publish(msg)

        # publish transform source
        msg = String()
        msg.data = self._transform_source
        self._transform_source_pub.publish(msg)

    def motor_position_sub_callback(self, msg: Float64Stamped, i: int):

        self._joint_states[i] = msg.data
        
    def set_transform_source_callback(self, request: SetString.Request, response: SetString.Response) -> SetString.Response:
        
        self.set_transform_source(request.data)

        return response
    
    def calibrate_zed_pose_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:

        state = 0
        timeout_sec = 2

        while True:
            self._rate.sleep()
            
            # initialize
            if state == 0:
                start_time = self.get_clock().now().seconds_nanoseconds()[0]
                state = 1

            # wait for action services, send request
            elif state == 1:
                if self._set_pose_cli.service_is_ready():
                    start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    cli_request = SetPose.Request()
                    try:
                        transform = self._tf_buffer.lookup_transform(target_frame = 'map', source_frame = self._optitrack_frame, time = rclpy.time.Time())
                        transform_matrix = TransformMatrix(self.transform2mat(transform.transform))
                        cli_request.pos = transform_matrix.decompose()[:3].astype(float).tolist()
                        cli_request.orient = np.radians(transform_matrix.decompose()[3:]).astype(float).tolist()
                        future = self._set_pose_cli.call_async(cli_request)
                        state = 2
                    except Exception as exc:
                        response.success = False
                        self.get_logger().error('Calibrate zed pose: Look up transform failed: ' + str(exc))
                        state = 99
                else:
                    state = 1

            # wait for results
            elif state == 2:
                if future.done():
                    response.success = future.result().success
                    state = 99
                else:
                    state = 2

            elif state == 99:
                break

            if self.get_clock().now().seconds_nanoseconds()[0] - start_time > timeout_sec:
                response.success = False
                self.get_logger().error('Calibrate zed pose: Time out')
                state = 99
        
        return response

    def set_transform_source(self, value: str):

        if value in ['zed', 'optitrack', 'fwk']:
            self._transform_source = value
        else:
            self.get_logger().error('Failed setting source type: ' + value)


def main(args = None):

    rclpy.init(args = args)
    try:
        node = PlatformStatePublisherNode()
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
