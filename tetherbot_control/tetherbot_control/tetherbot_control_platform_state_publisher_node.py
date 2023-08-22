from __future__ import annotations

import rclpy
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from zed_interfaces.srv import SetPose
from custom_msgs.msg import Float64Array, MotorPosition
from custom_srvs.srv import SetString
from .tetherbot_control_base_state_publisher_node import BaseStatePublisherNode
from tbotlib import TbTetherbot, TransformMatrix

class PlatformStatePublisherNode(BaseStatePublisherNode):

    def __init__(self):

        super().__init__(node_name = 'platform_state_publisher')

        # declare parameters
        self.declare_parameter('default_transform_source', 'optitrack')

        # set parameters
        self._tbot: TbTetherbot = TbTetherbot.load(self._config_file)
        self._joint_states = self._tbot.l
        self._zed_pose = PoseStamped()
        self._optitrack_pose = PoseStamped()
        self.set_transform_source(self.get_parameter('default_transform_source').get_parameter_value().string_value)

        # by default the gripper parent is the hold, but we set it to the world/map
        # this way T_local gets referenced to the world/map and not to the hold frame
        for gripper in self._tbot.grippers:
            gripper.parent = self._tbot

        # subscriptions
        self.create_subscription(PoseStamped, '/zedm/zed_node/pose', self.zed_pose_sub_callback, 1)
        self.create_subscription(PoseStamped, '/tetherbot_optitrack/pose', self.optitrack_pose_sub_callback, 1)
        for i in range(self._tbot.m):
            self.create_subscription(MotorPosition, 'motor' + str(i) + '/position', lambda msg, i=i: self.motor_position_sub_callback(msg, i), 1)
        for i in range(self._tbot.k):
            self.create_subscription(PoseStamped, self._tbot.grippers[i].name + '/gripper_state_publisher/pose', lambda msg, i=i: self.gripper_pose_sub_callback(msg, i), 1)
        # publishers
        self._pose_pub = self.create_publisher(PoseStamped, self.get_name() + '/pose', 1)
        self._transform_source_pub = self.create_publisher(String, self.get_name() + '/transform_source', 1)
        self._joint_states_pub = self.create_publisher(Float64Array, self.get_name() + '/joint_states', 1)
        # services
        self.create_service(SetString, self.get_name() + '/set_transform_source', self.set_transform_source_callback)
        self.create_service(Trigger, self.get_name() + '/calibrate_zed_pose', self.calibrate_zed_pose_callback)
        # clients
        self._set_pose_cli = self.create_client(SetPose, 'set_pose', callback_group = ReentrantCallbackGroup())
        # timer
        self.create_timer(0.2, self.timer_callback)
        # rate
        self._rate = self.create_rate(10)

    def timer_callback(self):

        # publish joint states
        msg = Float64Array()
        msg.data = self._joint_states.astype(float).tolist()
        self._joint_states_pub.publish(msg)

        # publish pose
        pose = PoseStamped()
        if self._transform_source == 'zed':
            pose = self._zed_pose
        elif self._transform_source == 'optitrack':
            pose = self._optitrack_pose
        elif self._transform_source == 'fwk':
            self._tbot.fwk(self._joint_states, self._tbot.platform.T_world) # use previous transform as first guess for fwk
            q = self._tbot.platform.T_world.q
            r = self._tbot.platform.T_world.r
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = r[0]
            pose.pose.position.y = r[1]
            pose.pose.position.z = r[2]
            pose.pose.orientation.w = q[0]
            pose.pose.orientation.x = q[1]
            pose.pose.orientation.y = q[2]
            pose.pose.orientation.z = q[3]
        self._pose_pub.publish(pose)

        # publish transform
        transform = TransformStamped()
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation.w = pose.pose.orientation.w
        transform.transform.rotation.x = pose.pose.orientation.x
        transform.transform.rotation.y = pose.pose.orientation.y
        transform.transform.rotation.z = pose.pose.orientation.z
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = self._tbot.platform.name
        self._tf_broadcaster.sendTransform(transform)

        # publish transform source
        msg = String()
        msg.data = self._transform_source
        self._transform_source_pub.publish(msg)

    def gripper_pose_sub_callback(self, msg: PoseStamped, i: int):

        self._tbot.grippers[i].T_local = TransformMatrix(self.pose2mat(msg.pose))

    def motor_position_sub_callback(self, msg: MotorPosition, i: int):

        self._joint_states[i] = msg.actual_position

    def zed_pose_sub_callback(self, msg: PoseStamped):
            
        self._zed_pose = msg

    def optitrack_pose_sub_callback(self, msg: PoseStamped):

        self._optitrack_pose = msg
        
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
                    T = TransformMatrix([self._zed_pose.pose.position.x,
                                         self._zed_pose.pose.position.y,
                                         self._zed_pose.pose.position.z,
                                         self._zed_pose.pose.orientation.w,
                                         self._zed_pose.pose.orientation.x,
                                         self._zed_pose.pose.orientation.y,
                                         self._zed_pose.pose.orientation.z])
                    cli_request.pos = T.decompose()[:3]
                    cli_request.orient = T.decompose()[3:]

                    future = self._set_pose_cli.call_async(cli_request)
                    state = 2
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
