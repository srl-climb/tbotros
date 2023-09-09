from __future__ import annotations

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, CancelResponse
from custom_actions.action import Empty as EmptyAction
from std_msgs.msg import Bool
from std_srvs.srv import Empty as EmptyService
from threading import Lock

class GripperControllerNode(Node):

    def __init__(self):

        super().__init__('gripper_controller')

        ActionServer(self, EmptyAction, self.get_name() + '/open', execute_callback = self.open_execute_callback, cancel_callback = self.cancel_callback)
        ActionServer(self, EmptyAction, self.get_name() + '/close', execute_callback = self.close_execute_callback, cancel_callback = self.cancel_callback)

        self._servo_open_cli = ActionClient(self, EmptyAction, 'wireless_servo/open')
        self._servo_close_cli = ActionClient(self, EmptyAction, 'wireless_servo/close')
        
        self.create_service(EmptyService, self.get_name() + '/confirm_contact', self.confirm_contact_srv_callback)
        self.create_subscription(Bool, self.get_name() + '/contactswitch', self.contactswitch_sub_callback, 1)

        self._contact_confirmed_pub = self.create_publisher(Bool, self.get_name() + '/contact_confirmed', 1)

        self._rate = self.create_rate(5)
        self._lock = Lock()
        self._contactswitch = False
        self._user_confirmed = False
        self._timeout_in_sec = 20 # must be longer than action duration of servo
        self._current_action_id = -1

    def contactswitch_sub_callback(self, msg: Bool):

        self._contactswitch = msg.data

        contact_confirmed_msg = Bool()
        contact_confirmed_msg.data = self._user_confirmed or self._contactswitch

        self._contact_confirmed_pub.publish(contact_confirmed_msg)

    def confirm_contact_srv_callback(self, request: EmptyService.Request, response: EmptyService.Response) -> EmptyService.Response:

        self._user_confirmed  = True

        return response

    def open_execute_callback(self, goal_handle: ServerGoalHandle) -> EmptyAction.Result:

        return self.execute(goal_handle, 0)

    def close_execute_callback(self, goal_handle: ServerGoalHandle) -> EmptyAction.Result:

        return self.execute(goal_handle, 1)
    
    def cancel_callback(self, cancel_request):

        return CancelResponse.ACCEPT

    def execute(self, goal_handle: ServerGoalHandle, close: bool) -> EmptyAction.Result:

        action_id = self.get_action_id()

        with self._lock:
            state = 0
            
            while True:
                self._rate.sleep()
                # initialize
                if state == 0:
                    self.get_logger().info('Open/close action [%s]: Start' %action_id)
                    start_time = self.get_clock().now().seconds_nanoseconds()[0]

                    self._user_confirmed  = False   
                    send_goal_future: rclpy.Future = None  
                    if close:
                        servo_cli = self._servo_close_cli
                    else:
                        servo_cli = self._servo_open_cli
                    state = 1

                # warn if contact switch not yet triggered
                elif state == 1:
                    if self._contactswitch == 0:
                        self.get_logger().warn('Contact switch not triggered, contact or confirmation required before timeout to proceed')
                    state = 2

                # wait for contact 
                elif state == 2:
                    if self._user_confirmed or self._contactswitch or not close:
                        start_time = self.get_clock().now().seconds_nanoseconds()[0] # reset timeout
                        state = 3
                    else:
                        state = 2

                # send request to client
                elif state == 3:
                    if servo_cli.server_is_ready():
                        send_goal_future = servo_cli.send_goal_async(EmptyAction.Goal())
                        state = 4
                    else:
                        state = 3

                # wait for future
                elif state == 4:
                    if send_goal_future.done():
                        if send_goal_future.result().status == 4: # succeeded
                            goal_handle.succeed()
                            state = 99
                            self.get_logger().info('Open/close action [%s]: Completed' %action_id)
                        elif send_goal_future.result().status > 4: # canceld/aborted
                            goal_handle.abort()
                            state = 99
                            self.get_logger().info('Open/close action [%s]: Aborted, servo stopped unexpectedly' %action_id)
                    else:
                        state = 4

                # shutdown
                elif state == 99:
                    if send_goal_future is not None:
                        if send_goal_future.done():
                            send_goal_future.result().cancel_goal_async()
                        send_goal_future.cancel()
                    self._user_confirmed = False
                    break

                # exit conditions 
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    state = 99
                    self.get_logger().error('Open/close action [%s]: Canceled' %action_id)
                elif (self.get_clock().now().seconds_nanoseconds()[0] - start_time) > self._timeout_in_sec:
                    goal_handle.abort()
                    state = 99
                    self.get_logger().error('Open/close action [%s]: Aborted, timed out' %action_id)
                elif action_id != self._current_action_id:
                    goal_handle.abort()
                    state = 99
                    self.get_logger().info('Open/close action [%s]: Aborted, overwritten by new action' %action_id)
                        
            return EmptyAction.Result()

    def get_action_id(self) -> int:
        
        self._current_action_id = self._current_action_id + 1
        
        return self._current_action_id


def main(args = None):

    rclpy.init(args = args)
    try:
        node = GripperControllerNode()
        executor = MultiThreadedExecutor()
        try:
            rclpy.spin(node, executor = executor)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()