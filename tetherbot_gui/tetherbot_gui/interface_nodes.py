from __future__ import annotations
from multiprocessing import Queue, Event
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
import uuid
import rclpy

class SubscriptionNode(Node):
    # A class representing a subscription node

    def __init__(self, msg_name: str, msg_type: type, timeout_sec: float = 60):
        # Initialize the superclass node
        super().__init__('gui_subscription_node_' + str(uuid.uuid4()).replace('-',''))
        # Attach uuid to node name to ensure that the node has a unique name

        # Create a subscription to a specific message topic and bind a callback function to it
        self._subscription = self.create_subscription(msg_type, msg_name, self.subscription_callback, 1)
        
        # Set the message queue for the subscription node
        self.msg_queue = Queue(1)   

        # timeout in seconds until the topic is considered inactive
        self._timeout_sec = timeout_sec
        self._msg_received = False

        # timeout timer
        #if self._timeout_sec > 0:
        #    self.create_timer(self._timeout_sec, self.timeout_callback)

    def subscription_callback(self, msg):
        # Callback function for the subscription

        # Put the received message in the output queue if the queue is empty
        if self.msg_queue.qsize() == 0:
            self.msg_queue.put(msg, timeout = 2)

        self._msg_received = True

    def timeout_callback(self):

        if not self._msg_received:
            self.get_logger().error('Topic ' + self._subscription.topic_name + ' not active for ' + str(self._timeout_sec) + 's')
        else:
            self._msg_received = False

           
class ClientNode(Node):
    # A class representing a client node

    def __init__(self, srv_name: str, srv_type: type, enable_response: bool = True):
        # Initialize the superclass node
        super().__init__('gui_client_node_' + str(uuid.uuid4()).replace('-',''))
        # Attach uuid to node name to ensure that the node has a unique name
        
        # Create a client for a specific service
        self._client = self.create_client(srv_type, srv_name)
        
        # Create a timer and bind a callback function to it
        self._timer = self.create_timer(0.5, self.timer_callback)
        
        # Set the response queue for the client node
        self.res_queue = Queue(10)
        # Set the request queue for the client node
        self.req_queue = Queue(10)
        
        self._busy = False
        self._future = None
        self._state = 0
        self._enable_response = enable_response

    def timer_callback(self):
        # state machine

        # wait for request
        if self._state == 0:
            if not self.req_queue.empty():
                if self._client.service_is_ready():
                    self._future = self._client.call_async(self.req_queue.get())
                    self._busy = True
                    self._state = 1
                else:
                    self.req_queue.get()
                    self.get_logger().error('Service ' + self._client.srv_name + ' not ready')
                    self._state = 0
            else:
                self._state = 0

        # wait for result
        elif self._state == 1:
            if self._future.done():
                if self._enable_response:
                    self.res_queue.put(self._future.result(), timeout = 2)
                self._state = 2
            else:
                self._state = 1

        # clean up
        elif self._state == 2:
            self._busy = False
            self._future = None
            self._state = 0


class ActionClientNode(Node):

    def __init__(self, action_name: str, action_type: type, auto_cancel: bool = False, enable_feedback: bool = False, enable_result: bool = False, enable_status: bool = True):
        # auto_cancel: If true, an ongoing action will be automatically canceled once a new goal arrives, the new goal is send afterwards
        #              If false, an ongoing action will not be canceled once a new goal arrives, the new goal is send immediatly
        # NOTE: With auto_cancel disabled, the user will not be able to cancel the previous action once a new goal ist send, 
        # it should thus be used with actions which automatically terminate once a new goal arrives
        
        super().__init__('gui_actionclient_node_' + str(uuid.uuid4()).replace('-',''))

        self._client = ActionClient(self, action_type, action_name)

        self._timer = self.create_timer(0.5, self.timer_callback)

        self.goal_queue = Queue(10)
        self.result_queue = Queue(10)
        self.feedback_queue = Queue(3)
        self.status_queue = Queue(3)
        self.cancel_event = Event()

        self._busy = False
        self._state = 0
        self._send_goal_future: rclpy.Future = None
        self._goal_handle: ClientGoalHandle = None
        self._get_result_future: rclpy.Future = None
        self._cancel_future: rclpy.Future = None
        self._auto_cancel = auto_cancel
        self._enable_result = enable_result
        self._enable_feedback = enable_feedback
        self._enable_status = enable_status

    def timer_callback(self):
        # state machine

        # wait for goal
        if self._state == 0:
            if not self.goal_queue.empty():
                if self._client.server_is_ready():
                    self.cancel_event.clear()
                    self._busy = True
                    self._send_goal_future = self._client.send_goal_async(self.goal_queue.get(), feedback_callback = self.feedback_callback)
                    self._state = 1
                else:
                    self.goal_queue.get()
                    self.get_logger().error('Action server ' + self._client._action_name + ' not ready')
                    self._state = 0
            else: 
                self._state = 0

        # wait for send goal
        elif self._state == 1:
            if self._send_goal_future.done():
                self._goal_handle = self._send_goal_future.result()
                self._get_result_future = self._goal_handle.get_result_async()
                self._state = 2
            else:
                self._state = 1

        # wait for result
        elif self._state == 2:
            if self._get_result_future.done():
                if self._enable_result:
                    self.result_queue.put(self._get_result_future.result().result, timeout = 2)
                self._state = 99
            else:
                self._state = 2

        # cancel
        elif self._state == 97:
            if self._send_goal_future is not None:
                self._send_goal_future.cancel()
            if self._goal_handle is not None:
                self._cancel_future = self._goal_handle.cancel_goal_async()
                self._state = 98
            else:
                self._state = 99
            self._busy = False
            

        # wait for cancel
        elif self._state == 98:
            if self._cancel_future.done():
                self._state = 99
            else:
                self._state = 98
            #NOTE: If we do not wait for cancel, the goal handle will be destroyed before status changes to 'canceled'

        # shutdown, clean up
        elif self._state == 99:
            self._send_goal_future = None
            self._goal_handle = None
            self._get_result_future = None
            self._cancel_future = None
            self._busy = False
            self._state = 0

        # exit conditions
        # check for cancel event
        if self.cancel_event.is_set() and self._busy:
            self._state = 97
        # check for new goal in goal queue
        elif self._busy and not self.goal_queue.empty():
            if self._auto_cancel:
                self._state = 97
            else:
                self._state = 99
        
        # status queue
        if self._goal_handle is not None and self._enable_status and self.status_queue.empty():
            self.status_queue.put(self._goal_handle.status)

    def feedback_callback(self, msg):

        if self._enable_feedback and self.feedback_queue.empty():
            self.feedback_queue.put(msg.feedback)





