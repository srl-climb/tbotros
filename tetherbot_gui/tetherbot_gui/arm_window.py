from __future__ import annotations
import tkinter as tk
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty as EmptyService
from custom_msgs.msg import Float64Array
from custom_actions.action import Empty as EmptyAction
from geometry_msgs.msg import Pose, PoseStamped
from .window import Window
from typing import TYPE_CHECKING, Dict, List

if TYPE_CHECKING:
    from .tetherbot_gui import App


class ArmWindow(Window):

    def __init__(self, parent: App, title: str = 'Arm Window'):

        super().__init__(parent, title)

    def create_user_interface(self):

        self._actual_pose_node = self.create_subscriber_node(msg_type = PoseStamped, 
                                                             msg_name = self.parent.tbot.platform.arm.name + '/arm_state_publisher/pose')
        self._target_pose_node = self.create_subscriber_node(msg_type = Pose, 
                                                             msg_name = self.parent.tbot.platform.arm.name + '/arm_controller/target_pose')
        self._joint_states_node = self.create_subscriber_node(msg_type = Float64Array, 
                                                              msg_name = self.parent.tbot.platform.arm.name + '/arm_state_publisher/joint_states')
        self._control_enabled_node = self.create_subscriber_node(msg_type = Bool, 
                                                                 msg_name = self.parent.tbot.platform.arm.name + '/arm_controller/control_enabled')
        self._servo_close_node = self.create_subscriber_node(msg_type = Int8,
                                                             msg_name = self.parent.tbot.platform.arm.name + '/wireless_servo_peripheral/servo_closed')
        self._contact_switch_node = self.create_subscriber_node(msg_type = Bool,
                                                                msg_name = self.parent.tbot.platform.arm.name + '/wireless_servo_peripheral/limitswitch0')
        self._contact_confirmed_node = self.create_subscriber_node(msg_type = Bool,
                                                                   msg_name = self.parent.tbot.platform.arm.name + '/docking_controller/contact_confirmed')
        self._enable_control_node = self.create_client_node(srv_type = EmptyService, enable_response = False,
                                                            srv_name = self.parent.tbot.platform.arm.name + '/arm_controller/enable_control')
        self._disable_control_node = self.create_client_node(srv_type = EmptyService, enable_response = False,
                                                             srv_name = self.parent.tbot.platform.arm.name + '/arm_controller/disable_control')
        self._confirm_contact_node = self.create_client_node(srv_type = EmptyService, enable_response = False,
                                                             srv_name = self.parent.tbot.platform.arm.name + '/docking_controller/confirm_contact')
        self._open_action_node = self.create_action_client_node(action_type = EmptyAction, 
                                                                action_name = self.parent.tbot.platform.arm.name + '/docking_controller/open')
        self._close_action_node = self.create_action_client_node(action_type = EmptyAction,
                                                                 action_name = self.parent.tbot.platform.arm.name + '/docking_controller/close')
        
        state_frame = self.create_label_frame(master = self, text = 'State')
        state_frame.grid(row = 0, column = 0)

        self._actual_pose_label_frame = self.create_pose_label_frame(master = state_frame, text = 'Actual Pose:')
        self._actual_pose_label_frame.grid(row = 0, column = 0, columnspan = 2)
        self._target_pose_label_frame = self.create_pose_label_frame(master = state_frame, text = 'Target Pose:')
        self._target_pose_label_frame.grid(row = 1, column = 0, columnspan = 2)

        label = self.create_label(master = state_frame, text = 'Joint States:')
        label.grid(row = 2, column = 0)
        self._joint_states_label = self.create_vector_label(length = len(self.parent.tbot.platform.arm.links), digits = 3, master = state_frame)
        self._joint_states_label.grid(row = 2, column = 1)

        label = self.create_label(master = state_frame, text = 'Control Enabled:')
        label.grid(row = 3, column = 0)
        self._control_enabled_label = self.create_bool_label(master = state_frame)
        self._control_enabled_label.grid(row = 3, column = 1)

        label = self.create_label(master = state_frame, text = 'Closed: ')
        label.grid(row = 4, column = 0)
        self._closed_label = self.create_bool_label(master = state_frame)
        self._closed_label.grid(row = 4, column = 1)

        label = self.create_label(master = state_frame, text = 'Contact Switch:')
        label.grid(row = 5, column = 0)
        self._contact_switch_label = self.create_bool_label(master = state_frame)
        self._contact_switch_label.grid(row = 5, column = 1)

        label = self.create_label(master = state_frame, text = 'Contact Confirmed:')
        label.grid(row = 6, column = 0)
        self._contact_confirmed_label = self.create_bool_label(master = state_frame)
        self._contact_confirmed_label.grid(row = 6, column = 1)

        service_frame = self.create_label_frame(master = self, text = 'Services')
        service_frame.grid(row = 1, column = 0)

        self._enable_control_button = self.create_button(master = service_frame, text = 'Enable Control', command = self.enable_control_button_callback)
        self._enable_control_button.grid(row = 0, column = 0)
        self._disable_control_button = self.create_button(master = service_frame, text = 'Disable Control', command = self.disable_control_button_callback)
        self._disable_control_button.grid(row = 1, column = 0)
        self._confirm_contact_button = self.create_button(master = service_frame, text = 'Confirm Contact', command = self.confirm_contact_button_callback)
        self._confirm_contact_button.grid(row = 2, column = 0)
        
        action_frame = self.create_label_frame(master = self, text = 'Actions')
        action_frame.grid(row = 2, column = 0)

        self._open_button = self.create_button(master = action_frame, text = 'Open', command = self.open_button_callback)
        self._open_button.grid(row = 0, column = 0, columnspan = 2)
        self._close_button = self.create_button(master = action_frame, text = 'Close', command = self.close_button_callback)
        self._close_button.grid(row = 1, column = 0, columnspan = 2)
        self._cancel_button = self.create_cancel_button(master = action_frame, command = self.cancel_button_callback)
        self._cancel_button.grid(row = 2, column = 0, columnspan = 2)

        label = self.create_label(master = action_frame, text = 'Status:')
        label.grid(row = 3, column = 0)
        self._status_label = self.create_action_status_label(master = action_frame)
        self._status_label.grid(row = 3, column = 1)

        self.create_timer(callback = self.timer_callback, timeout_ms = 100)

    def timer_callback(self):

        if not self._actual_pose_node.msg_queue.empty():
            msg: PoseStamped = self._actual_pose_node.msg_queue.get()
            self._actual_pose_label_frame.update_data(msg.pose)
        if not self._target_pose_node.msg_queue.empty():
            msg: Pose = self._target_pose_node.msg_queue.get()
            self._target_pose_label_frame.update_data(msg)
        if not self._joint_states_node.msg_queue.empty():
            msg: Float64Array = self._joint_states_node.msg_queue.get()
            self._joint_states_label.update_data(msg.data)
        if not self._control_enabled_node.msg_queue.empty():
            msg: Bool = self._control_enabled_node.msg_queue.get()
            self._control_enabled_label.update_data(msg.data)
        if not self._servo_close_node.msg_queue.empty():
            msg: Int8 = self._servo_close_node.msg_queue.get()
            self._closed_label.update_data(msg.data)
        if not self._contact_switch_node.msg_queue.empty():
            msg: Bool = self._contact_switch_node.msg_queue.get()
            self._contact_switch_label.update_data(msg.data)
        if not self._contact_confirmed_node.msg_queue.empty():
            msg: Bool = self._contact_confirmed_node.msg_queue.get()
            self._contact_confirmed_label.update_data(msg.data)
        if not self._open_action_node.status_queue.empty():
            self._status_label.update_data(self._open_action_node.status_queue.get())
        if not self._close_action_node.status_queue.empty():
            self._status_label.update_data(self._close_action_node.status_queue.get())
        
    def enable_control_button_callback(self):

        self._enable_control_node.req_queue.put(EmptyService.Request())

    def disable_control_button_callback(self):

        self._disable_control_node.req_queue.put(EmptyService.Request())

    def confirm_contact_button_callback(self):

        self._confirm_contact_node.req_queue.put(EmptyService.Request())

    def open_button_callback(self):

        self._open_action_node.goal_queue.put(EmptyAction.Goal())

    def close_button_callback(self):

        self._close_action_node.goal_queue.put(EmptyAction.Goal())

    def cancel_button_callback(self):

        self._open_action_node.cancel_event.set()
        self._close_action_node.cancel_event.set()
    