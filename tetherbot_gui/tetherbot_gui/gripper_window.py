from __future__ import annotations
import tkinter as tk
from std_msgs.msg import Bool, String, Int8
from std_srvs.srv import Empty as EmptyService
from custom_srvs.srv import SetString
from custom_actions.action import Empty as EmptyAction
from geometry_msgs.msg import PoseStamped
from .interface_nodes import SubscriptionNode, ClientNode, ActionClientNode
from .tkinter_objects import TkBoolLabel, TkButton, TkStringLabel, TkCancelButton, \
    TkActionStatusLabel, TkPoseLabelFrame, TkOptionMenu
from .window import Window
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App

    
class GripperWindow(Window):

    def __init__(self, parent: App, title: str = 'Gripper Window'):
        
        super().__init__(parent, title)

    def create_user_interface(self):

        self._pose_frames: list[TkPoseLabelFrame] = []
        self._transform_source_labels: list[TkStringLabel] = []
        self._hold_name_labels: list[TkStringLabel] = []
        self._closed_labels: list[TkBoolLabel] = []
        self._contact_switch_labels: list[TkBoolLabel] = []
        self._contact_confirmed_labels: list[TkBoolLabel] = []
        self._hold_name_menues: list[TkOptionMenu] = []
        self._set_hold_buttons: list[TkButton] = []
        self._transform_source_menues: list[TkOptionMenu] = []
        self._set_transform_buttons: list[TkButton] = []
        self._open_buttons: list[TkButton] = []
        self._close_buttons: list[TkButton] = []
        self._cancel_buttons: list[TkCancelButton] = []
        self._confirm_contact_buttons: list[TkButton] = []
        self._action_status_labels: list[TkActionStatusLabel] = []

        self._pose_nodes: list[SubscriptionNode] = []
        self._transform_source_nodes: list[SubscriptionNode] = []
        self._contact_confirmed_nodes: list[SubscriptionNode] = []
        self._hold_name_nodes: list[SubscriptionNode] = []
        self._closed_nodes: list[SubscriptionNode] = []
        self._contact_switch_nodes: list[SubscriptionNode] = []
        self._set_hold_clients: list[ClientNode] = []
        self._set_transform_clients: list[ClientNode] = []
        self._confirm_contact_clients: list[ClientNode] = []
        self._open_action_clients: list[ActionClientNode] = []
        self._close_action_clients: list[ActionClientNode] = []

        hold_names = []
        for hold in self.parent.tbot.wall.holds:
            hold_names.append(hold.name)

        for gripper, i in zip(self.parent.tbot.grippers, range(self.parent.tbot.k)):

            self._pose_nodes.append(
                self.create_subscriber_node(msg_name = gripper.name + '/gripper_state_publisher/pose', msg_type = PoseStamped))
            self._hold_name_nodes.append(
                self.create_subscriber_node(msg_name = gripper.name + '/gripper_state_publisher/hold_name', msg_type = String))
            self._closed_nodes.append(
                self.create_subscriber_node(msg_name = gripper.name + '/wireless_servo_peripheral/servo_closed', msg_type = Int8))
            self._contact_switch_nodes.append(
                self.create_subscriber_node(msg_name = gripper.name + '/wireless_servo_peripheral/limitswitch0', msg_type = Bool))
            self._transform_source_nodes.append(
                self.create_subscriber_node(msg_name = gripper.name + '/gripper_state_publisher/transform_source', msg_type = String))
            self._contact_confirmed_nodes.append(
                self.create_subscriber_node(msg_name = gripper.name + '/gripper_controller/contact_confirmed', msg_type = Bool))
            self._set_transform_clients.append(
                self.create_client_node(srv_name = gripper.name + '/gripper_state_publisher/set_transform_source', srv_type = SetString, enable_response = False))
            self._set_hold_clients.append(
                self.create_client_node(srv_name = gripper.name + '/gripper_state_publisher/set_hold', srv_type = SetString, enable_response = False))
            self._confirm_contact_clients.append(
                self.create_client_node(srv_name = gripper.name + '/gripper_controller/confirm_contact', srv_type = EmptyService, enable_response = False))
            self._open_action_clients.append(
                self.create_action_client_node(action_name = gripper.name + '/gripper_controller/open', action_type = EmptyAction))
            self._close_action_clients.append(
                self.create_action_client_node(action_name = gripper.name + '/gripper_controller/close', action_type = EmptyAction))

            main_frame = self.create_label_frame(master = self, text = 'Gripper ' + str(i) + ' (' + gripper.name + ')')
            main_frame.grid(row = 0, column = i)
            state_frame = self.create_label_frame(master = main_frame, text = 'State')
            state_frame.grid(row = 0, column = 0)
            pose_frame = self.create_pose_label_frame(master = state_frame)
            pose_frame.grid(row = 0, column = 0, columnspan = 2)
            self._pose_frames.append(pose_frame)

            label = self.create_label(master = state_frame, text = 'Closed:')
            label.grid(row = 2, column = 0)
            closed_label = self.create_bool_label(master = state_frame)
            closed_label.grid(row = 2, column = 1)
            self._closed_labels.append(closed_label)

            label = self.create_label(master = state_frame, text = 'Contactswitch:')
            label.grid(row = 3, column = 0)
            contact_switch_label = self.create_bool_label(master = state_frame)
            contact_switch_label.grid(row = 3, column = 1)
            self._contact_switch_labels.append(contact_switch_label)

            label = self.create_label(master = state_frame, text = 'Contact Confirmed:')
            label.grid(row = 4, column = 0)
            contact_confirmed_label = self.create_bool_label(master = state_frame)
            contact_confirmed_label.grid(row = 4, column = 1)
            self._contact_confirmed_labels.append(contact_confirmed_label)

            label = self.create_label(master = state_frame, text = 'Transform Source:')
            label.grid(row = 5, column = 0)
            transform_source_label = self.create_string_label(master = state_frame)
            transform_source_label.grid(row = 5, column = 1)
            self._transform_source_labels.append(transform_source_label)

            label = self.create_label(master = state_frame, text = 'Hold Name:')
            label.grid(row = 6, column = 0)
            hold_name_label = self.create_string_label(master = state_frame)
            hold_name_label.grid(row = 6, column = 1)
            self._hold_name_labels.append(hold_name_label)

            service_frame = self.create_label_frame(master = main_frame, text = 'Services')
            service_frame.grid(row = 1, column = 0)

            label = self.create_label(master = service_frame, text = 'Hold Name:')
            label.grid(row = 0, column = 0)
            hold_name_menu = self.create_option_menu(master = service_frame, values = hold_names)
            hold_name_menu.grid(row = 0, column = 1)
            self._hold_name_menues.append(hold_name_menu)
            
            command = lambda node = self._set_hold_clients[-1], menu = hold_name_menu: self.set_hold_button_callback(node, menu)
            set_hold_button = self.create_button(master = service_frame, text = 'Set Hold', command = command)
            set_hold_button.grid(row = 1, column = 0, columnspan = 2)

            label = self.create_label(master = service_frame, text = 'Transform Source:')
            label.grid(row = 2, column = 0)

            transform_source_menu = self.create_option_menu(master = service_frame, values = ['arm', 'hold', 'marker'])
            transform_source_menu.grid(row = 2, column =  1)
            self._transform_source_menues.append(transform_source_menu)
            
            command = lambda node = self._set_transform_clients[-1], menu = transform_source_menu: self.set_transform_button_callback(node, menu)
            set_transform_button = self.create_button(master = service_frame, text = 'Set Transform Source', command = command)
            set_transform_button.grid(row = 3, column = 0, columnspan = 2)
            self._set_transform_buttons.append(set_hold_button)

            command = lambda node = self._confirm_contact_clients[-1]: self.confirm_contact_button_callback(node)
            confirm_contact_button = self.create_button(master = service_frame, text = 'Confirm Contact', command = command)
            confirm_contact_button.grid(row = 4, column = 0, columnspan = 2)
            self._set_transform_buttons.append(confirm_contact_button)

            action_frame = self.create_label_frame(master = main_frame, text = 'Actions')
            action_frame.grid(row = 2, column = 0)

            command = lambda node = self._open_action_clients[-1]: self.open_button_callback(node)
            open_button = self.create_button(master = action_frame, text = 'Open', command = command)
            open_button.grid(row = 0 , column = 0, columnspan = 2)
            self._open_buttons.append(open_button)

            command = lambda node = self._close_action_clients[-1]: self.close_button_callback(node)
            close_button = self.create_button(master = action_frame, text = 'Close', command = command)
            close_button.grid(row = 1, column = 0, columnspan = 2)
            self._close_buttons.append(close_button)

            command = lambda node1 = self._close_action_clients[-1], node2 = self._open_action_clients[-1]: self.cancel_button_callback(node1, node2)
            cancel_button = self.create_cancel_button(master = action_frame, command = command)
            cancel_button.grid(row = 2, column = 0, columnspan = 2)
            self._cancel_buttons.append(cancel_button)

            label = self.create_label(master = action_frame, text = 'Status:')
            label.grid(row = 3, column = 0)
            status_label = self.create_action_status_label(master = action_frame)
            status_label.grid(row = 3, column = 1)
            self._action_status_labels.append(status_label)

        self.create_timer(callback=self.timer_callback, timeout_ms=100)

    def set_transform_button_callback(self, node: ClientNode, menu = TkOptionMenu):
        
        request = SetString.Request()
        request.data = menu.variable.get()

        node.req_queue.put(request)

    def set_hold_button_callback(self, node: ClientNode, menu = TkOptionMenu):

        request = SetString.Request()
        request.data = menu.variable.get()
        node.req_queue.put(request)

    def confirm_contact_button_callback(self, node: ClientNode):

        node.req_queue.put(EmptyService.Request())

    def open_button_callback(self, node: ActionClientNode):

        node.goal_queue.put(EmptyAction.Goal())

    def close_button_callback(self, node: ActionClientNode):

        node.goal_queue.put(EmptyAction.Goal())

    def cancel_button_callback(self, node1: ActionClientNode, node2: ActionClientNode):
        
        node1.cancel_event.set()
        node2.cancel_event.set()

    def timer_callback(self):

        for i in range(self.parent.tbot.k):
            
            if not self._pose_nodes[i].msg_queue.empty():
                msg: PoseStamped = self._pose_nodes[i].msg_queue.get()
                self._pose_frames[i].update_data(msg.pose)
            if not self._transform_source_nodes[i].msg_queue.empty():
                msg: String = self._transform_source_nodes[i].msg_queue.get()
                self._transform_source_labels[i].update_data(msg.data)
            if not self._contact_switch_nodes[i].msg_queue.empty():
                msg: Bool = self._contact_switch_nodes[i].msg_queue.get()
                self._contact_switch_labels[i].update_data(msg.data)
            if not self._contact_confirmed_nodes[i].msg_queue.empty():
                msg: Bool = self._contact_confirmed_nodes[i].msg_queue.get()
                self._contact_confirmed_labels[i].update_data(msg.data)
            if not self._closed_nodes[i].msg_queue.empty():
                msg: Bool = self._closed_nodes[i].msg_queue.get()
                self._closed_labels[i].update_data(msg.data)
            if not self._hold_name_nodes[i].msg_queue.empty():
                msg: String = self._hold_name_nodes[i].msg_queue.get()
                self._hold_name_labels[i].update_data(msg.data)
            if not self._open_action_clients[i].status_queue.empty():
                status = self._open_action_clients[i].status_queue.get()
                self._action_status_labels[i].update_data(status)
            if not self._close_action_clients[i].status_queue.empty():
                status = self._close_action_clients[i].status_queue.get()
                self._action_status_labels[i].update_data(status)

