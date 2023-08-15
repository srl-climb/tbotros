from __future__ import annotations
from std_msgs.msg import Bool, String, Int8
from std_srvs.srv import Trigger
from custom_actions.action import Empty as EmptyAction
from .interface_nodes import SubscriptionNode, ClientNode, ActionClientNode
from .tkinter_objects import TkBoolLabel, TkButton, TkLabel, TkCancelButton, \
    TkActionStatusLabel
from .window import Window
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App


class WirelessServoWindow(Window):

    def __init__(self, parent: App, title: str = 'Wireless Servo Window'):
        
        super().__init__(parent, title)

    def create_user_interface(self):

        self.arduino_local_name_labels: list[TkLabel] = []
        self.arduino_connected_labels: list[TkBoolLabel] = []
        self.arduino_error_code_labels: list[TkLabel] = []
        self.servo_running_labels: list[TkBoolLabel] = []
        self.servo_closed_labels: list[TkBoolLabel] = []
        self.limitswitch0_labels: list[TkBoolLabel] = []
        self.limitswitch1_labels: list[TkBoolLabel] = []
        self.success_labels: list[TkBoolLabel] = []
        self.status_labels: list[TkBoolLabel] = []

        self.clear_arduino_buttons: list[TkButton] = []
        self.reset_arduino_buttons: list[TkButton] = []
        self.open_servo_buttons: list[TkButton] = []
        self.close_servo_buttons: list[TkButton] = []
        self.cancel_servo_buttons: list[TkCancelButton] = []
 
        self.arduino_local_name_nodes: list[SubscriptionNode] = []
        self.arduino_connected_nodes: list[SubscriptionNode] = []
        self.arduino_error_code_nodes: list[SubscriptionNode] = []
        self.servo_running_nodes: list[SubscriptionNode] = []
        self.servo_closed_nodes: list[SubscriptionNode] = []
        self.limitswitch0_nodes: list[SubscriptionNode] = []
        self.limitswitch1_nodes: list[SubscriptionNode] = []
        self.reset_arduino_nodes: list[ClientNode] = []
        self.clear_arduino_nodes: list[ClientNode] = []
        self.open_servo_nodes: list[ActionClientNode] = []
        self.close_servo_nodes: list[ActionClientNode] = []

        # create master frame
        master_frame = self.create_label_frame(master = self, text = 'Master')
        master_frame.grid(row = 0, column = 0, sticky="N")
        master_service_frame = self.create_label_frame(master = master_frame, text = 'Services')
        master_service_frame.pack()
        self.clear_all_button = self.create_button(master = master_service_frame, width = 10, command = self.clear_all_button_callback, text = 'Clear All')
        self.clear_all_button.grid(row = 0, column = 0, columnspan = 2)
        self.reset_all_button = self.create_button(master = master_service_frame, width = 10, command = self.reset_all_button_callback, text = 'Reset All')
        self.reset_all_button.grid(row = 1, column = 0, columnspan = 2)

        self.create_button()
         
        # create arduino label frames
        self.create_arduino_label_frame(0, 1, 'Arduino Central 0', 'wireless_servo_central', 'central')

        for i in range(self.parent.tbot.k):
            self.create_arduino_label_frame(0, i+2, 'Arduino Peripheral ' + str(i), self.parent.tbot.grippers[i].name + '/wireless_servo_peripheral', 'peripheral')

        self.create_arduino_label_frame(0, self.parent.tbot.k+2, 'Arduino Peripheral ' + str(self.parent.tbot.k), self.parent.tbot.platform.arm.name + '/wireless_servo_peripheral', 'peripheral')

        self.create_timer(callback = self.timer_callback, timeout_ms = 100)
    
    def create_arduino_label_frame(self, row: int, column: int, text: str, node_namespace: str, arduino_type: str):

        # create nodes
        self.arduino_local_name_nodes.append(
            self.create_subscriber_node(msg_name = node_namespace + '/arduino_local_name', msg_type = String))
        self.arduino_connected_nodes.append(
            self.create_subscriber_node(msg_name = node_namespace + '/arduino_connected', msg_type = Bool))
        self.arduino_error_code_nodes.append(
            self.create_subscriber_node(msg_name = node_namespace + '/arduino_error_code', msg_type = Int8))
        self.reset_arduino_nodes.append(
            self.create_client_node(srv_name = node_namespace + '/reset_arduino', srv_type = Trigger))
        self.clear_arduino_nodes.append(
            self.create_client_node(srv_name = node_namespace + '/clear_arduino', srv_type = Trigger))
        
        if arduino_type == 'peripheral':
            self.servo_running_nodes.append(
                self.create_subscriber_node(msg_name = node_namespace + '/servo_running', msg_type = Bool))
            self.servo_closed_nodes.append(
                self.create_subscriber_node(msg_name = node_namespace + '/servo_closed', msg_type = Int8))
            self.limitswitch0_nodes.append(
                self.create_subscriber_node(msg_name = node_namespace + '/limitswitch0', msg_type = Bool))
            self.limitswitch1_nodes.append(
                self.create_subscriber_node(msg_name = node_namespace + '/limitswitch1', msg_type = Bool))
            self.open_servo_nodes.append(
                self.create_action_client_node(action_name = node_namespace + '/servo_open', action_type = EmptyAction, auto_cancel = False))
            self.close_servo_nodes.append(
                self.create_action_client_node(action_name = node_namespace + '/servo_close', action_type = EmptyAction, auto_cancel = False))

        # arduino state label frame
        labelframe = self.create_label_frame(master = self, text = text)
        labelframe.grid(row = row, column = column, sticky="N")

        # state label frame
        state_labelframe = self.create_label_frame(master = labelframe, text = 'State')
        state_labelframe.grid(row = 0, column = 0)

        label = self.create_label(master = state_labelframe, text = "Local name:")
        label.grid(row = 0, column = 0)
        self.arduino_local_name_labels.append(self.create_string_label(master = state_labelframe))
        self.arduino_local_name_labels[-1].grid(row = 0, column = 1)
        label = self.create_label(master = state_labelframe, text = "Connected:")
        label.grid(row = 1, column = 0)
        self.arduino_connected_labels.append(self.create_bool_label(master = state_labelframe))
        self.arduino_connected_labels[-1].grid(row = 1, column = 1)
        label = self.create_label(master = state_labelframe, text = "Error code:")
        label.grid(row = 2, column = 0)
        self.arduino_error_code_labels.append(self.create_error_code_label(master = state_labelframe))
        self.arduino_error_code_labels[-1].grid(row = 2, column = 1)

        if arduino_type == 'peripheral':
            label = self.create_label(master = state_labelframe, text = "Servo Running:")
            label.grid(row = 3, column = 0)
            self.servo_running_labels.append(self.create_bool_label(master = state_labelframe))
            self.servo_running_labels[-1].grid(row = 3, column = 1)
            label = self.create_label(master = state_labelframe, text = "Servo Closed:")
            label.grid(row = 4, column = 0)
            self.servo_closed_labels.append(self.create_bool_label(master = state_labelframe))
            self.servo_closed_labels[-1].grid(row = 4, column = 1)
            label = self.create_label(master = state_labelframe, text = "Limitswitch 0:")
            label.grid(row = 5, column = 0)
            self.limitswitch0_labels.append(self.create_bool_label(master = state_labelframe))
            self.limitswitch0_labels[-1].grid(row = 5, column = 1)
            label = self.create_label(master = state_labelframe, text = "Limitswitch 1:")
            label.grid(row = 6, column = 0)
            self.limitswitch1_labels.append(self.create_bool_label(master = state_labelframe))
            self.limitswitch1_labels[-1].grid(row = 6, column = 1)

        # service label frame
        service_labelframe = self.create_label_frame(master = labelframe, text = 'Services')
        service_labelframe.grid(row = 1, column = 0)

        label = self.create_label(master = service_labelframe, text = "Success:")
        label.grid(row = 2, column = 0)
        self.success_labels.append(self.create_bool_label(master = service_labelframe))
        self.success_labels[-1].grid(row = 2, column = 1)

        command = lambda node = self.clear_arduino_nodes[-1], success_label = self.success_labels[-1]: self.clear_button_callback(node, success_label)
        self.clear_arduino_buttons.append(self.create_button(
            master = service_labelframe, width = 10, text='Clear', command = command))
        self.clear_arduino_buttons[-1].grid(row = 0, column = 0, columnspan = 2)

        command = lambda node = self.reset_arduino_nodes[-1], success_label = self.success_labels[-1]: self.reset_button_callback(node, success_label)
        self.clear_arduino_buttons.append(self.create_button(
            master = service_labelframe, width = 10, text='Reset', command = command))
        self.clear_arduino_buttons[-1].grid(row = 1, column = 0, columnspan = 2)

        # action label frame
        if arduino_type == 'peripheral':
            action_labelframe = self.create_label_frame(master = labelframe, text = 'Actions')
            action_labelframe.grid(row = 2, column = 0)

            label = self.create_label(master = action_labelframe, text = "Status:")
            label.grid(row = 3, column = 0)
            self.status_labels.append(self.create_action_status_label(master = action_labelframe))
            self.status_labels[-1].grid(row = 3, column = 1)
            
            command = lambda node = self.open_servo_nodes[-1], status_label = self.status_labels[-1]: self.open_servo_button_callback(node, status_label)
            self.open_servo_buttons.append(self.create_button(
                master = action_labelframe, width = 10, text='Open Servo', command = command))
            self.open_servo_buttons[-1].grid(row = 0, column = 0, columnspan = 2)

            command = lambda node = self.close_servo_nodes[-1], status_label = self.status_labels[-1]: self.close_servo_button_callback(node, status_label)
            self.close_servo_buttons.append(self.create_button(
                master = action_labelframe, width = 10, text='Close Servo', command = command))
            self.close_servo_buttons[-1].grid(row = 1, column = 0, columnspan = 2)

            command = lambda open_servo_node = self.open_servo_nodes[-1], close_servo_node = self.close_servo_nodes[-1]: self.cancel_servo_button_callback(open_servo_node, close_servo_node)
            self.cancel_servo_buttons.append(self.create_cancel_button(
                master = action_labelframe, width = 10, command = command))
            self.cancel_servo_buttons[-1].grid(row = 2, column = 0, columnspan = 2)

    def clear_all_button_callback(self):
        for node, label in zip(self.clear_arduino_nodes, self.success_labels):
            self.clear_button_callback(node, label)

    def reset_all_button_callback(self):
        for node, label in zip(self.reset_arduino_nodes, self.success_labels):
            self.reset_button_callback(node, label)

    def open_servo_button_callback(self, node: ActionClientNode, status_label: TkActionStatusLabel):
        status_label.update_data(0)
        node.goal_queue.put(EmptyAction.Goal())

    def close_servo_button_callback(self, node: ActionClientNode, status_label: TkActionStatusLabel):
        status_label.update_data(0)
        node.goal_queue.put(EmptyAction.Goal())

    def cancel_servo_button_callback(self, open_servo_node: ActionClientNode, close_servo_node: ActionClientNode):
        close_servo_node.cancel_event.set()
        open_servo_node.cancel_event.set()

    def clear_button_callback(self, node: ClientNode, success_label: TkBoolLabel):
        node.req_queue.put(Trigger.Request())
        success_label.update_data(None)

    def reset_button_callback(self, node: ClientNode, success_label: TkBoolLabel):
        node.req_queue.put(Trigger.Request())
        success_label.update_data(None)

    def timer_callback(self):
        
        for node, label in zip(self.arduino_connected_nodes, self.arduino_connected_labels):
            if not node.msg_queue.empty():
                label.update_data(node.msg_queue.get().data)

        for node, label in zip(self.arduino_local_name_nodes, self.arduino_local_name_labels):
            if not node.msg_queue.empty():
                label.update_data(node.msg_queue.get().data)

        for node, label in zip(self.arduino_error_code_nodes, self.arduino_error_code_labels):
            if not node.msg_queue.empty():
                label.update_data(node.msg_queue.get().data)

        for node, label in zip(self.servo_running_nodes, self.servo_running_labels):
            if not node.msg_queue.empty():
                label.update_data(node.msg_queue.get().data)

        for node, label in zip(self.servo_closed_nodes, self.servo_closed_labels):
            if not node.msg_queue.empty():
                label.update_data(node.msg_queue.get().data)

        for node, label in zip(self.limitswitch0_nodes, self.limitswitch0_labels):
            if not node.msg_queue.empty():
                label.update_data(node.msg_queue.get().data)

        for node, label in zip(self.limitswitch1_nodes, self.limitswitch1_labels):
            if not node.msg_queue.empty():
                label.update_data(node.msg_queue.get().data)

        for node, label in zip(self.clear_arduino_nodes, self.success_labels):
            if not node.res_queue.empty():
                label.update_data(node.res_queue.get().success)
        
        for node, label in zip(self.reset_arduino_nodes, self.success_labels):
            if not node.res_queue.empty():
                label.update_data(node.res_queue.get().success)

        for node, label in zip(self.open_servo_nodes, self.status_labels):
            if not node.status_queue.empty():
                label.update_data(node.status_queue.get())

        for node, label in zip(self.close_servo_nodes, self.status_labels):
            if not node.status_queue.empty():
                label.update_data(node.status_queue.get())

