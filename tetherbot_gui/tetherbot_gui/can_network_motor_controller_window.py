from __future__ import annotations
import tkinter as tk
from std_msgs.msg import Float64, String, Int16
from std_srvs.srv import Trigger
from custom_msgs.msg import Statusword, MotorPosition, CanError, DigitalInputs
from custom_actions.action import MoveMotor
from .interface_nodes import SubscriptionNode, ClientNode, ActionClientNode
from .tkinter_objects import TkBoolLabel, TkStringLabel, TkFloatLabel, TkErrorCodeLabel, \
    TkActionStatusLabel, TkFloatEntry, TkOptionMenu
from .window import Window
from typing import TYPE_CHECKING, Dict, List

if TYPE_CHECKING:
    from .tetherbot_gui import App

    
class CanNetworkMotorControllerWindow(Window):

    def __init__(self, parent: App, title: str = 'CanNetwork Motor Controller'):
        # Initialize the CanNetworkMotorController class, which is a subclass of Window
        # The parent parameter represents the parent window
        # The title parameter is the title of the window

        self._motor_namespaces = ['motor' + str(i) for i in range(13)]

        super().__init__(parent, title)

    def create_user_interface(self):
        # Create the user interface 

        frame = self.create_scroll_frame(master = self)

        # nodes and labels for topics
        self.statusword_nodes: list[SubscriptionNode] = []
        self.statusword_labels: List[Dict[str, TkBoolLabel]] = []
        self.state_nodes: list[SubscriptionNode] = []
        self.state_labels: List[TkStringLabel] = []
        self.position_nodes: list[SubscriptionNode] = []
        self.position_labels: List[Dict[str, TkFloatLabel]] = []
        self.velocity_nodes: list[SubscriptionNode] = []
        self.velocity_labels: List[TkFloatLabel] = []
        self.current_nodes: list[SubscriptionNode] = []
        self.current_labels: List[TkFloatLabel] = []
        self.inputs_nodes: list[SubscriptionNode] = []
        self.inputs_labels: List[Dict[str, TkBoolLabel]] = []
        self.can_error_nodes: list[SubscriptionNode] = []
        self.can_error_labels: List[Dict[str, TkStringLabel, TkErrorCodeLabel, TkFloatLabel]] =[]

        self.mode_menues: List[TkOptionMenu] = []
        self.target_position_entries: List[TkFloatEntry] = []
        self.profile_velocity_entries: List[TkFloatEntry] = []
        self.profile_acceleration_entries: List[TkFloatEntry] = []
        self.profile_deceleration_entries: List[TkFloatEntry] = []
        self.status_labels: List[TkActionStatusLabel] = []
        self.success_labels: List[TkBoolLabel] = []
        self.message_labels: List[TkStringLabel] = []
        
        self.shut_down_nodes: list[ClientNode] = []
        self.switch_on_nodes: list[ClientNode] = []
        self.disable_voltage_nodes: list[ClientNode] = []
        self.disable_operation_nodes: list[ClientNode] = []
        self.enable_operation_nodes: list[ClientNode] = []
        self.quick_stop_nodes: list[ClientNode] = []
        self.fault_reset_nodes: list[ClientNode] = []
        self.save_home_nodes: list[ClientNode] = []
        self.move_action_nodes: list[ActionClientNode] = []

        # Iterate through each motor namespace
        for i in range(len(self._motor_namespaces)):
            self.statusword_labels.append({})
            self.position_labels.append({})
            self.inputs_labels.append({})
            self.can_error_labels.append({})

            # Create a subscriber node for receiving status word messages
            self.statusword_nodes.append(self.create_subscriber_node(msg_name=self._motor_namespaces[i] + '/faulhaber_motor/statusword',
                                                                     msg_type=Statusword))  
            self.state_nodes.append(self.create_subscriber_node(msg_name=self._motor_namespaces[i] + '/faulhaber_motor/state',
                                                         msg_type=String))  
            self.position_nodes.append(self.create_subscriber_node(msg_name=self._motor_namespaces[i] + '/faulhaber_motor/position',
                                                                   msg_type=MotorPosition))
            self.velocity_nodes.append(self.create_subscriber_node(msg_name=self._motor_namespaces[i] + '/faulhaber_motor/velocity',
                                                                   msg_type=Float64))
            self.current_nodes.append(self.create_subscriber_node(msg_name=self._motor_namespaces[i] + '/faulhaber_motor/current',
                                                                  msg_type=Int16))
            self.inputs_nodes.append(self.create_subscriber_node(msg_name=self._motor_namespaces[i] + '/faulhaber_motor/inputs',
                                                                  msg_type=DigitalInputs))
            self.can_error_nodes.append(self.create_subscriber_node(msg_name=self._motor_namespaces[i] + '/faulhaber_motor/can_error',
                                                                    msg_type=CanError, timeout_sec = -1))
            self.shut_down_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/shut_down',
                                                                srv_type=Trigger))
            self.switch_on_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/switch_on',
                                                                srv_type=Trigger))
            self.disable_voltage_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/disable_voltage',
                                                                      srv_type=Trigger))
            self.disable_operation_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/disable_operation',
                                                                        srv_type=Trigger))
            self.enable_operation_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/enable_operation',
                                                                       srv_type=Trigger))
            self.quick_stop_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/quick_stop',
                                                                 srv_type=Trigger))
            self.fault_reset_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/fault_reset',
                                                                  srv_type=Trigger))
            self.save_home_nodes.append(self.create_client_node(srv_name=self._motor_namespaces[i] + '/faulhaber_motor/save_home',
                                                                srv_type=Trigger))
            self.move_action_nodes.append(self.create_action_client_node(action_name=self._motor_namespaces[i] + '/faulhaber_motor/move',
                                                                         action_type=MoveMotor))
            
            motor_labelframe = self.create_label_frame(master=frame, text='Motor ' + str(i))
            motor_labelframe.grid(row=0, column= 1 + i)

            # Create labels and bool labels for each status word/state

            state_frame = self.create_label_frame(master = motor_labelframe, text = 'State')
            state_frame.grid(row = 0 , column = 0, columnspan=2)

            labelframe = self.create_label_frame(master=state_frame, text='Statusword')
            labelframe.grid(row = 0 , column = 0, columnspan=2)

            label = self.create_label(master=labelframe, text="Ready to switch on:")
            label.grid(row=0, column=0)
            self.statusword_labels[i]['ready_to_switch_on'] = self.create_bool_label(master=labelframe)
            self.statusword_labels[i]['ready_to_switch_on'].grid(row=0, column=1)

            label = self.create_label(master=labelframe, text="Switched on:")
            label.grid(row=1, column=0)
            self.statusword_labels[i]['switched_on'] = self.create_bool_label(master=labelframe)
            self.statusword_labels[i]['switched_on'].grid(row=1, column=1)

            label = self.create_label(master = labelframe, text = "Voltage enabled:")
            label.grid(row=2,column=0)
            self.statusword_labels[i]['voltage_enabled'] = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['voltage_enabled'].grid(row=2,column=1)

            label = self.create_label(master = labelframe, text = "Operation enabled:")
            label.grid(row=3,column=0)
            self.statusword_labels[i]['operation_enabled'] = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['operation_enabled'].grid(row=3,column=1)

            label = self.create_label(master = labelframe, text = "Target reached:")
            label.grid(row=4,column=0)
            self.statusword_labels[i]['target_reached'] = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['target_reached'].grid(row=4,column=1)

            label = self.create_label(master = labelframe, text = "Quick stop:")
            label.grid(row=5,column=0)
            self.statusword_labels[i]['quick_stop'] = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['quick_stop'].grid(row=5,column=1)

            label = self.create_label(master = labelframe, text = "Switch on disabled:")
            label.grid(row=6,column=0)
            self.statusword_labels[i]['switch_on_disabled'] = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['switch_on_disabled'].grid(row=6,column=1)

            label = self.create_label(master = labelframe, text = "Set point acknowledge:")
            label.grid(row=7,column=0)
            self.statusword_labels[i]['setpoint_acknowledge'] = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['setpoint_acknowledge'].grid(row=7,column=1)

            label = self.create_label(master = labelframe, text = "Internal limit active:")
            label.grid(row=8,column=0)
            self.statusword_labels[i]['internal_limit_active'] = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['internal_limit_active'].grid(row=8,column=1)

            label = self.create_label(master = labelframe, text = "Deviation error:")
            label.grid(row=9,column=0)
            self.statusword_labels[i]['deviation_error']  = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['deviation_error'].grid(row=9,column=1)

            label = self.create_label(master = labelframe, text = "Fault:")
            label.grid(row=10,column=0)
            self.statusword_labels[i]['fault']  = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['fault'].grid(row=10,column=1)

            label = self.create_label(master = labelframe, text = "Warning:")
            label.grid(row=11,column=0)
            self.statusword_labels[i]['warning']  = self.create_bool_label(master = labelframe)
            self.statusword_labels[i]['warning'].grid(row=11,column=1)

            label = self.create_label(master=state_frame, text="State:")
            label.grid(row = 1 , column = 0)
            label = self.create_string_label(master=state_frame)
            label.grid(row=1, column=1)
            self.state_labels.append(label)

            label = self.create_label(master=state_frame, text="Actual Position [m; deg]:")
            label.grid(row=2, column=0)
            self.position_labels[i]['actual_pos'] = self.create_float_label(master=state_frame)
            self.position_labels[i]['actual_pos'].grid(row=2, column=1)

            label = self.create_label(master=state_frame, text="Target Position [m; deg]:")
            label.grid(row=3, column=0)
            self.position_labels[i]['target_pos'] = self.create_float_label(master=state_frame)
            self.position_labels[i]['target_pos'].grid(row=3, column=1)

            label = self.create_label(master=state_frame, text="Velocity [m/s; deg/s]:")
            label.grid(row=4, column=0)
            label = self.create_float_label(master=state_frame)
            label.grid(row=4, column=1)
            self.velocity_labels.append(label)

            label = self.create_label(master=state_frame, text="Current [mA]:")
            label.grid(row=5, column=0)
            label = self.create_float_label(master=state_frame)
            label.grid(row=5, column=1)
            self.current_labels.append(label)

            labelframe = self.create_label_frame(master=state_frame, text='Inputs')
            labelframe.grid(row = 6 , column = 0, columnspan=2)

            label = self.create_label(master=labelframe, text="Negative limit switch:")
            label.grid(row=0, column=0)
            self.inputs_labels[i]['negative_limit_switch'] = self.create_bool_label(master=labelframe)
            self.inputs_labels[i]['negative_limit_switch'].grid(row=0, column=1)

            label = self.create_label(master=labelframe, text="Positive limit switch:")
            label.grid(row=1, column=0)
            self.inputs_labels[i]['positive_limit_switch'] = self.create_bool_label(master=labelframe)
            self.inputs_labels[i]['positive_limit_switch'].grid(row=1, column=1)

            label = self.create_label(master=labelframe, text="Homing switch:")
            label.grid(row=2, column=0)
            self.inputs_labels[i]['homing_switch'] = self.create_bool_label(master=labelframe)
            self.inputs_labels[i]['homing_switch'].grid(row=2, column=1)

            labelframe = self.create_label_frame(master=state_frame, text='CAN Error')
            labelframe.grid(row = 7, column = 0, columnspan=2)

            label = self.create_label(master=labelframe, text="Description:")
            label.grid(row=0, column=0)
            self.can_error_labels[i]['description'] = self.create_string_label(master=labelframe)
            self.can_error_labels[i]['description'].grid(row=0, column=1)

            label = self.create_label(master=labelframe, text="Code:")
            label.grid(row=1, column=0)
            self.can_error_labels[i]['code'] = self.create_error_code_label(master=labelframe)
            self.can_error_labels[i]['code'].grid(row=1, column=1)

            label = self.create_label(master=labelframe, text="Timestamp:")
            label.grid(row=2, column=0)
            self.can_error_labels[i]['timestamp'] = self.create_float_label(master=labelframe)
            self.can_error_labels[i]['timestamp'].grid(row=2, column=1)

            service_frame = self.create_label_frame(master = motor_labelframe, text = 'Services')
            service_frame.grid(row = 1 , column = 0, columnspan=2)

            label = self.create_label(master=service_frame, text="Success:")
            label.grid(row=4, column=0)
            label = self.create_bool_label(master=service_frame)
            label.grid(row=4, column=1)
            self.success_labels.append(label)

            label = self.create_label(master=service_frame, text="Message:")
            label.grid(row=5, column=0)
            label = self.create_string_label(master=service_frame)
            label.grid(row=5, column=1)
            self.message_labels.append(label)

            command = lambda node = self.shut_down_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Shut Down')
            button.grid(row = 0, column = 0)

            command = lambda node = self.switch_on_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Switch On')
            button.grid(row = 0, column = 1)

            command = lambda node = self.disable_voltage_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Disable Voltage')
            button.grid(row = 1, column = 0)

            command = lambda node = self.quick_stop_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Quick Stop')
            button.config(bg="#FFC0CB")
            button.grid(row = 1, column = 1)

            command = lambda node = self.disable_operation_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Disable Operation')
            button.grid(row = 2, column = 0)

            command = lambda node = self.enable_operation_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Enable Operation')
            button.grid(row = 2, column = 1)

            command = lambda node = self.save_home_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Save Home*')
            button.grid(row = 3, column = 1)

            command = lambda node = self.fault_reset_nodes[-1], label = self.success_labels[-1]: self.trigger_service_button_callback(node, label)
            button = self.create_button(master = service_frame, width = 13, command = command, text = 'Fault Reset')
            button.grid(row = 3, column = 0)

            action_frame = self.create_label_frame(master = motor_labelframe, text = 'Actions')
            action_frame.grid(row = 2, column = 0, columnspan=2)

            label = self.create_label(master=action_frame, text="Mode:")
            label.grid(row=0, column=0)
            menu = self.create_option_menu(master = action_frame, values= ['Home', 'Position (abs.)', 'Position (rel.)', 'Home*'])
            menu.grid(row = 0, column = 1)
            self.mode_menues.append(menu)

            label = self.create_label(master=action_frame, text="Target Position:")
            label.grid(row=1, column=0)
            entry = self.create_float_entry(master=action_frame)
            entry.grid(row=1, column=1)
            self.target_position_entries.append(entry)

            label = self.create_label(master=action_frame, text="Profile Velocity:")
            label.grid(row=2, column=0)
            entry = self.create_float_entry(master=action_frame)
            entry.grid(row=2, column=1)
            self.profile_velocity_entries.append(entry)

            label = self.create_label(master=action_frame, text="Profile Acceleration:")
            label.grid(row=3, column=0)
            entry = self.create_float_entry(master=action_frame, min_val = 0)
            entry.grid(row=3, column=1)
            self.profile_acceleration_entries.append(entry)

            label = self.create_label(master=action_frame, text="Profile Deceleration:")
            label.grid(row=4, column=0)
            entry = self.create_float_entry(master=action_frame, min_val = 0)
            entry.grid(row=4, column=1)
            self.profile_deceleration_entries.append(entry)

            command = lambda node = self.move_action_nodes[-1], \
                mode_menu = self.mode_menues[-1], \
                target_position_entry = self.target_position_entries[-1], \
                profile_velocity_entry = self.profile_velocity_entries[-1], \
                profile_acceleration_entry = self.profile_acceleration_entries[-1], \
                profile_deceleration_entry = self.profile_deceleration_entries[-1]: \
                self.start_move_button_callback(node, 
                                                mode_menu, 
                                                target_position_entry, 
                                                profile_velocity_entry, 
                                                profile_acceleration_entry, 
                                                profile_deceleration_entry)
            button = self.create_button(master = action_frame, text='Start', width = 13, command = command)
            button.grid(row=5, column=0)
            command = lambda node = self.move_action_nodes[-1]: self.stop_move_button_callback(node)
            button = self.create_cancel_button(master = action_frame, text='Stop', width = 13, command = command)
            button.grid(row=5, column=1)

            label = self.create_label(master = action_frame, text = 'Status: ')
            label.grid(row=6, column=0)
            label = self.create_action_status_label(master = action_frame)
            label.grid(row=6, column=1)
            self.status_labels.append(label)
            
        master_frame = self.create_label_frame(master=frame, text='Master')
        master_frame.grid(row=0, column=0, sticky='N')

        service_frame = self.create_label_frame(master=master_frame, text='Services')
        service_frame.grid(row=0,column=0)

        command = lambda nodes = self.shut_down_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Shut Down')
        button.grid(row = 0, column = 0)

        command = lambda nodes = self.switch_on_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Switch On')
        button.grid(row = 0, column = 1)

        command = lambda nodes = self.disable_voltage_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Disable Voltage')
        button.grid(row = 1, column = 0)

        command = lambda nodes = self.fault_reset_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Quick Stop')
        button.config(bg="#FFC0CB")
        button.grid(row = 1, column = 1)

        command = lambda nodes = self.disable_operation_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Disable Operation')
        button.grid(row = 2, column = 0)

        command = lambda nodes = self.enable_operation_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Enable Operation')
        button.grid(row = 2, column = 1)

        command = lambda nodes = self.fault_reset_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Fault Reset')
        button.grid(row = 3, column = 0)

        command = lambda nodes = self.save_home_nodes, labels = self.success_labels: self.trigger_service_button_callbacks(nodes, labels)
        button = self.create_button(master = service_frame, width = 13, command = command, text = 'Save Home*')
        button.grid(row = 3, column = 1)

        action_frame = self.create_label_frame(master = master_frame, text = 'Actions')
        action_frame.grid(row=1,column=0)

        button = self.create_button(master = action_frame, text = 'Home*', command = self.home_star_button_callbacks, width = 13)
        button.grid(row=0,column=0)
        button = self.create_cancel_button(master=action_frame, text='STOP', command = self.stop_move_button_callbacks, width = 13)
        button.grid(row=0, column = 1)

        self.create_timer(callback=self.timer_callback, timeout_ms=100)

    def timer_callback(self):
        # Timer callback function that updates the status word labels

        for i in range(len(self._motor_namespaces)):
            if not self.statusword_nodes[i].msg_queue.empty():
                msg: Statusword = self.statusword_nodes[i].msg_queue.get()
                self.statusword_labels[i]['ready_to_switch_on'].update_data(msg.ready_to_switch_on)
                self.statusword_labels[i]['switched_on'].update_data(msg.switched_on)
                self.statusword_labels[i]['voltage_enabled'].update_data(msg.voltage_enabled)
                self.statusword_labels[i]['operation_enabled'].update_data(msg.operation_enabled)
                self.statusword_labels[i]['target_reached'].update_data(msg.target_reached)
                self.statusword_labels[i]['quick_stop'].update_data(msg.quick_stop)
                self.statusword_labels[i]['switch_on_disabled'].update_data(msg.switch_on_disabled)
                self.statusword_labels[i]['setpoint_acknowledge'].update_data(msg.setpoint_acknowledge_or_speed_or_homing_attained)
                self.statusword_labels[i]['internal_limit_active'].update_data(msg.internal_limit_active)
                self.statusword_labels[i]['deviation_error'].update_data(msg.deviation_error)
                self.statusword_labels[i]['fault'].update_data(msg.fault)
                self.statusword_labels[i]['warning'].update_data(msg.warning)

            if not self.state_nodes[i].msg_queue.empty():
                msg: String = self.state_nodes[i].msg_queue.get()
                self.state_labels[i].update_data(msg.data)

            if not self.position_nodes[i].msg_queue.empty():
                msg: MotorPosition = self.position_nodes[i].msg_queue.get()
                self.position_labels[i]['target_pos'].update_data(msg.target_position)
                self.position_labels[i]['actual_pos'].update_data(msg.actual_position)

            if not self.velocity_nodes[i].msg_queue.empty():
                msg: Float64 = self.velocity_nodes[i].msg_queue.get()
                self.velocity_labels[i].update_data(msg.data)

            if not self.current_nodes[i].msg_queue.empty():
                msg: Int16 = self.current_nodes[i].msg_queue.get()
                self.current_labels[i].update_data(msg.data)

            if not self.inputs_nodes[i].msg_queue.empty():
                msg: DigitalInputs = self.inputs_nodes[i].msg_queue.get()
                self.inputs_labels[i]['negative_limit_switch'].update_data(msg.negative_limit_switch)
                self.inputs_labels[i]['positive_limit_switch'].update_data(msg.positive_limit_switch)
                self.inputs_labels[i]['homing_switch'].update_data(msg.homing_switch)

            if not self.can_error_nodes[i].msg_queue.empty():
                msg: CanError = self.can_error_nodes[i].msg_queue.get()
                self.can_error_labels[i]['description'].update_data(msg.description)
                self.can_error_labels[i]['code'].update_data(msg.code)
                self.can_error_labels[i]['timestamp'].update_data(msg.timestamp)

            if not self.shut_down_nodes[i].res_queue.empty():
                response: Trigger.Response = self.shut_down_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.switch_on_nodes[i].res_queue.empty():
                response: Trigger.Response = self.switch_on_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.disable_operation_nodes[i].res_queue.empty():
                response: Trigger.Response = self.disable_operation_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.disable_voltage_nodes[i].res_queue.empty():
                response: Trigger.Response = self.disable_voltage_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.enable_operation_nodes[i].res_queue.empty():
                response: Trigger.Response = self.enable_operation_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.quick_stop_nodes[i].res_queue.empty():
                response: Trigger.Response = self.quick_stop_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.fault_reset_nodes[i].res_queue.empty():
                response: Trigger.Response = self.fault_reset_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.save_home_nodes[i].res_queue.empty():
                response: Trigger.Response = self.save_home_nodes[i].res_queue.get()
                self.success_labels[i].update_data(response.success)
                self.message_labels[i].update_data(response.message)

            if not self.move_action_nodes[i].status_queue.empty():
                self.status_labels[i].update_data(self.move_action_nodes[i].status_queue.get())

    def trigger_service_button_callbacks(self, nodes: list[ClientNode], labels: list[TkBoolLabel]):

        for node, label in zip(nodes, labels):
            self.trigger_service_button_callback(node, label)

    def trigger_service_button_callback(self, node: ClientNode, label: TkBoolLabel):

        label.update_data(None)

        node.req_queue.put(Trigger.Request())

    def start_move_button_callback(self,
                                   node: ActionClientNode, 
                                   mode_menu: TkOptionMenu, 
                                   target_position_entry: TkFloatEntry, 
                                   profile_velocity_entry: TkFloatEntry,  
                                   profile_acceleration_entry: TkFloatEntry, 
                                   profile_deceleration_entry: TkFloatEntry,
                                   mode: str = None):
        
        goal = MoveMotor.Goal()
        
        if mode is None:
            mode = mode_menu.get_data()

        if mode == 'Home':
            goal.mode = int(1)
        elif mode == 'Position (abs.)':
            goal.mode = int(0)
            goal.absolute_relative = False
            goal.profile_acceleration = profile_acceleration_entry.get_data()
            goal.profile_deceleration = profile_deceleration_entry.get_data()
            goal.profile_velocity = profile_velocity_entry.get_data()
            goal.target_position = target_position_entry.get_data()
        elif mode == 'Position (rel.)':
            goal.mode = int(0)
            goal.absolute_relative = True
            goal.profile_acceleration = profile_acceleration_entry.get_data()
            goal.profile_deceleration = profile_deceleration_entry.get_data()
            goal.profile_velocity = profile_velocity_entry.get_data()
            goal.target_position = target_position_entry.get_data()
        elif mode == 'Home*':
            goal.mode = int(3)

        node.goal_queue.put(goal)

    def home_star_button_callbacks(self):

        for node in self.move_action_nodes:
            self.start_move_button_callback(node, None, None, None,None, None, 'Home*')

    def stop_move_button_callbacks(self):

        for node in self.move_action_nodes:
            self.stop_move_button_callback(node)

    def stop_move_button_callback(self, node: ActionClientNode):

        node.cancel_event.set()

    def destroy(self) -> None:

        for node in self.move_action_nodes:
            self.stop_move_button_callback(node)

        return super().destroy()
