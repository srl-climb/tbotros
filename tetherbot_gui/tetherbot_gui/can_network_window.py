from __future__ import annotations
from typing import TYPE_CHECKING
from .tkinter_objects import TkOptionMenu, TkLabelFrame, TkLabel, TkBoolLabel, TkStringLabel, TkButton, TkFloatEntry, \
    TkCancelButton, TkActionStatusLabel, TkScrollFrame, TkFloatLabel
from .interfaces import Float64MsgInterface, Int16MsgInterface, TriggerSrvInterface, MoveMotorActionInterface, \
    StatuswordMsgInterface, MotorPositionInterface, DigitalInputsMsgInterface, StringMsgInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App


class CanNetworkWindow(Window):

    def __init__(self, master: App, title: str = 'CAN Network'):
        
        super().__init__(master, title, icon_file='srl_icon_can.png')

    def create_ui(self):

        motor_namespaces = ['motor' + str(i) for i in range(13)]
        
        frame = TkScrollFrame(master=self)

        master_frame = TkLabelFrame(master=frame, text='Master')
        master_frame.grid(row=0, column=0, sticky='N')

        service_frame = TkLabelFrame(master=master_frame, text='Services')
        service_frame.grid(row=0,column=0)

        shut_down_all_button = TkButton(master = service_frame, width = 13, text = 'Shut Down')
        shut_down_all_button.grid(row = 0, column = 0)

        switch_on_all_button = TkButton(master = service_frame, width = 13, text = 'Switch On')
        switch_on_all_button.grid(row = 0, column = 1)

        disable_voltage_all_button = TkButton(master = service_frame, width = 13, text = 'Disable Voltage')
        disable_voltage_all_button.grid(row = 1, column = 0)

        quick_stop_all_button = TkButton(master = service_frame, width = 13, text = 'Quick Stop')
        quick_stop_all_button.config(bg="#FFC0CB")
        quick_stop_all_button.grid(row = 1, column = 1)

        disable_operation_all_button = TkButton(master = service_frame, width = 13, text = 'Disable Operation')
        disable_operation_all_button.grid(row = 2, column = 0)

        enable_operation_all_button = TkButton(master = service_frame, width = 13, text = 'Enable Operation')
        enable_operation_all_button.grid(row = 2, column = 1)

        fault_reset_all_button = TkButton(master = service_frame, width = 13, text = 'Fault Reset')
        fault_reset_all_button.grid(row = 3, column = 0)

        save_home_all_button = TkButton(master = service_frame, width = 13, text = 'Save Home*')
        save_home_all_button.grid(row = 3, column = 1)

        action_frame = TkLabelFrame(master = master_frame, text = 'Actions')
        action_frame.grid(row=1,column=0)

        button = TkButton(master = action_frame, text = 'Home*', width = 13, command=self.home_all_button_callback)
        button.grid(row=0,column=0)
        button = TkButton(master = action_frame, text = 'CSP', width = 13, command=self.csp_all_button_callback)
        button.grid(row=0,column=1)
        button = TkButton(master=action_frame, text='Stop', width = 13, command=self.stop_all_button_callback)
        button.grid(row=1, column = 0)
        button.config(bg="#FFC0CB")

        shut_down_srv_interfaces: list[TriggerSrvInterface] = []
        switch_on_srv_interfaces: list[TriggerSrvInterface] = []
        disable_voltage_srv_interfaces: list[TriggerSrvInterface] = []
        quick_stop_srv_interfaces: list[TriggerSrvInterface] = []
        disable_operation_srv_interfaces: list[TriggerSrvInterface] = []
        enable_operation_srv_interfaces: list[TriggerSrvInterface] = []
        fault_reset_srv_interfaces: list[TriggerSrvInterface] = []
        save_home_srv_interfaces: list[TriggerSrvInterface] = []
        self.move_action_interfaces: list[MoveMotorActionInterface] = []

        for motor_namespace, i in zip(motor_namespaces, range(len(motor_namespaces))):

            motor_labelframe = TkLabelFrame(master=frame, text='Motor ' + str(i))
            motor_labelframe.grid(row=0, column=1+i)

            state_frame = TkLabelFrame(master = motor_labelframe, text = 'State')
            state_frame.grid(row = 0 , column = 0, columnspan=2)

            statusword_frame = TkLabelFrame(master=state_frame, text='Statusword')
            statusword_frame.grid(row = 0 , column = 0, columnspan=2)

            label = TkLabel(master=statusword_frame, text="Ready to switch on:")
            label.grid(row=0, column=0)
            ready_to_switch_on_label = TkBoolLabel(master=statusword_frame)
            ready_to_switch_on_label.grid(row=0, column=1)

            label = TkLabel(master=statusword_frame, text="Switched on:")
            label.grid(row=1, column=0)
            switched_on_label = TkBoolLabel(master=statusword_frame)
            switched_on_label.grid(row=1, column=1)

            label = TkLabel(master=statusword_frame, text="Voltage enabled:")
            label.grid(row=2, column=0)
            voltage_enabled_label = TkBoolLabel(master=statusword_frame)
            voltage_enabled_label.grid(row=2, column=1)

            label = TkLabel(master=statusword_frame, text="Operation enabled:")
            label.grid(row=3, column=0)
            operation_enabled_label = TkBoolLabel(master=statusword_frame)
            operation_enabled_label.grid(row=3, column=1)

            label = TkLabel(master=statusword_frame, text="Target reached:")
            label.grid(row=4, column=0)
            target_reached_label = TkBoolLabel(master=statusword_frame)
            target_reached_label.grid(row=4, column=1)

            label = TkLabel(master=statusword_frame, text="Quick stop:")
            label.grid(row=5, column=0)
            quick_stop_label = TkBoolLabel(master=statusword_frame)
            quick_stop_label.grid(row=5, column=1)

            label = TkLabel(master=statusword_frame, text="Switch on disabled:")
            label.grid(row=6, column=0)
            switch_on_disabled_label = TkBoolLabel(master=statusword_frame)
            switch_on_disabled_label.grid(row=6, column=1)

            label = TkLabel(master=statusword_frame, text="Set point acknowledge:")
            label.grid(row=7, column=0)
            setpoint_acknowledge_label = TkBoolLabel(master=statusword_frame)
            setpoint_acknowledge_label.grid(row=7, column=1)

            label = TkLabel(master=statusword_frame, text="Internal limit active:")
            label.grid(row=8, column=0)
            internal_limit_active_label = TkBoolLabel(master=statusword_frame)
            internal_limit_active_label.grid(row=8, column=1)

            label = TkLabel(master=statusword_frame, text="Deviation error:")
            label.grid(row=9, column=0)
            deviation_error_label = TkBoolLabel(master=statusword_frame)
            deviation_error_label.grid(row=9, column=1)

            label = TkLabel(master=statusword_frame, text="Fault:")
            label.grid(row=10, column=0)
            fault_label = TkBoolLabel(master=statusword_frame)
            fault_label.grid(row=10, column=1)

            label = TkLabel(master=statusword_frame, text="Warning:")
            label.grid(row=11, column=0)
            warning_label = TkBoolLabel(master=statusword_frame)
            warning_label.grid(row=11, column=1)

            StatuswordMsgInterface(master=self, msg_name = motor_namespace + '/faulhaber_motor/statusword',
                                   ready_to_switch_on_label=ready_to_switch_on_label, 
                                   switched_on_label=switched_on_label,
                                   voltage_enabled_label=voltage_enabled_label,
                                   operation_enabled_label=operation_enabled_label,
                                   target_reached_label=target_reached_label,
                                   quick_stop_label=quick_stop_label,
                                   switch_on_disabled_label=switch_on_disabled_label,
                                   setpoint_acknowledge_label=setpoint_acknowledge_label,
                                   internal_limit_active_label=internal_limit_active_label,
                                   deviation_error_label=deviation_error_label,
                                   fault_label=fault_label,
                                   warning_label=warning_label)

            label = TkLabel(master=state_frame, text="State:")
            label.grid(row = 1 , column = 0)
            label = TkStringLabel(master=state_frame)
            label.grid(row=1, column=1)

            StringMsgInterface(master=self, msg_name = motor_namespace + '/faulhaber_motor/state', label=label)

            label = TkLabel(master=state_frame, text="Actual Position [m; deg]:")
            label.grid(row=2, column=0)
            actual_position_label = TkFloatLabel(master=state_frame)
            actual_position_label.grid(row=2, column=1)

            label = TkLabel(master=state_frame, text="Target Position [m; deg]:")
            label.grid(row=3, column=0)
            target_postion_label = TkFloatLabel(master=state_frame)
            target_postion_label.grid(row=3, column=1)

            MotorPositionInterface(master=self, msg_name = motor_namespace + '/faulhaber_motor/position',
                                   target_position_label=target_postion_label,
                                   actual_position_label=actual_position_label)

            label = TkLabel(master=state_frame, text="Velocity [m/s; deg/s]:")
            label.grid(row=4, column=0)
            velocity_label = TkFloatLabel(master=state_frame)
            velocity_label.grid(row=4, column=1)
            
            Float64MsgInterface(master=self, msg_name = motor_namespace + '/faulhaber_motor/velocity', label=velocity_label)

            label = TkLabel(master=state_frame, text="Current [mA]:")
            label.grid(row=5, column=0)
            current_label = TkFloatLabel(master=state_frame)
            current_label.grid(row=5, column=1)
            
            Int16MsgInterface(master=self, msg_name = motor_namespace + '/faulhaber_motor/current', label=current_label)

            input_frame = TkLabelFrame(master=state_frame, text='Inputs')
            input_frame.grid(row = 6 , column = 0, columnspan=2)

            label = TkLabel(master=input_frame, text="Negative limit switch:")
            label.grid(row=0, column=0)
            negative_limit_switch_label = TkBoolLabel(master=input_frame)
            negative_limit_switch_label.grid(row=0, column=1)

            label = TkLabel(master=input_frame, text="Positive limit switch:")
            label.grid(row=1, column=0)
            positive_limit_switch_label = TkBoolLabel(master=input_frame)
            positive_limit_switch_label.grid(row=1, column=1)

            label = TkLabel(master=input_frame, text="Homing switch:")
            label.grid(row=2, column=0)
            homing_switch_label = TkBoolLabel(master=input_frame)
            homing_switch_label.grid(row=2, column=1)

            DigitalInputsMsgInterface(master=self, msg_name = motor_namespace + '/faulhaber_motor/inputs', 
                                      negative_limit_switch_label=negative_limit_switch_label,
                                      positive_limit_switch_label=positive_limit_switch_label,
                                      homing_switch_label=homing_switch_label)
            
            service_frame = TkLabelFrame(master = motor_labelframe, text = 'Services')
            service_frame.grid(row = 1 , column = 0, columnspan=2)

            label = TkLabel(master=service_frame, text="Success:")
            label.grid(row=4, column=0)
            success_label = TkBoolLabel(master=service_frame)
            success_label.grid(row=4, column=1)
            
            label = TkLabel(master=service_frame, text="Message:")
            label.grid(row=5, column=0)
            message_label = TkStringLabel(master=service_frame)
            message_label.grid(row=5, column=1)

            button = TkButton(master = service_frame, width = 13, text = 'Shut Down')
            button.grid(row = 0, column = 0)

            shut_down_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/shut_down',
                                    success_label=success_label, message_label=message_label, button = button))

            button = TkButton(master = service_frame, width = 13, text = 'Switch On')
            button.grid(row = 0, column = 1)

            switch_on_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/switch_on',
                                    success_label=success_label, message_label=message_label, button = button))

            button = TkButton(master = service_frame, width = 13, text = 'Disable Voltage')
            button.grid(row = 1, column = 0)

            disable_voltage_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/disable_voltage',
                                    success_label=success_label, message_label=message_label, button = button))

            button = TkButton(master = service_frame, width = 13, text = 'Quick Stop')
            button.config(bg="#FFC0CB")
            button.grid(row = 1, column = 1)

            quick_stop_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/quick_stop',
                                    success_label=success_label, message_label=message_label, button = button))

            button = TkButton(master = service_frame, width = 13, text = 'Disable Operation')
            button.grid(row = 2, column = 0)

            disable_operation_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/disable_operation',
                                    success_label=success_label, message_label=message_label, button = button))

            button = TkButton(master = service_frame, width = 13, text = 'Enable Operation')
            button.grid(row = 2, column = 1)

            enable_operation_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/enable_operation',
                                    success_label=success_label, message_label=message_label, button = button))

            button = TkButton(master = service_frame, width = 13, text = 'Save Home*')
            button.grid(row = 3, column = 1)

            save_home_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/save_home',
                                    success_label=success_label, message_label=message_label, button = button))

            button = TkButton(master = service_frame, width = 13, text = 'Fault Reset')
            button.grid(row = 3, column = 0)

            fault_reset_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name = motor_namespace + '/faulhaber_motor/fault_reset',
                                    success_label=success_label, message_label=message_label, button = button))
            
            
            action_frame = TkLabelFrame(master = motor_labelframe, text = 'Actions')
            action_frame.grid(row = 2, column = 0, columnspan=2)

            label = TkLabel(master=action_frame, text="Mode:")
            label.grid(row=0, column=0)
            mode_menu = TkOptionMenu(master = action_frame, values=['Home', 'Position (abs.)', 'Position (rel.)', 'Home*', 'CSP'])
            mode_menu.grid(row = 0, column = 1)

            label = TkLabel(master=action_frame, text="Target Position:")
            label.grid(row=1, column=0)
            target_position_entry = TkFloatEntry(master=action_frame)
            target_position_entry.grid(row=1, column=1)

            label = TkLabel(master=action_frame, text="Profile Velocity:")
            label.grid(row=2, column=0)
            profile_velocity_entry = TkFloatEntry(master=action_frame)
            profile_velocity_entry.grid(row=2, column=1)

            label = TkLabel(master=action_frame, text="Profile Acceleration:")
            label.grid(row=3, column=0)
            profile_acceleration_entry = TkFloatEntry(master=action_frame, min_val = 0)
            profile_acceleration_entry.grid(row=3, column=1)

            label = TkLabel(master=action_frame, text="Profile Deceleration:")
            label.grid(row=4, column=0)
            profile_deceleration_entry = TkFloatEntry(master=action_frame, min_val = 0)
            profile_deceleration_entry.grid(row=4, column=1)

            execute_button = TkButton(master = action_frame, text='Start', width = 13)
            execute_button.grid(row=5, column=0)
            cancel_button = TkCancelButton(master = action_frame, text='Stop', width = 13)
            cancel_button.grid(row=5, column=1)

            label = TkLabel(master = action_frame, text = 'Status: ')
            label.grid(row=6, column=0)
            status_label = TkActionStatusLabel(master = action_frame)
            status_label.grid(row=6, column=1)

            self.move_action_interfaces.append(
                MoveMotorActionInterface(master=self, action_name=motor_namespace + '/faulhaber_motor/move',
                                         execute_button=execute_button,
                                         cancel_button=cancel_button,
                                         status_label=status_label,
                                         mode_menu=mode_menu,
                                         profile_acceleration_entry=profile_acceleration_entry,
                                         profile_deceleration_entry=profile_deceleration_entry,
                                         profile_velocity_entry=profile_velocity_entry,
                                         target_position_entry=target_position_entry))
            
        shut_down_all_button.configure(command=lambda interfaces=shut_down_srv_interfaces: [interface.button_callback() for interface in interfaces])
        switch_on_all_button.configure(command=lambda interfaces=switch_on_srv_interfaces: [interface.button_callback() for interface in interfaces])
        disable_voltage_all_button.configure(command=lambda interfaces=disable_voltage_srv_interfaces: [interface.button_callback() for interface in interfaces])
        quick_stop_all_button.configure(command=lambda interfaces=quick_stop_srv_interfaces: [interface.button_callback() for interface in interfaces])
        disable_operation_all_button.configure(command=lambda interfaces=disable_operation_srv_interfaces: [interface.button_callback() for interface in interfaces])
        enable_operation_all_button.configure(command=lambda interfaces=enable_operation_srv_interfaces: [interface.button_callback() for interface in interfaces])
        fault_reset_all_button.configure(command=lambda interfaces=fault_reset_srv_interfaces: [interface.button_callback() for interface in interfaces])
        save_home_all_button.configure(command=lambda interfaces=save_home_srv_interfaces: [interface.button_callback() for interface in interfaces])

    def home_all_button_callback(self):

        for interface in self.move_action_interfaces:
            interface.mode_menu.variable.set(interface.mode_menu.values[3])
            interface.execute_button_callback()

    def csp_all_button_callback(self):

        for interface in self.move_action_interfaces:
            interface.mode_menu.variable.set(interface.mode_menu.values[4])
            interface.execute_button_callback()

    def stop_all_button_callback(self):

        for interface in self.move_action_interfaces:
            interface.cancel_button_callback()
