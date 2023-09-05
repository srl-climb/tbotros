from __future__ import annotations
from typing import TYPE_CHECKING
from .tkinter_objects import TkLabelFrame, TkLabel, TkBoolLabel, TkButton, TkStringLabel, \
    TkCancelButton, TkActionStatusLabel, TkPoseLabelFrame, TkArrayLabel, TkOptionMenu, TkFloatLabel, TkErrorCodeLabel
from .interfaces import BoolMsgInterface, TensionSrvInterface, EmptySrvInterface, StringMsgInterface, TriggerSrvInterface, Int8MsgInterface, \
    PoseStampedMsgInterface, SetStringSrvInterface, Float64ArrayMsgInterface, EmptyActionInterface, BoolArrayMsgInterface, Float64MsgInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App


class ServoWindow(Window):

    def __init__(self, master: App, title: str = 'Wireless Servo'):
        
        super().__init__(master, title, icon_file='srl_icon_servo.png')

    def create_ui(self):

        arduinos = []
        arduinos.append({'name': 'Arduino Central 0', 'node': 'wireless_servo_central', 'type': 'central'})
        for i in range(5):
            arduinos.append({'name': 'Arduino Peripheral ' + str(i), 'node': self.master.tbot.grippers[i].name + '/wireless_servo_peripheral', 'type': 'peripheral'})
        arduinos.append({'name': 'Arduino Peripheral ' + str(i+1), 'node': self.master.tbot.platform.arm.name + '/wireless_servo_peripheral', 'type': 'peripheral'})

        master_frame = TkLabelFrame(master=self, text='Master')
        master_frame.grid(row=0, column=0, sticky="N")
        master_service_frame = TkLabelFrame(master=master_frame, text='Services')
        master_service_frame.pack()
        clear_all_button = TkButton(master=master_service_frame, width=10, text='Clear All')
        clear_all_button.grid(row=0, column=0, columnspan=2)
        reset_all_button = TkButton(master=master_service_frame, width=10, text='Reset All')
        reset_all_button.grid(row=1, column=0, columnspan=2)

        clear_srv_interfaces: list[TriggerSrvInterface] = []
        reset_srv_interfaces: list[TriggerSrvInterface] = []

        for (arduino, i) in zip(arduinos, range(1, len(arduinos)+1)):

            arduino_frame = TkLabelFrame(master=self, text=arduino['name'])
            arduino_frame.grid(row=0, column=i, sticky="N")

            state_labelframe = TkLabelFrame(master=arduino_frame, text='State')
            state_labelframe.grid(row=0, column=0)

            label = TkLabel(master=state_labelframe, text="Local name:")
            label.grid(row=0, column=0)
            label = TkStringLabel(master=state_labelframe)
            label.grid(row=0, column=1)

            StringMsgInterface(master=self, msg_name=arduino['node'] + '/arduino_local_name', label=label)

            label = TkLabel(master=state_labelframe, text="Connected:")
            label.grid(row=1, column=0)
            label = TkBoolLabel(master=state_labelframe)
            label.grid(row=1, column=1)

            BoolMsgInterface(master=self, msg_name=arduino['node'] + '/arduino_connected', label=label)

            label = TkLabel(master=state_labelframe, text="Error code:")
            label.grid(row=2, column=0)
            label = TkErrorCodeLabel(master=state_labelframe)
            label.grid(row=2, column=1)

            Int8MsgInterface(master=self, msg_name=arduino['node'] + '/arduino_error_code', label=label)

            if arduino['type'] == 'peripheral':
                label = TkLabel(master=state_labelframe, text="Servo Running:")
                label.grid(row=3, column=0)
                label = TkBoolLabel(master=state_labelframe)
                label.grid(row=3, column=1)

                BoolMsgInterface(master=self, msg_name=arduino['node'] + '/servo_running', label=label)

                label = TkLabel(master=state_labelframe, text="Servo Closed:")
                label.grid(row=4, column=0)
                label = TkBoolLabel(master=state_labelframe)
                label.grid(row=4, column=1)

                BoolMsgInterface(master=self, msg_name=arduino['node'] + '/servo_closed', label=label)

                label = TkLabel(master=state_labelframe, text="Limitswitch 0:")
                label.grid(row=5, column=0)
                label = TkBoolLabel(master=state_labelframe)
                label.grid(row=5, column=1)

                BoolMsgInterface(master=self, msg_name=arduino['node'] + '/limitswitch0', label=label)

                label = TkLabel(master=state_labelframe, text="Limitswitch 1:")
                label.grid(row=6, column=0)
                label = TkBoolLabel(master=state_labelframe)
                label.grid(row=6, column=1)

                BoolMsgInterface(master=self, msg_name=arduino['node'] + '/limitswitch1', label=label)

            service_labelframe = TkLabelFrame(master=arduino_frame, text='Services')
            service_labelframe.grid(row=1, column=0)

            label = TkLabel(master=service_labelframe, text="Success:")
            label.grid(row=2, column=0)
            success_label = TkBoolLabel(master=service_labelframe)
            success_label.grid(row=2, column=1)
            button = TkButton(master=service_labelframe, width=10, text='Clear')
            button.grid(row=0, column=0, columnspan=2)

            clear_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name=arduino['node'] + '/clear_arduino', button=button, success_label=success_label))

            button = TkButton(master=service_labelframe, width=10, text='Reset')
            button.grid(row=1, column=0, columnspan=2)

            reset_srv_interfaces.append(
                TriggerSrvInterface(master=self, srv_name=arduino['node'] + '/clear_arduino', button=button, success_label=success_label))

            if arduino['type'] == 'peripheral':
                action_labelframe = TkLabelFrame(master=arduino_frame, text='Actions')
                action_labelframe.grid(row=2, column=0)

                label = TkLabel(master=action_labelframe, text="Status:")
                label.grid(row=3, column=0)
                status_label = TkActionStatusLabel(master=action_labelframe)
                status_label.grid(row=3, column=1)
                execute_open_button = TkButton(master=action_labelframe, width=10, text='Open Servo')
                execute_open_button.grid(row=0, column=0, columnspan=2)
                execute_close_button = TkButton(master=action_labelframe, width=10, text='Close Servo')
                execute_close_button.grid(row=1, column=0, columnspan=2)
                cancel_button = TkCancelButton(master=action_labelframe, width=10)
                cancel_button.grid(row=2, column=0, columnspan=2)

                open_action_interface = EmptyActionInterface(master=self, action_name=arduino['node'] + '/servo_open', 
                                                             execute_button=execute_open_button, 
                                                             cancel_button=cancel_button, 
                                                             status_label=status_label)
                
                close_action_interface = EmptyActionInterface(master=self, action_name=arduino['node'] + '/servo_close', 
                                                              execute_button=execute_open_button, 
                                                              cancel_button=cancel_button, 
                                                              status_label=status_label)
                
                # share the cancel button between both interfaces
                cancel_button.configure(command=lambda interfaces=[open_action_interface, close_action_interface]:
                                        [interface.cancel_button_callback() for interface in interfaces])
                
            clear_all_button.configure(command=lambda interfaces=clear_srv_interfaces: [interface.button_callback() for interface in interfaces])
            reset_all_button.configure(command=lambda interfaces=reset_srv_interfaces: [interface.button_callback() for interface in interfaces])