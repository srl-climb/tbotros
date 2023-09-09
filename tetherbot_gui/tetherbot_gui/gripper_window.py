from __future__ import annotations
from typing import TYPE_CHECKING
from .tkinter_objects import TkLabelFrame, TkLabel, TkBoolLabel, TkButton, \
    TkCancelButton, TkActionStatusLabel, TkPoseLabelFrame, TkOptionMenu, TkStringLabel
from .interfaces import BoolMsgInterface, Int8MsgInterface, EmptySrvInterface, StringMsgInterface, SetStringSrvInterface, \
    PoseStampedMsgInterface, EmptyActionInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App

class GripperWindow(Window):

    def __init__(self, master: App, title: str = 'Grippers'):
        
        super().__init__(master, title, icon_file='srl_icon_grip.png')

    def create_ui(self):

        hold_names = [hold.name for hold in self.master.tbot.wall.holds]

        for gripper, i in zip(self.master.tbot.grippers, range(self.master.tbot.k)):
            
            main_frame = TkLabelFrame(master=self, text = 'Gripper ' + str(i) + ' (' + gripper.name + ')')
            main_frame.grid(row = 0, column = i)

            state_frame = TkLabelFrame(master=main_frame, text='State')
            state_frame.grid(row=0, column=0)

            label_frame = TkPoseLabelFrame(master=state_frame)
            label_frame.grid(row=0, column=0, columnspan=2)

            PoseStampedMsgInterface(master=self, msg_name=gripper.name + '/gripper_state_publisher/pose', label_frame=label_frame)
            
            label = TkLabel(master=state_frame, text='Closed:')
            label.grid(row=2, column=0)
            label = TkBoolLabel(master=state_frame)
            label.grid(row=2, column=1)

            Int8MsgInterface(master=self, msg_name=gripper.name + '/wireless_servo/closed', label=label)
            
            label = TkLabel(master=state_frame, text='Contactswitch:')
            label.grid(row=3, column=0)
            label = TkBoolLabel(master=state_frame)
            label.grid(row=3, column=1)

            BoolMsgInterface(master=self, msg_name=gripper.name + '/wireless_servo/limitswitch0', label=label)
            
            label = TkLabel(master=state_frame, text='Contact Confirmed:')
            label.grid(row=4, column=0)
            label = TkBoolLabel(master=state_frame)
            label.grid(row=4, column=1)
            
            BoolMsgInterface(master=self, msg_name=gripper.name + '/gripper_controller/contact_confirmed', label=label)

            label = TkLabel(master=state_frame, text='Transform Source:')
            label.grid(row=5, column=0)
            label = TkStringLabel(master=state_frame)
            label.grid(row=5, column=1)

            StringMsgInterface(master=self, msg_name=gripper.name + '/gripper_state_publisher/transform_source', label=label)
            
            label = TkLabel(master=state_frame, text='Hold Name:')
            label.grid(row=6, column=0)
            label = TkStringLabel(master=state_frame)
            label.grid(row=6, column=1)

            StringMsgInterface(master=self, msg_name=gripper.name + '/gripper_state_publisher/hold_name', label=label)
            
            service_frame = TkLabelFrame(master=main_frame, text='Services')
            service_frame.grid(row=1, column=0)

            label = TkLabel(master=service_frame, text='Hold Name:')
            label.grid(row=0, column=0)
            option_menu = TkOptionMenu(master=service_frame, values=hold_names)
            option_menu.grid(row=0, column=1)
            button = TkButton(master=service_frame, text='Set Hold')
            button.grid(row=1, column=0, columnspan=2)

            SetStringSrvInterface(master=self, srv_name=gripper.name + '/gripper_state_publisher/set_hold', entry=option_menu, button=button)

            label = TkLabel(master=service_frame, text='Transform Source:')
            label.grid(row=2, column=0)
            option_menu = TkOptionMenu(master=service_frame, values=['arm', 'hold', 'marker'])
            option_menu.grid(row=2, column= 1)  
            button = TkButton(master=service_frame, text='Set Transform Source')
            button.grid(row=3, column=0, columnspan=2)

            SetStringSrvInterface(master=self, srv_name=gripper.name + '/gripper_state_publisher/set_transform_source', entry=option_menu, button=button)
            
            button = TkButton(master=service_frame, text='Confirm Contact')
            button.grid(row=4, column=0, columnspan=2)

            EmptySrvInterface(master=self, srv_name=gripper.name + '/gripper_controller/confirm_contact', button=button)
            
            action_frame = TkLabelFrame(master=main_frame, text='Actions')
            action_frame.grid(row=2, column=0)

            execute_open_button = TkButton(master = action_frame, text = 'Open')
            execute_open_button.grid(row = 0 , column = 0, columnspan = 2)
            execute_close_button = TkButton(master = action_frame, text = 'Close')
            execute_close_button.grid(row = 1, column = 0, columnspan = 2)
            cancel_button = TkCancelButton(master = action_frame)
            cancel_button.grid(row = 2, column = 0, columnspan = 2)
            label = TkLabel(master = action_frame, text = 'Status:')
            label.grid(row = 3, column = 0)
            status_label = TkActionStatusLabel(master = action_frame)
            status_label.grid(row = 3, column = 1)

            open_action_interface = EmptyActionInterface(master=self, action_name=gripper.name + '/gripper_controller/open',
                                                         execute_button=execute_open_button, cancel_button=cancel_button, status_label=status_label)
        
            close_action_interface = EmptyActionInterface(master=self, action_name=gripper.name + '/gripper_controller/close',
                                                          execute_button=execute_close_button, cancel_button=cancel_button, status_label=status_label)
            
            # share the cancel button between both interfaces
            cancel_button.configure(command=lambda interfaces=[open_action_interface, close_action_interface]:
                                        [interface.cancel_button_callback() for interface in interfaces])
