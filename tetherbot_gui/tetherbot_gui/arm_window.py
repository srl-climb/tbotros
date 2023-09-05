from __future__ import annotations
from typing import TYPE_CHECKING
from .tkinter_objects import TkLabelFrame, TkLabel, TkBoolLabel, TkButton, \
    TkCancelButton, TkActionStatusLabel, TkPoseLabelFrame, TkArrayLabel
from .interfaces import BoolMsgInterface, Int8MsgInterface, EmptySrvInterface, \
    PoseStampedMsgInterface, PoseMsgInterface, Float64ArrayMsgInterface, EmptyActionInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App


class ArmWindow(Window):

    def __init__(self, master: App, title: str = 'Arm'):
        
        super().__init__(master, title, icon_file='srl_icon_arm.png')

    def create_ui(self):

        state_frame = TkLabelFrame(master = self, text = 'State')
        state_frame.grid(row = 0, column = 0)

        label_frame = TkPoseLabelFrame(master = state_frame, text = 'Actual Pose:')
        label_frame.grid(row = 0, column = 0, columnspan = 2)

        PoseStampedMsgInterface(master=self, msg_name=self.master.tbot.platform.arm.name + '/arm_state_publisher/pose', label_frame=label_frame)

        label_frame = TkPoseLabelFrame(master = state_frame, text = 'Target Pose:')
        label_frame.grid(row = 1, column = 0, columnspan = 2)

        PoseMsgInterface(master=self, msg_name=self.master.tbot.platform.arm.name + '/arm_controller/target_pose', label_frame=label_frame)

        label = TkLabel(master = state_frame, text = 'Joint States:')
        label.grid(row = 2, column = 0)
        label = TkArrayLabel(length = len(self.master.tbot.platform.arm.links), digits = 3, master = state_frame)
        label.grid(row = 2, column = 1)

        Float64ArrayMsgInterface(master=self, msg_name=self.master.tbot.platform.arm.name + '/arm_state_publisher/joint_states', label=label)

        label = TkLabel(master = state_frame, text = 'Control Enabled:')
        label.grid(row = 3, column = 0)
        label = TkBoolLabel(master = state_frame)
        label.grid(row = 3, column = 1)

        BoolMsgInterface(master=self, msg_name=self.master.tbot.platform.arm.name + '/arm_controller/control_enabled', label=label)

        label = TkLabel(master = state_frame, text = 'Closed: ')
        label.grid(row = 4, column = 0)
        label = TkBoolLabel(master = state_frame)
        label.grid(row = 4, column = 1)

        Int8MsgInterface(master=self, msg_name=self.master.tbot.platform.arm.name + '/wireless_servo_peripheral/servo_closed', label=label)

        label = TkLabel(master = state_frame, text = 'Contact Switch:')
        label.grid(row = 5, column = 0)
        contact_switch_label = TkBoolLabel(master = state_frame)
        contact_switch_label.grid(row = 5, column = 1)

        BoolMsgInterface(master=self, msg_name=self.master.tbot.platform.arm.name + '/wireless_servo_peripheral/limitswitch0', label=label)

        label = TkLabel(master = state_frame, text = 'Contact Confirmed:')
        label.grid(row = 6, column = 0)
        contact_confirmed_label = TkBoolLabel(master = state_frame)
        contact_confirmed_label.grid(row = 6, column = 1)

        BoolMsgInterface(master=self, msg_name=self.master.tbot.platform.arm.name + '/docking_controller/contact_confirmed', label=label)

        service_frame = TkLabelFrame(master = self, text = 'Services')
        service_frame.grid(row = 1, column = 0)

        button = TkButton(master = service_frame, text = 'Enable Control')
        button.grid(row = 0, column = 0)

        EmptySrvInterface(master=self, srv_name=self.master.tbot.platform.arm.name + '/arm_controller/enable_control', button=button)

        button = TkButton(master = service_frame, text = 'Disable Control')
        button.grid(row = 1, column = 0)

        EmptySrvInterface(master=self, srv_name=self.master.tbot.platform.arm.name + '/arm_controller/disable_control', button=button)

        button = TkButton(master = service_frame, text = 'Confirm Contact')
        button.grid(row = 2, column = 0)

        EmptySrvInterface(master=self, srv_name=self.master.tbot.platform.arm.name + '/docking_controller/confirm_contact', button=button)
        
        action_frame = TkLabelFrame(master = self, text = 'Actions')
        action_frame.grid(row = 2, column = 0)

        execute_open_button = TkButton(master = action_frame, text = 'Open')
        execute_open_button.grid(row = 0, column = 0, columnspan = 2)
        execute_close_button = TkButton(master = action_frame, text = 'Close')
        execute_close_button.grid(row = 1, column = 0, columnspan = 2)
        cancel_button = TkCancelButton(master = action_frame)
        cancel_button.grid(row = 2, column = 0, columnspan = 2)
        label = TkLabel(master = action_frame, text = 'Status:')
        label.grid(row = 3, column = 0)
        status_label = TkActionStatusLabel(master = action_frame)
        status_label.grid(row = 3, column = 1)

        open_action_interface = EmptyActionInterface(master=self, action_name= self.master.tbot.platform.arm.name + '/docking_controller/open',
                                                     execute_button=execute_open_button, cancel_button=cancel_button, status_label=status_label)
        
        close_action_interface = EmptyActionInterface(master=self, action_name= self.master.tbot.platform.arm.name + '/docking_controller/close',
                                                      execute_button=execute_close_button, cancel_button=cancel_button, status_label=status_label)
        
        # share the cancel button between both interfaces
        cancel_button.configure(command=lambda interfaces=[open_action_interface, close_action_interface]:
                                        [interface.cancel_button_callback() for interface in interfaces])
