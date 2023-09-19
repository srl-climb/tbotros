from __future__ import annotations
from typing import TYPE_CHECKING
from .tkinter_objects import TkLabelFrame, TkLabel, TkBoolLabel, TkButton, TkStringLabel, \
    TkCancelButton, TkActionStatusLabel, TkPoseLabelFrame, TkArrayLabel, TkOptionMenu, TkFloatLabel
from .interfaces import BoolMsgInterface, TensionSrvInterface, EmptySrvInterface, StringMsgInterface, TriggerSrvInterface, \
    PoseStampedMsgInterface, SetStringSrvInterface, Float64ArrayMsgInterface, EmptyActionInterface, BoolArrayMsgInterface, Float64StampedMsgInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App


class PlatformWindow(Window):

    def __init__(self, master: App, title: str = 'Platform'):
        
        super().__init__(master, title, icon_file='srl_icon_plat.png')

    def create_ui(self):

        gripper_names = [gripper.name for gripper in self.master.tbot.grippers]

        state_frame = TkLabelFrame(master=self, text='State')
        state_frame.grid(row=0, column=0)

        label_frame = TkPoseLabelFrame(master=state_frame, text='Actual Pose:')
        label_frame.grid(row=0, column=0, columnspan=2)

        PoseStampedMsgInterface(master=self, msg_name=self.master.tbot.platform.name + '/platform_state_publisher/pose', label_frame=label_frame)

        label_frame = TkPoseLabelFrame(master=state_frame, text='Target Pose:')
        label_frame.grid(row=1, column=0, columnspan=2)

        PoseStampedMsgInterface(master=self, msg_name=self.master.tbot.platform.name + '/platform_controller/target_pose', label_frame=label_frame)

        label = TkLabel(master=state_frame, text='Joint States:')
        label.grid(row=2, column=0)
        label = TkArrayLabel(length=self.master.tbot.platform.m, digits=3, master=state_frame)
        label.grid(row=2, column=1)

        Float64ArrayMsgInterface(master=self, msg_name=self.master.tbot.platform.name + '/platform_state_publisher/joint_states', label=label)

        label = TkLabel(master=state_frame, text='Control Enabled:')
        label.grid(row=3, column=0)
        label = TkBoolLabel(master=state_frame)
        label.grid(row=3, column=1)

        BoolMsgInterface(master=self, msg_name=self.master.tbot.platform.name + '/platform_controller/control_enabled', label=label)

        label = TkLabel(master=state_frame, text='Transform Source:')
        label.grid(row=4, column=0)
        label = TkStringLabel(master=state_frame)
        label.grid(row=4, column=1)

        StringMsgInterface(master=self, msg_name=self.master.tbot.platform.name + '/platform_state_publisher/transform_source', label=label)

        label = TkLabel(master=state_frame, text='Tether Tension:')
        label.grid(row=5, column=0)
        label = TkArrayLabel(master=state_frame, length=self.master.tbot.platform.m, digits=0)
        label.grid(row=5, column=1)

        BoolArrayMsgInterface(master=self, msg_name=self.master.tbot.platform.name + '/platform_controller/tether_tension', label=label)

        label = TkLabel(master=state_frame, text='Stability:')
        label.grid(row=6, column=0)
        label = TkFloatLabel(master=state_frame, digits=3)
        label.grid(row=6, column=1)

        Float64StampedMsgInterface(master=self, msg_name=self.master.tbot.platform.name + '/platform_state_publisher/stability', label=label)

        service_frame = TkLabelFrame(master=self, text='Services')
        service_frame.grid(row=1, column=0)

        button = TkButton(master=service_frame, text='Enable Control')
        button.grid(row=0, column=0, columnspan=2)

        EmptySrvInterface(master=self, srv_name=self.master.tbot.platform.name + '/platform_controller/enable_control', button=button)
        
        button = TkButton(master=service_frame, text='Disable Control')
        button.grid(row=1, column=0, columnspan=2)

        EmptySrvInterface(master=self, srv_name=self.master.tbot.platform.name + '/platform_controller/disable_control', button=button)
        
        label = TkLabel(master=service_frame, text='Transform Source:')
        label.grid(row=2, column=0)
        option_menu = TkOptionMenu(master=service_frame, values=['optitrack', 'zed', 'fwk'])
        option_menu.grid(row=2, column=1)
        button = TkButton(master=service_frame, text='Set Transform Source')
        button.grid(row=3, column=0, columnspan=2)

        SetStringSrvInterface(master=self, srv_name=self.master.tbot.platform.name + '/platform_state_publisher/set_transform_source', 
                              button=button, entry=option_menu)

        label = TkLabel(master=service_frame, text='Gripper Name:')
        label.grid(row=4, column=0)
        option_menu = TkOptionMenu(master=service_frame, values=gripper_names)
        option_menu.grid(row=4, column=1)
        label = TkLabel(master=service_frame, text='Status:')
        label.grid(row=8, column=0)
        success_label = TkBoolLabel(master=service_frame)
        success_label.grid(row=8, column=1)
        button = TkButton(master=service_frame, text='Tension Tethers')
        button.grid(row=5, column=0, columnspan=2)

        TensionSrvInterface(master=self, srv_name=self.master.tbot.platform.name + '/platform_controller/tension_gripper_tethers', 
                            button=button, success_label=success_label, option_menu=option_menu, tension_value=True)

        button = TkButton(master=service_frame, text='Untension Tethers')
        button.grid(row=6, column=0, columnspan=2)

        TensionSrvInterface(master=self, srv_name=self.master.tbot.platform.name + '/platform_controller/tension_gripper_tethers', 
                            button=button, success_label=success_label, option_menu=option_menu, tension_value=False)

        button = TkButton(master=service_frame, text='Calibrate ZED Pose')
        button.grid(row=7, column=0, columnspan=2)

        TriggerSrvInterface(master=self, srv_name=self.master.tbot.platform.name + '/platform_state_publisher/calibrate_zed_pose',
                            button=button, success_label=success_label)

        action_frame = TkLabelFrame(master=self, text='Actions')
        action_frame.grid(row=2, column=0)

        execute_button = TkButton(master=action_frame, text='Calibrate Tether Lengths')
        execute_button.grid(row=0, column=0, columnspan=2)
        cancel_button = TkCancelButton(master=action_frame)
        cancel_button.grid(row=1, column=0, columnspan=2)
        status_label = TkLabel(master=action_frame, text='Status:')
        status_label.grid(row=2, column=0)
        status_label = TkActionStatusLabel(master=action_frame)
        status_label.grid(row=2, column=1)

        EmptyActionInterface(master=self, action_name=self.master.tbot.platform.name + '/platform_controller/calibrate_tether_lengths',
                             execute_button=execute_button, cancel_button=cancel_button, status_label=status_label)