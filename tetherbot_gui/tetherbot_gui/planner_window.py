from __future__ import annotations
from typing import TYPE_CHECKING
from tkinter import ttk
from .tkinter_objects import TkLabelFrame, TkLabel, TkBoolLabel, TkButton, TkPoseEntryFrame, \
    TkCancelButton, TkActionStatusLabel, TkOptionMenu, TkStringLabel, TkEntry
from .interfaces import BoolMsgInterface, EmptySrvInterface, StringMsgInterface, SetStringSrvInterface, \
    TriggerSrvInterface, PlanTetherbotActionInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App

class PlannerWindow(Window):

    def __init__(self, master: App, title: str='Planner'):
        
        super().__init__(master, title, icon_file='srl_icon_plan.png')

    def create_ui(self):

        gripper_names = [gripper.name for gripper in self.master.tbot.grippers]
        hold_names = [hold.name for hold in self.master.tbot.wall.holds]

        state_frame = TkLabelFrame(master=self, text='State')
        state_frame.grid(row=0, column=0)

        label = TkLabel(master=state_frame, text='Busy:')
        label.grid(row=0,column=0)
        label = TkBoolLabel(master=state_frame)
        label.grid(row=0,column=1)

        BoolMsgInterface(master=self, msg_name='planner/busy', label=label)
        
        label = TkLabel(master=state_frame, text='Commands File:')
        label.grid(row=1, column=0)
        label = TkStringLabel(master=state_frame)
        label.grid(row=1, column=1)
        label.configure(width=25)
        label.bind('<Configure>', lambda e: label.config(wraplength=label.winfo_width()-10))

        StringMsgInterface(master=self, msg_name='planner/commands_file', label=label)

        label = TkLabel(master=state_frame, text='Commands Saved:')
        label.grid(row=2, column=0)
        label = TkBoolLabel(master=state_frame)
        label.grid(row=2, column=1)

        BoolMsgInterface(master=self, msg_name='planner/commands_saved', label=label)

        service_frame = TkLabelFrame(master = self, text = 'Services')
        service_frame.grid(row=1, column=0)

        button = TkButton(master=service_frame, text='Display State')
        button.grid(row=0, column=0, columnspan=2)

        EmptySrvInterface(master=self, srv_name='planner/display_state', button=button)

        button = TkButton(master=service_frame, text='Display Commands')
        button.grid(row=1, column=0, columnspan=2)

        EmptySrvInterface(master=self, srv_name='planner/display_commands', button=button)

        label = TkLabel(master=service_frame, text='File Path:')
        label.grid(row=2, column=0)
        entry = TkEntry(master=service_frame)
        entry.grid(row=2, column=1)
        button = TkButton(master=service_frame, text='Set Commands File')
        button.grid(row=3, column=0, columnspan=2)

        SetStringSrvInterface(master=self, srv_name='planner/set_commands_file', entry=entry, button=button)

        button = TkButton(master= service_frame, text='Save Commands')
        button.grid(row=4, column=0, columnspan=2)
        label = TkLabel(master=service_frame, text='Success: ')
        label.grid(row=5, column=0)
        label = TkBoolLabel(master=service_frame)
        label.grid(row=5, column=1)

        TriggerSrvInterface(master=self, srv_name='planner/save_commands', success_label=label, button=button)

        action_frame = TkLabelFrame(master=self, text='Actions')
        action_frame.grid(row=2, column=0)

        notebook = ttk.Notebook(master=action_frame)
        notebook.grid(row=0, column=0)
        
        frame = TkLabelFrame(master=notebook)
        frame.pack(fill='both', expand=True)
        platform_pose_entry_frame = TkPoseEntryFrame(master=frame)
        platform_pose_entry_frame.grid(row=0, column=0)
        notebook.add(frame, text='Platform')

        frame = TkLabelFrame(master=notebook)
        frame.pack(fill='both', expand=True)
        arm_pose_entry_frame = TkPoseEntryFrame(master=frame)
        arm_pose_entry_frame.grid(row=0, column=0)
        notebook.add(frame, text='Arm')

        frame = TkLabelFrame(master=notebook)
        frame.pack(fill='both', expand=True)
        label = TkLabel(master=frame, text='Gripper Name:')
        label.grid(row=0, column=0)
        pick_gripper_menu = TkOptionMenu(master=frame, values=gripper_names)
        pick_gripper_menu.grid(row=0, column=1)
        notebook.add(frame, text='Pick')

        frame = TkLabelFrame(master=notebook)
        frame.pack(fill='both', expand=True)
        label = TkLabel(master=frame, text='Hold Name:')
        label.grid(row=0, column=0)
        place_hold_menu = TkOptionMenu(master=frame, values=hold_names)
        place_hold_menu.grid(row=0, column=1)
        notebook.add(frame, text='Place')

        frame = TkLabelFrame(master=notebook)
        frame.pack(fill='both', expand=True)
        label = TkLabel(master=frame, text='Gripper Name:')
        label.grid(row=0, column=0)
        pick_and_place_gripper_menu = TkOptionMenu(master=frame, values=gripper_names)
        pick_and_place_gripper_menu.grid(row=0, column=1)
        label = TkLabel(master=frame, text='Hold Name:')
        label.grid(row=1, column=0)
        pick_and_place_hold_menu = TkOptionMenu(master=frame, values=hold_names)
        pick_and_place_hold_menu.grid(row=1, column=1)
        notebook.add(frame, text='Pick + Place')

        frame = TkLabelFrame(master=notebook)
        frame.pack(fill='both', expand=True)
        label = TkLabel(master=frame, text='Gripper Name:')
        label.grid(row=0, column=0)
        to_configuration_gripper_menu = TkOptionMenu(master=frame, values=gripper_names)
        to_configuration_gripper_menu.grid(row=0, column=1)
        notebook.add(frame, text='To Config')

        frame = TkLabelFrame(master=notebook)
        frame.pack(fill='both', expand=True)
        global_hold_menus: list[TkOptionMenu] = []
        i = 0
        for gripper_name in gripper_names:
            label = TkLabel(master=frame, text='Gripper ' + str(i) + ' (' + gripper_name + ')')
            label.grid(row=i, column=0)
            menu = TkOptionMenu(master=frame, values=hold_names)
            menu.grid(row=i, column=1)
            global_hold_menus.append(menu)
            i = i + 1
        notebook.add(frame, text='Global')

        frame = TkLabelFrame(master=action_frame)
        frame.configure({"relief":"flat"})
        frame.grid(row=1,column=0)
        execute_button = TkButton(master=frame, text='Plan')
        execute_button.grid(row=0, column=0, columnspan=2)
        cancel_button = TkCancelButton(master=frame)
        cancel_button.grid(row=1, column=0, columnspan=2)
        label = TkLabel(master=frame, text='Current: ')
        label.grid(row=2, column=0)
        feedback_label = TkStringLabel(master=frame)
        feedback_label.grid(row=2,column=1)
        feedback_label.configure(width=25, height=6)
        feedback_label.bind('<Configure>', lambda e: feedback_label.config(wraplength=feedback_label.winfo_width()-10))
        label = TkLabel(master=frame, text='Status:', width=10)
        label.grid(row=3, column=0)
        status_label = TkActionStatusLabel(master=frame)
        status_label.grid(row=3, column=1) 

        PlanTetherbotActionInterface(master=self,
                                     action_name='planner/plan',
                                     execute_button=execute_button,
                                     feedback_label=feedback_label,
                                     cancel_button=cancel_button,
                                     status_label=status_label,
                                     notebook=notebook,
                                     to_configuration_gripper_menu=to_configuration_gripper_menu,
                                     platform_pose_entry_frame=platform_pose_entry_frame,
                                     arm_pose_entry_frame=arm_pose_entry_frame,
                                     pick_gripper_menu=pick_gripper_menu,
                                     place_hold_menu=place_hold_menu,
                                     pick_and_place_gripper_menu=pick_and_place_gripper_menu,
                                     pick_and_place_hold_menu=pick_and_place_hold_menu,
                                     global_hold_menus=global_hold_menus)
