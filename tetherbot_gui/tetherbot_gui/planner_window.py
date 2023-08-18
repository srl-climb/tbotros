from __future__ import annotations
import tkinter as tk
import numpy as np
from tkinter import ttk
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, Empty as EmptyService
from custom_srvs.srv import SetString
from custom_actions.action import Empty as ExecuteSequence, PlanTetherbot
from geometry_msgs.msg import Pose
from .tkinter_objects import TkOptionMenu
from .window import Window
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App

    
class PlannerWindow(Window):

    def __init__(self, parent: App, title: str = 'Planner Window'):

        super().__init__(parent, title)

    def create_user_interface(self):

        self._gripper_names = [gripper.name for gripper in self.parent.tbot.grippers]
        self._hold_names = [hold.name for hold in self.parent.tbot.wall.holds]

        self._busy_node = self.create_subscriber_node(msg_type=Bool, msg_name='planner/busy')
        self._commands_path_node = self.create_subscriber_node(msg_type=String, msg_name='planner/commands_path')
        self._commands_saved_node = self.create_subscriber_node(msg_type=Bool, msg_name='planner/commands_saved')
        self._display_state_node = self.create_client_node(srv_type=EmptyService, srv_name='planner/display_state', enable_response = False)
        self._display_commands_node = self.create_client_node(srv_type=EmptyService, srv_name='planner/display_commands', enable_response = False)
        self._set_commands_path_node = self.create_client_node(srv_type=SetString, srv_name='planner/set_commands_path', enable_response = False)
        self._save_commands_node = self.create_client_node(srv_type=Trigger, srv_name='planner/save_commands')
        self._plan_node = self.create_action_client_node(action_type=PlanTetherbot, action_name='planner/plan', auto_cancel = True, enable_feedback = True)

        state_frame = self.create_label_frame(master = self, text = 'State')
        state_frame.grid(row = 0, column = 0)

        label = self.create_label(master=state_frame, text='Busy:')
        label.grid(row=0,column=0)
        self._busy_label = self.create_bool_label(master=state_frame)
        self._busy_label.grid(row=0,column=1)
        label = self.create_label(master=state_frame, text='Commands Path:')
        label.grid(row=1, column=0)
        self._commands_path_label = self.create_string_label(master=state_frame)
        self._commands_path_label.grid(row=1, column=1)
        self._commands_path_label.configure(width = 25)
        self._commands_path_label.bind('<Configure>', lambda e: self._commands_path_label.config(wraplength=self._commands_path_label.winfo_width()-10))
        label = self.create_label(master=state_frame, text='Commands Saved:')
        label.grid(row=2, column=0)
        self._commands_saved_label = self.create_bool_label(master=state_frame)
        self._commands_saved_label.grid(row=2, column=1)

        service_frame = self.create_label_frame(master = self, text = 'Services')
        service_frame.grid(row = 1, column = 0)

        button = self.create_button(master = service_frame, text = 'Display State', command=self.display_state_button_callback)
        button.grid(row = 0, column = 0, columnspan = 2)
        button = self.create_button(master = service_frame, text = 'Display Commands', command=self.display_commands_button_callback)
        button.grid(row = 1, column = 0, columnspan = 2)
        label = self.create_label(master = service_frame, text = 'File Path:')
        label.grid(row = 2, column = 0)
        self._set_commands_path_entry = self.create_entry(master = service_frame)
        self._set_commands_path_entry.grid(row = 2, column = 1)
        button = self.create_button(master= service_frame, text = 'Set Commands Path', command=self.set_commands_path_button_callback)
        button.grid(row = 3, column = 0, columnspan = 2)
        button = self.create_button(master= service_frame, text = 'Save Commands', command=self.save_commands_button_callback)
        button.grid(row = 4, column = 0, columnspan = 2)
        label = self.create_label(master = service_frame, text = 'Success: ')
        label.grid(row = 5, column = 0)
        self._save_success_label = self.create_bool_label(master = service_frame)
        self._save_success_label.grid(row = 5, column = 1)

        action_frame = self.create_label_frame(master = self, text = 'Actions')
        action_frame.grid(row = 2, column = 0)

        notebook = ttk.Notebook(master = action_frame)
        notebook.grid(row=0, column=0)
        
        frame = self.create_label_frame(master = notebook)
        frame.pack(fill='both', expand=True)
        self._plan_platform_pose_entry = self.create_pose_entry_frame(master = frame)
        self._plan_platform_pose_entry.grid(row = 0, column = 0)
        button = self.create_button(master = frame, text = 'Plan', command=self.plan_platform_button_callback)
        button.grid(row = 1, column = 0)
        notebook.add(frame, text = 'Platform')

        frame = self.create_label_frame(master = notebook)
        frame.pack(fill='both', expand=True)
        self._plan_arm_pose_entry = self.create_pose_entry_frame(master = frame)
        self._plan_arm_pose_entry.grid(row = 0, column = 0)
        button = self.create_button(master = frame, text = 'Plan', command=self.plan_arm_button_callback)
        button.grid(row = 1, column = 0)
        notebook.add(frame, text = 'Arm')

        frame = self.create_label_frame(master = notebook)
        frame.pack(fill='both', expand=True)
        label = self.create_label(master = frame, text = 'Gripper Name:')
        label.grid(row = 0, column = 0)
        self._plan_pick_gripper_menu = self.create_option_menu(master = frame, values = self._gripper_names)
        self._plan_pick_gripper_menu.grid(row = 0, column = 1)
        button = self.create_button(master = frame, text = 'Plan', command=self.plan_pick_button_callback)
        button.grid(row = 1, column = 0, columnspan = 2)
        notebook.add(frame, text = 'Pick')

        frame = self.create_label_frame(master = notebook)
        frame.pack(fill='both', expand=True)
        label = self.create_label(master = frame, text = 'Hold Name:')
        label.grid(row = 0, column = 0)
        self._plan_place_hold_menu = self.create_option_menu(master = frame, values = self._hold_names)
        self._plan_place_hold_menu.grid(row = 0, column = 1)
        button = self.create_button(master = frame, text = 'Plan', command=self.plan_place_button_callback)
        button.grid(row = 1, column = 0, columnspan = 2)
        notebook.add(frame, text = 'Pick')

        frame = self.create_label_frame(master = notebook)
        frame.pack(fill='both', expand=True)
        label = self.create_label(master = frame, text = 'Gripper Name:')
        label.grid(row = 0, column = 0)
        self._plan_pick_and_place_gripper_menu = self.create_option_menu(master = frame, values = self._gripper_names)
        self._plan_pick_and_place_gripper_menu.grid(row = 0, column = 1)
        label = self.create_label(master = frame, text = 'Hold Name:')
        label.grid(row = 1, column = 0)
        self._plan_pick_and_place_hold_menu = self.create_option_menu(master = frame, values = self._hold_names)
        self._plan_pick_and_place_hold_menu.grid(row = 1, column = 1)
        button = self.create_button(master = frame, text = 'Plan', command=self.plan_pick_and_place_button_callback)
        button.grid(row = 2, column = 0, columnspan = 2)
        notebook.add(frame, text = 'Pick + Place')

        frame = self.create_label_frame(master = notebook)
        frame.pack(fill='both', expand=True)
        label = self.create_label(master = frame, text = 'Gripper Name:')
        label.grid(row = 0, column = 0)
        self._plan_to_configuration_gripper_menu = self.create_option_menu(master = frame, values = self._gripper_names)
        self._plan_to_configuration_gripper_menu.grid(row = 0, column = 1)
        button = self.create_button(master = frame, text = 'Plan', command=self.plan_to_configuration_button_callback)
        button.grid(row = 1, column = 0, columnspan = 2)
        notebook.add(frame, text = 'To Configuration')

        frame = self.create_label_frame(master = notebook)
        frame.pack(fill='both', expand=True)
        self._plan_global_hold_menues: list[TkOptionMenu] = []
        i = 0
        for gripper_name in self._gripper_names:
            label = self.create_label(master = frame, text = 'Gripper ' + str(i) + ' (' + gripper_name + ')')
            label.grid(row = i, column = 0)
            menu = self.create_option_menu(master = frame, values = self._hold_names)
            menu.grid(row = i, column = 1)
            self._plan_global_hold_menues.append(menu)
            i = i + 1
        button = self.create_button(master = frame, text = 'Plan', command=self.plan_global_button_callback)
        button.grid(row = i, column = 0, columnspan = 2)
        notebook.add(frame, text = 'Global')

        frame = self.create_label_frame(master = action_frame)
        frame.configure({"relief":"flat"})
        frame.grid(row=1,column=0)
        button = self.create_cancel_button(master = frame, command = self.plan_action_cancel_button_callback)
        button.grid(row = 0, column = 0, columnspan = 2)
        label = self.create_label(master = frame, text = 'Current: ')
        label.grid(row=1, column=0)
        self._plan_current_label = self.create_string_label(master = frame)
        self._plan_current_label.grid(row=1,column=1)
        self._plan_current_label.configure(width = 25, height = 6)
        self._plan_current_label.bind('<Configure>', lambda e: self._plan_current_label.config(wraplength=self._plan_current_label.winfo_width()-10))
        label = self.create_label(master = frame, text = 'Status:')
        label.grid(row = 2, column = 0)
        self._plan_status_label = self.create_action_status_label(master = frame)
        self._plan_status_label.grid(row = 2, column = 1) 

        self.create_timer(callback = self.timer_callback, timeout_ms = 100)

    def timer_callback(self):

        if not self._busy_node.msg_queue.empty():
            msg: Bool = self._busy_node.msg_queue.get()
            self._busy_label.update_data(msg.data)
        if not self._commands_path_node.msg_queue.empty():
            msg: String = self._commands_path_node.msg_queue.get()
            self._commands_path_label.update_data(msg.data)
        if not self._commands_saved_node.msg_queue.empty():
            msg: Bool = self._commands_saved_node.msg_queue.get()
            self._commands_saved_label.update_data(msg.data)
        if not self._save_commands_node.res_queue.empty():
            response: Trigger.Response = self._save_commands_node.res_queue.get()
            self._save_success_label.update_data(response.success)
        if not self._plan_node.status_queue.empty():
            self._plan_status_label.update_data(self._plan_node.status_queue.get())
        if not self._plan_node.feedback_queue.empty():
            feedback: PlanTetherbot.Feedback = self._plan_node.feedback_queue.get()
            self._plan_current_label.update_data(feedback.message)

    def display_state_button_callback(self):
        
        self._display_state_node.req_queue.put(EmptyService.Request())

    def display_commands_button_callback(self):

        self._display_commands_node.req_queue.put(EmptyService.Request())

    def set_commands_path_button_callback(self):

        request = SetString.Request()
        request.data = self._set_commands_path_entry.get()

        self._set_commands_path_node.req_queue.put(request)

    def save_commands_button_callback(self):
        
        self._save_success_label.update_data(None)
        self._save_commands_node.req_queue.put(Trigger.Request())

    def plan_action_cancel_button_callback(self):

        self._plan_node.cancel_event.set()

    def plan_platform_button_callback(self):

        self.send_to_plan_node(mode = 0,
                               pose = self._plan_platform_pose_entry.get_data())
        
    def plan_arm_button_callback(self):

        self.send_to_plan_node(mode = 1,
                               pose = self._plan_arm_pose_entry.get_data())
        
    def plan_pick_button_callback(self):

        self.send_to_plan_node(mode = 2,
                               gripper_index = self._plan_pick_gripper_menu.get_index())
        
    def plan_place_button_callback(self):

        self.send_to_plan_node(mode = 3,
                               hold_index = self._plan_place_hold_menu.get_index())
        
    def plan_pick_and_place_button_callback(self):

        self.send_to_plan_node(mode = 4,
                               gripper_index = self._plan_pick_and_place_gripper_menu.get_index(),
                               hold_index = self._plan_pick_and_place_hold_menu.get_index())

    def plan_to_configuration_button_callback(self):

        self.send_to_plan_node(mode = 5,
                               gripper_index = self._plan_to_configuration_gripper_menu.get_index())
        
    def plan_global_button_callback(self):

        self.send_to_plan_node(mode = 6,
                               goal_configuration = [menu.get_index() for menu in self._plan_global_hold_menues])

    def send_to_plan_node(self, mode: int, pose: Pose = None, gripper_index: int = 0, hold_index: int = 0, goal_configuration: np.ndarray = None):
        
        if pose is None:
            pose = Pose()
        if goal_configuration is None:
            goal_configuration = [0]
        
        goal = PlanTetherbot.Goal()
        goal.mode = int(mode)
        goal.goal_pose = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        goal.gripper_index = int(gripper_index)
        goal.hold_index = int(hold_index)
        goal.goal_configuration = np.array(goal_configuration).astype(int).tolist()

        self._plan_node.goal_queue.put(goal)
