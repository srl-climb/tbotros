from __future__ import annotations
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from custom_srvs.srv import SetString
from custom_actions.action import Empty as ExecuteSequence
from .window import Window
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App

    
class SequencerWindow(Window):

    def __init__(self, parent: App, title: str = 'Squencer Window'):

        super().__init__(parent, title)

    def create_user_interface(self):

        self._busy_node = self.create_subscriber_node(msg_type=Bool, msg_name='sequencer/busy')
        self._commands_path_node = self.create_subscriber_node(msg_type=String, msg_name='sequencer/commands_path')
        self._commands_loaded_node = self.create_subscriber_node(msg_type=Bool, msg_name='sequencer/commands_loaded')
        self._set_commands_path_node = self.create_client_node(srv_type=SetString, srv_name='sequencer/set_commands_path', enable_response = False)
        self._load_commands_node = self.create_client_node(srv_type=Trigger, srv_name='sequencer/load_commands')
        self._execute_sequence_node = self.create_action_client_node(action_type=ExecuteSequence, action_name='sequencer/execute_sequence', enable_feedback = True)

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
        label = self.create_label(master=state_frame, text='Commands Loaded:')
        label.grid(row=2, column=0)
        self._commands_loaded_label = self.create_bool_label(master=state_frame)
        self._commands_loaded_label.grid(row=2, column=1)

        service_frame = self.create_label_frame(master = self, text = 'Services')
        service_frame.grid(row = 1, column = 0)

        label = self.create_label(master = service_frame, text = 'File Path:')
        label.grid(row = 0, column = 0)
        self._set_commands_path_entry = self.create_entry(master = service_frame)
        self._set_commands_path_entry.grid(row = 0, column = 1)
        button = self.create_button(master= service_frame, text = 'Set Commands Path', command=self.set_commands_path_button_callback)
        button.grid(row = 1, column = 0, columnspan = 2)
        button = self.create_button(master = service_frame, text = 'Load Commands', command=self.load_commands_button_callback)
        button.grid(row = 2, column = 0, columnspan = 2)
        label = self.create_label(master = service_frame, text = 'Success: ')
        label.grid(row = 3, column = 0)
        self._load_success_label = self.create_bool_label(master = service_frame)
        self._load_success_label.grid(row = 3, column = 1)

        action_frame = self.create_label_frame(master = self, text = 'Actions')
        action_frame.grid(row = 2, column = 0)
        button = self.create_button(master = action_frame, text = 'Execute Commands', comman = self.execute_commands_button_callback)
        button.grid(row = 0, column = 0, columnspan = 2)
        button = self.create_cancel_button(master = action_frame, command = self.execute_commands_cancel_button_callback)
        button.grid(row = 1, column = 0, columnspan = 2)
        label = self.create_label(master = action_frame, text = 'Progress: ')
        label.grid(row = 2, column = 0)
        self._progress_label = self.create_string_label(master = action_frame)
        self._progress_label.grid(row = 2, column = 1)
        label = self.create_label(master = action_frame, text = 'Status:')
        label.grid(row = 3, column = 0)
        self._execute_status_label = self.create_action_status_label(master = action_frame)
        self._execute_status_label.grid(row = 3, column = 1)

        self.create_timer(callback = self.timer_callback, timeout_ms = 100)

    def timer_callback(self):

        if not self._busy_node.msg_queue.empty():
            msg: Bool = self._busy_node.msg_queue.get()
            self._busy_label.update_data(msg.data)
        if not self._commands_path_node.msg_queue.empty():
            msg: String = self._commands_path_node.msg_queue.get()
            self._commands_path_label.update_data(msg.data)
        if not self._commands_loaded_node.msg_queue.empty():
            msg: Bool = self._commands_loaded_node.msg_queue.get()
            self._commands_loaded_label.update_data(msg.data)
        if not self._load_commands_node.res_queue.empty():
            response: Trigger.Response = self._load_commands_node.res_queue.get()
            self._load_success_label.update_data(response.success)
        if not self._execute_sequence_node.feedback_queue.empty():
            feedback: ExecuteSequence.Feedback = self._execute_sequence_node.feedback_queue.get()
            self._progress_label.update_data(str(feedback.current) + ' of ' + str(feedback.length))
        if not self._execute_sequence_node.status_queue.empty():
            self._execute_status_label.update_data(self._execute_sequence_node.status_queue.get())

    def set_commands_path_button_callback(self):

        request = SetString.Request()
        request.data = self._set_commands_path_entry.get()

        self._set_commands_path_node.req_queue.put(request)
    
    def load_commands_button_callback(self):

        self._load_success_label.update_data(None)
        self._load_commands_node.req_queue.put(Trigger.Request())

    def execute_commands_button_callback(self):

        self._execute_sequence_node.goal_queue.put(ExecuteSequence.Goal())

    def execute_commands_cancel_button_callback(self):

        self._execute_sequence_node.cancel_event.set()
