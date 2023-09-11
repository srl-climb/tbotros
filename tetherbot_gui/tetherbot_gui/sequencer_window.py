from __future__ import annotations
from typing import TYPE_CHECKING
from .tkinter_objects import TkLabelFrame, TkLabel, TkBoolLabel, TkStringLabel, TkButton, TkEntry, TkCancelButton, TkActionStatusLabel
from .interfaces import BoolMsgInterface, StringMsgInterface, SetStringSrvInterface, TriggerSrvInterface, ExecuteSequenceActionInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App


class SequencerWindow(Window):

    def __init__(self, master: App, title: str = 'Sequencer'):
        
        super().__init__(master, title, icon_file='srl_icon_seq.png')

    def create_ui(self):

        state_frame = TkLabelFrame(master=self, text='State')
        state_frame.grid(row=0, column=0)

        label = TkLabel(master=state_frame, text='Busy:')
        label.grid(row=0,column=0)
        label = TkBoolLabel(master=state_frame)
        label.grid(row=0,column=1)

        BoolMsgInterface(master=self, label=label, msg_name='sequencer/busy')

        label = TkLabel(master=state_frame, text='Commands File:')
        label.grid(row=1, column=0)
        label = TkStringLabel(master=state_frame)
        label.grid(row=1, column=1)
        label.configure(width = 25)
        label.bind('<Configure>', lambda e: label.config(wraplength=label.winfo_width()-10))

        StringMsgInterface(master=self, label=label, msg_name='sequencer/commands_file')

        label = TkLabel(master=state_frame, text='Commands loaded:')
        label.grid(row=2,column=0)
        label = TkBoolLabel(master=state_frame)
        label.grid(row=2,column=1)

        BoolMsgInterface(master=self, label=label, msg_name='sequencer/commands_loaded')
        
        service_frame = TkLabelFrame(master = self, text = 'Services')
        service_frame.grid(row = 1, column = 0)

        label = TkLabel(master = service_frame, text = 'File Path:')
        label.grid(row = 0, column = 0)
        entry = TkEntry(master = service_frame)
        entry.grid(row = 0, column = 1)
        button = TkButton(master= service_frame, text = 'Set Commands File')
        button.grid(row = 1, column = 0, columnspan = 2)

        SetStringSrvInterface(master=self, entry=entry, button=button, srv_name='sequencer/set_commands_file')

        button = TkButton(master = service_frame, text = 'Load Commands')
        button.grid(row = 2, column = 0, columnspan = 2)
        label = TkLabel(master = service_frame, text = 'Success: ')
        label.grid(row = 3, column = 0)
        label = TkBoolLabel(master = service_frame)
        label.grid(row = 3, column = 1)

        TriggerSrvInterface(master=self, success_label=label, button=button, srv_name='sequencer/load_commands')

        action_frame = TkLabelFrame(master = self, text = 'Actions')
        action_frame.grid(row = 2, column = 0)

        execute_button = TkButton(master = action_frame, text = 'Execute Commands')
        execute_button.grid(row = 0, column = 0, columnspan = 2)
        cancel_button = TkCancelButton(master = action_frame)
        cancel_button.grid(row = 1, column = 0, columnspan = 2)
        label = TkLabel(master = action_frame, text = 'Progress: ')
        label.grid(row = 2, column = 0)
        progress_label = TkStringLabel(master = action_frame)
        progress_label.grid(row = 2, column = 1)
        label = TkLabel(master = action_frame, text = 'Message: ')
        label.grid(row = 3, column = 0)
        message_label = TkStringLabel(master = action_frame)
        message_label.grid(row = 3, column = 1)
        label = TkLabel(master = action_frame, text = 'Status:')
        label.grid(row = 4, column = 0)
        status_label = TkActionStatusLabel(master = action_frame)
        status_label.grid(row = 4, column = 1)

        ExecuteSequenceActionInterface(master=self, execute_button=execute_button, cancel_button=cancel_button, 
                                       status_label=status_label, progress_label=progress_label, message_label=message_label, 
                                       action_name='sequencer/execute_sequence')

