from __future__ import annotations
from typing import TYPE_CHECKING
from .tkinter_objects import TkLabelFrame, TkLabel, TkBoolLabel, TkButton, TkStringLabel, TkEntry, \
    TkCancelButton, TkActionStatusLabel, TkPoseLabelFrame, TkArrayLabel, TkOptionMenu, TkFloatLabel
from .interfaces import BoolMsgInterface, SerialSendSrvInterface, EmptySrvInterface
from .window import Window

if TYPE_CHECKING:
    from .tetherbot_gui import App


class MiscWindow(Window):

    def __init__(self, master: App, title: str='Misc'):
        
        super().__init__(master, title, icon_file='srl_icon_misc.png')

    def create_ui(self):

        serial_frame = TkLabelFrame(master=self, text='Serial Manager')
        serial_frame.grid(row=0, column=0)

        state_frame = TkLabelFrame(master=serial_frame, text='State')
        state_frame.grid(row=0, column=0)

        label = TkLabel(master=state_frame, text="Connected:")
        label.grid(row=0, column=0)
        label = TkBoolLabel(master=state_frame)
        label.grid(row=0, column=1)

        BoolMsgInterface(master=self, msg_name='serial_manager/connected', label=label)

        service_frame = TkLabelFrame(master=serial_frame, text='Services')
        service_frame.grid(row=1, column=0)

        label = TkLabel(master=service_frame, text="Msg:")
        label.grid(row=0, column=0)
        entry = TkEntry(master=service_frame)
        entry.grid(row=0, column=1, columnspan=2)
        button = TkButton(master=service_frame, text='Send')
        button.grid(row=1, column=1, columnspan=2)
        label = TkLabel(master=service_frame, text="Success:")
        label.grid(row=2, column=0)
        success_label = TkBoolLabel(master=service_frame, height=1)
        success_label.grid(row=2, column=1, columnspan=2)

        SerialSendSrvInterface(master=self, srv_name='/serial_manager/send', button=button, entry=entry, success_label=success_label)

        opti_frame = TkLabelFrame(master=self, text='OptiTrack')
        opti_frame.grid(row=1, column=0)

        service_frame = TkLabelFrame(master=opti_frame, text='Services')
        service_frame.grid(row=0, column=0)
        button = TkButton(master=service_frame, text='Tetherbot Pose Calibration')
        button.grid(row=0, column=0)

        EmptySrvInterface(master=self, srv_name='/tetherbot_tbot_calibration/tetherbot_coordinate_calibration_service', button=button)