from __future__ import annotations
import tkinter as tk
from std_msgs.msg import Bool
from custom_srvs.srv import SerialSend
from .window import Window
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App
    

class SerialManagerWindow(Window):

    def __init__(self, parent: App, title: str = 'Serial Manager Window'):
        
        super().__init__(parent, title)

    def create_user_interface(self):

        self.connected_node = self.create_subscriber_node(msg_name = 'serial_manager/connected',
                                                          msg_type = Bool)


        self.send_node = self.create_client_node(srv_name = '/serial_manager/send',
                                                 srv_type = SerialSend)

        labelframe = self.create_label_frame(master = self, text = 'State')
        labelframe.pack()

        label = self.create_label(master = labelframe, text = "Connected:")
        label.pack(side=tk.LEFT)

        self.connected_bool_label = self.create_bool_label(master = labelframe, height=1)
        self.connected_bool_label.pack(side=tk.LEFT)

        labelframe = self.create_label_frame(master = self, text = 'Send serial')
        labelframe.pack(expand=True)

        label = self.create_label(master = labelframe, text = "Msg:")
        label.grid(row=0, column=0)

        self.send_entry = self.create_entry(master = labelframe)
        self.send_entry.grid(row=0, column=1, columnspan=2)

        button = self.create_button(master = labelframe, text='Send', command = self.serial_send_button_callback)
        button.grid(row=1, column=1, columnspan=2)
        

        label = self.create_label(master = labelframe, text = "Success:")
        label.grid(row=2, column=0)

        self.success_bool_label = self.create_bool_label(master = labelframe, height=1)
        self.success_bool_label.grid(row=2, column=1, columnspan=2)

        self.create_timer(callback = self.timer_callback, timeout_ms = 100)

    def timer_callback(self):

        if not self.connected_node.msg_queue.empty():
            msg: Bool = self.connected_node.msg_queue.get()

            self.connected_bool_label.update_data(msg.data)

        if not self.send_node.res_queue.empty():
            res: SerialSend.Response = self.send_node.res_queue.get()

            self.success_bool_label.update_data(res.success)

    def serial_send_button_callback(self):
        
        req = SerialSend.Request()
        req.msg = self.send_entry.get()

        self.send_node.req_queue.put(req)

        self.success_bool_label.update_data(None)    