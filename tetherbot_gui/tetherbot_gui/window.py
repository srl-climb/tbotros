from __future__ import annotations
import os
import rclpy
import rclpy.logging
import rclpy.subscription
import rclpy.executors
import rclpy.impl.rcutils_logger
import tkinter as tk
from rclpy_wrapper.node import Node2
from typing import TYPE_CHECKING
from ament_index_python.packages import get_package_share_directory
from .interface_queue import InterfaceQueue
from .interfaces import ROSInterface

if TYPE_CHECKING:
    from .tetherbot_gui import App


class Window(tk.Toplevel):

    def __init__(self, master: App, title: str = 'Window', icon_file: str = None):

        self.master: App = None # for correct type hinting
        self.queue = InterfaceQueue(2000)
        self.interfaces: list[ROSInterface] = []

        super().__init__(master)
    
        self.title(title)

        if icon_file is not None:
            self.iconphoto(False, tk.PhotoImage(file=os.path.join(get_package_share_directory('tetherbot_gui'), 'icons', icon_file)))

        self.create_ui()
        self.update_loop()

    @property
    def node(self) -> Node2:

        return self.master.node
    
    def create_ui(self):

        pass

    def destroy(self) -> None:

        self.master.remove_window(self)

        for interface in self.interfaces:
            interface.destroy()
        
        return super().destroy()
    
    def update_loop(self):
        
        if self.queue.full():
            self.get_logger().warn('Window update queue full', throttle_duration_sec=1)

        while not self.queue.empty():
            label, data = self.queue.get()
            label.update_data(data)

        self.after(25, self.update_loop) 

    def get_logger(self) -> rclpy.impl.rcutils_logger.RcutilsLogger:

        return self.master.get_logger()


