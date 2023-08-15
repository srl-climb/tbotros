from __future__ import annotations
import rclpy
import rclpy.logging
import rclpy.subscription
import rclpy.executors
import rclpy.impl.rcutils_logger
import tkinter as tk
from threading import Thread, Event
from rclpy.node import Node
from .interface_nodes import SubscriptionNode, ClientNode, ActionClientNode
from .tkinter_objects import TkBoolLabel, TkEntry, TkFloatEntry, TkLabel, TkTimer, \
    TkLabelFrame, TkButton, TkStringLabel, TkErrorCodeLabel, TkCancelButton, \
    TkActionStatusLabel, TkCanvas, TkScrollbar, TkScrollFrame, TkFrame, TkPoseLabelFrame, \
    TkOptionMenu, TkVectorLabel, TkPoseEntryFrame, TkFloatLabel
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App


class Window(tk.Toplevel):

    def __init__(self, parent: App, title: str = 'Window'):

        super().__init__(parent)

        self.parent = parent
        self.title(title)
        
        self.buttons: list[TkButton] = []
        self.entries: list[TkEntry] = []
        self.labels: list[TkLabel] = []
        self.frames: list[tk.Frame] = []
        self.labelframes: list[TkLabelFrame] = []
        self.optionmenues: list[TkOptionMenu] = []
        self.nodes: list[Node] = []
        self.timers: list[TkTimer] = []

        self.node_thread: Thread = None
        self.node_thread_stop_event = Event() 

        self.create_user_interface()

        self.spin_nodes()

    def destroy(self) -> None:
        """
        Destroy the window.
        """
        self.parent.remove_window(self)
        self.stop_nodes()
        
        return super().destroy()
    
    def get_data(self) -> dict:
        """
        Get the data of the window.
        Returns:
            dict: The window data.
        """
        data = {}
        data['title'] = self.title()
        data['geometry'] = self.geometry()
        data['entry_strings'] = []

        for entry in self.entries:
            data['entry_strings'].append(entry.get())

        return data

    def set_data(self, data: dict):
        """
        Set the data of the window.
        Args:
            data (dict): The window data.
        """
        try:
            self.title(data['title'])
            self.geometry(data['geometry'])

            for entry, entry_string in zip(self.entries, data['entry_strings']):
                entry.delete(0, tk.END)
                entry.insert(0, entry_string)

        except Exception as exc:
            self.get_logger().error('Error during deserialization: ' + str(exc))

    def create_user_interface(self):
        """
        Create the user interface for the window.
        """
        pass

    def create_timer(self, **args):
        """
        Create a timer for the window.
        Args:
            **args: Additional arguments for the timer.
        Returns:
            TkTimer: The created timer.
        """
        self.timers.append(TkTimer(self, **args))
        self.timers[-1].run()

        return self.timers[-1]

    def create_button(self, **args) -> TkButton:
        """
        Create a button for the window.
        Args:
            **args: Additional arguments for the button.
        Returns:
            TkButton: The created button.
        """
        self.buttons.append(TkButton(**args))
        return self.buttons[-1]
    
    def create_cancel_button(self, **args) -> TkCancelButton:

        self.buttons.append(TkCancelButton(**args))
        return self.buttons[-1]
    
    def create_entry(self, **args) -> TkEntry:
        """
        Create an entry widget for the window.
        Args:
            **args: Additional arguments for the entry widget.
        Returns:
            TkEntry: The created entry widget.
        """
        self.entries.append(TkEntry(**args))
        return self.entries[-1]
    
    def create_float_entry(self, **args) -> TkFloatEntry:
        """
        Create an entry widget for the window.
        Args:
            **args: Additional arguments for the entry widget.
        Returns:
            TkFloatEntry: The created entry widget.
        """
        self.entries.append(TkFloatEntry(**args))
        return self.entries[-1]
    
    def create_label(self, **args) -> TkLabel:
        """
        Create a label for the window.
        Args:
            **args: Additional arguments for the label.
        Returns:
            TkLabel: The created label.
        """
        self.labels.append(TkLabel(**args))
        return self.labels[-1]
    
    def create_bool_label(self, **args) -> TkBoolLabel:

        self.labels.append(TkBoolLabel(**args))
        return self.labels[-1]
    
    def create_string_label(self, **args) -> TkStringLabel:

        self.labels.append(TkStringLabel(**args))
        return self.labels[-1]
    
    def create_float_label(self, **args) -> TkFloatLabel:

        self.labels.append(TkFloatLabel(**args))
        return self.labels[-1]
    
    def create_error_code_label(self, **args) -> TkErrorCodeLabel:

        self.labels.append(TkErrorCodeLabel(**args))
        return self.labels[-1]
    
    def create_action_status_label(self, **args) -> TkActionStatusLabel:

        self.labels.append(TkActionStatusLabel(**args))
        return self.labels[-1]
    
    def create_vector_label(self, **args) -> TkVectorLabel:

        self.labels.append(TkVectorLabel(**args))
        return self.labels[-1]
    
    def create_frame(self, **args) -> TkFrame:

        self.frames.append(TkFrame(**args))
        return self.frames[-1]
    
    def create_scroll_frame(self, **args) -> TkScrollFrame:

        self.frames.append(TkScrollFrame(**args))
        return self.frames[-1]
    
    def create_label_frame(self, **args) -> TkLabelFrame:

        self.labelframes.append(TkLabelFrame(**args))
        return self.labelframes[-1]

    def create_pose_label_frame(self, **args) -> TkPoseLabelFrame:

        self.labelframes.append(TkPoseLabelFrame(**args))
        return self.labelframes[-1]
    
    def create_pose_entry_frame(self, **args) -> TkPoseEntryFrame:

        self.labelframes.append(TkPoseEntryFrame(**args))
        return self.labelframes[-1]
    
    def create_option_menu(self, **args) -> TkOptionMenu:

        self.optionmenues.append(TkOptionMenu(**args))
        return self.optionmenues[-1]
    
    def create_subscriber_node(self, **args) -> SubscriptionNode:
        """
        Create a subscriber node for the window.
        Args:
            **args: Additional arguments for the subscriber node.
        Returns:
            SubscriptionNode: The created subscriber node.
        """
        self.nodes.append(SubscriptionNode(**args))

        return self.nodes[-1]

    def create_client_node(self, **args) -> ClientNode:
        """
        Create a client node for the window.
        Args:
            **args: Additional arguments for the client node.
        Returns:
            ClientNode: The created client node.
        """
        self.nodes.append(ClientNode(**args))
        return self.nodes[-1]
    
    def create_action_client_node(self, **args) -> ActionClientNode:

        self.nodes.append(ActionClientNode(**args))
        return self.nodes[-1]
    
    def spin_nodes(self):
        """
        Start spinning the nodes in a seperate thread.
        """
        if self.nodes:
            self.node_thread = Thread(target=self.node_executor, args=(self.nodes, self.node_thread_stop_event))
            self.node_thread.start()

    def stop_nodes(self):
        """
        Stop the spinning of nodes by calling the stop event.
        """
        if self.node_thread is not None:
            self.node_thread_stop_event.set() 
            self.node_thread.join(2)

    @staticmethod
    def node_executor(nodes, stop_event: Event):
        """
        Execute the nodes in the window.
        Args:
            nodes: The nodes to execute.
            stop_event (Event): The event to stop node execution.
        """
        executor = rclpy.executors.MultiThreadedExecutor()

        for node in nodes:
            executor.add_node(node)

        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)  # execute one callback

    def get_logger(self) -> rclpy.impl.rcutils_logger.RcutilsLogger:
        """
        Get the logger for the window.
        Returns:
            rclpy.impl.rcutils_logger.RcutilsLogger: The logger.
        """
        return rclpy.logging.get_logger(self.title)


