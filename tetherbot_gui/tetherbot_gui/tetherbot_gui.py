from __future__ import annotations
import rclpy
import rclpy.logging
import rclpy.subscription
import rclpy.executors
import rclpy.impl.rcutils_logger
import tkinter as tk
import os
import signal
import subprocess
from threading import Thread, Event
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from tbotlib import TbTetherbot
from .window import Window
from .can_network_window import CanNetworkWindow
from .servo_window import ServoWindow
from .misc_window import MiscWindow
from .arm_window import ArmWindow
from .gripper_window import GripperWindow
from .platform_window import PlatformWindow
from .sequencer_window import SequencerWindow
from .planner_window import PlannerWindow

class App(tk.Tk):

    def __init__(self):
        super().__init__()

        # create node representing the gui
        self.node = Node('tetherbot_gui')

        # create menubar
        menubar = tk.Menu(self)
        self.config(menu=menubar)

        # icon for application and all other windows
        self.iconphoto(True, tk.PhotoImage(file=os.path.join(get_package_share_directory('tetherbot_gui'), 'icons/srl_icon.png')))

        # title
        self.title('Tetherbot UI 2.0')

        # create file menu
        file_menu = tk.Menu(menubar)
        file_menu.add_command(label='Exit', command=self.destroy)

        # create window menu
        window_menu = tk.Menu(menubar)
        window_menu.add_command(label='Can Network', command=lambda: self.open_window_callback(CanNetworkWindow))
        window_menu.add_command(label='Wireless Servo', command=lambda: self.open_window_callback(ServoWindow))
        window_menu.add_command(label='Misc', command=lambda: self.open_window_callback(MiscWindow))
        window_menu.add_separator()
        window_menu.add_command(label='Arm', command=lambda: self.open_window_callback(ArmWindow))
        window_menu.add_command(label='Gripper', command=lambda: self.open_window_callback(GripperWindow))
        window_menu.add_command(label='Platform', command=lambda: self.open_window_callback(PlatformWindow))
        window_menu.add_separator()
        window_menu.add_command(label='Planner', command=lambda: self.open_window_callback(PlannerWindow))
        window_menu.add_command(label='Sequencer', command=lambda: self.open_window_callback(SequencerWindow))
        window_menu.add_separator()
        window_menu.add_command(label='Preset 1 (all)', command=lambda: self.open_windows_callback([CanNetworkWindow, ServoWindow, MiscWindow, ArmWindow, GripperWindow, PlatformWindow, PlannerWindow, SequencerWindow]))
        window_menu.add_command(label='Preset 2 (hardware)', command=lambda: self.open_windows_callback([CanNetworkWindow, ServoWindow, MiscWindow]))
        window_menu.add_separator()
        window_menu.add_command(label='Close Windows', command=self.close_windows_callback)

        # create tool menu
        tool_menu = tk.Menu(menubar)
        tool_menu.add_command(label='Rviz', command=lambda: subprocess.run(['rviz2']))
        tool_menu.add_command(label='RQT', command=lambda: subprocess.run(['rqt']))
        tool_menu.add_command(label='RQT Graph', command=lambda: subprocess.run(['rqt_graph']))
        tool_menu.add_command(label='SSH', command=lambda: subprocess.Popen(["gnome-terminal --title=srl-orin -- sh -c 'cd ~; ssh srl-orin@192.168.1.2; bash'"], shell=True))

        # add menus to menubar
        menubar.add_cascade(label='File', menu=file_menu)
        menubar.add_cascade(label='Window', menu=window_menu)
        menubar.add_cascade(label='Tools', menu=tool_menu)

        # list to store all windows
        self.windows: list[Window] = []

        # load tetherbot object from config path
        tbot_desc_file = os.path.join(get_package_share_directory('tbotros_description'), 'desc/tetherbot_light.pkl')
        try:
            self.tbot: TbTetherbot = TbTetherbot.load(tbot_desc_file)
        except Exception as exc:
            self.get_logger().error('Failed loading tetherbot object: ' + str(exc))

        # add ctrl+C support
        signal.signal(signal.SIGINT, self.destroy)
        #self.bind_all('<Control-c>', self.destroy)
        self.check() # causes the terminal to be sampled for events in regular intervals

    def check(self): 

        self.after(500, self.check) 

    def open_window_callback(self, window_cls: type[Window]):

        for window in self.windows:
            if type(window) is window_cls:
                self.bring_window_to_front(window)
                return

        self.add_window(window_cls)

    def open_windows_callback(self, window_clss: list[type[Window]]):

        for window_cls in window_clss:
            self.open_window_callback(window_cls)

    def close_windows_callback(self):

        for window in reversed(self.windows):
            window.destroy()

    def add_window(self, window_cls: type[Window]) -> Window:

        window = window_cls(self)
        self.windows.append(window)

        return window

    def remove_window(self, window: Window):

        self.windows.remove(window)

    def bring_window_to_front(self, window: Window):

        window.lift()
        window.focus_set()

    def get_logger(self) -> rclpy.impl.rcutils_logger.RcutilsLogger:

        return self.node.get_logger()
    
    def destroy(self, *_):
        
        self.node.destroy_node()

        super().destroy()

    def mainloop(self) -> None:

        stop_event = Event()
        spin_thread = Thread(target=self.ros_spin_thread, args=(self.node, stop_event))
        spin_thread.start()

        super().mainloop()

        stop_event.set()
        spin_thread.join()

    def ros_spin_thread(self, node: Node, stop_event: Event):

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while stop_event.is_set() == False:
                executor.spin_once(timeout_sec=0.5)
        except rclpy.executors.ExternalShutdownException:
            print('hi')
            pass

        executor.shutdown()


def main():

    rclpy.init()

    app = App()
    app.mainloop()
    
    rclpy.shutdown()

if __name__ == '__main__':

    main()
