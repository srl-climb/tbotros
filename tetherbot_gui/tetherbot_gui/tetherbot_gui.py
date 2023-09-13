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
from rclpy_wrapper.node import Node2
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
        self.node = Node2('tetherbot_gui')

        self.node.declare_parameter('desc_file', '/home/climb/ros2_ws/src/tbotros_description/tbotros_description/desc/tetherbot/tetherbot_light.pkl')
        self.desc_file = self.node.get_parameter('desc_file').get_parameter_value().string_value

        # create menubar
        menubar = tk.Menu(self)
        self.config(menu=menubar)

        # icon for application and all other windows
        self.iconphoto(True, tk.PhotoImage(file=os.path.join(get_package_share_directory('tetherbot_gui'), 'icons/srl_icon.png')))

        # title
        self.title('Tetherbot UI 2.0')

        # background image
        image = tk.PhotoImage(file=os.path.join(get_package_share_directory('tetherbot_gui'), 'icons/srl_logo.png')).subsample(2)
        label = tk.Label(image=image)
        label.pack()
        label.img = image

        # create file menu
        file_menu = tk.Menu(menubar)
        file_menu.add_command(label='Exit', command=self.destroy)
        file_menu.add_command(label='Exit All', command=lambda: [self.destroy_subprocesses(), self.destroy()])

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
        tool_menu.add_command(label='Rviz', command=lambda:self.create_subprocess('rviz2 -d ' + os.path.join(get_package_share_directory('tetherbot_gui'), 'config/tbot.rviz')))
        tool_menu.add_command(label='Rqt', command=lambda: self.create_subprocess('rqt'))
        tool_menu.add_command(label='Rqt Graph', command=lambda: self.create_subprocess('rqt_graph'))

        # create tool menu
        jetson_menu = tk.Menu(menubar)
        jetson_menu.add_command(label='Launch', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/jetson_launch.sh')))
        jetson_menu.add_command(label='Terminal', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/jetson_terminal.sh')))
        jetson_menu.add_command(label='Setup', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/jetson_setup.sh')))
        jetson_menu.add_command(label='Monitor', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/jetson_monitor.sh')))
        jetson_menu.add_command(label='Shutdown', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/jetson_shutdown.sh')))
        jetson_menu.add_command(label='Reboot', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/jetson_reboot.sh')))

        # create ground station menu
        gs_menu = tk.Menu(menubar)
        gs_menu.add_command(label='Launch', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/gs_launch.sh')))

        # create ros bag menu
        bag_menu = tk.Menu(menubar)
        bag_menu.add_command(label='Record', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/bag_record_launch.sh')))
        bag_menu.add_command(label='Play', command=lambda: self.create_subprocess("sh " + os.path.join(get_package_share_directory('tetherbot_gui'), 'bash/bag_play_launch.sh')))

        # add menus to menubar
        menubar.add_cascade(label='File', menu=file_menu)
        menubar.add_cascade(label='Window', menu=window_menu)
        menubar.add_cascade(label='Tools', menu=tool_menu)
        menubar.add_cascade(label='Jetson', menu=jetson_menu)
        menubar.add_cascade(label='Ground Station', menu=gs_menu)
        menubar.add_cascade(label='ROS Bag', menu=bag_menu)

        # list to store all windows
        self.windows: list[Window] = []

        # list to store all subprocesses
        self.processes: list[subprocess.Popen] = []

        # load tetherbot object from config path
        try:
            self.tbot: TbTetherbot = TbTetherbot.load(self.desc_file)
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
    
    def create_subprocess(self, args: str):

        self.processes.append(subprocess.Popen(args, shell=True, preexec_fn=os.setpgrp))

    def destroy_subprocesses(self):

        for process in reversed(self.processes):
            os.killpg(process.pid, signal.SIGTERM)
            self.processes.remove(process)
    
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

    def ros_spin_thread(self, node: Node2, stop_event: Event):

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while stop_event.is_set() == False:
                executor.spin_once(timeout_sec=0.5)
        except rclpy.executors.ExternalShutdownException:
            pass

        executor.shutdown()


def main():

    rclpy.init()

    app = App()
    app.mainloop()
    
    rclpy.shutdown()

if __name__ == '__main__':

    main()
