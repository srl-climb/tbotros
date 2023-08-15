from __future__ import annotations
import rclpy
import rclpy.logging
import rclpy.subscription
import rclpy.executors
import rclpy.impl.rcutils_logger
import tkinter as tk
import yaml
import os
import sys
import signal
from tkinter import filedialog
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from tbotlib import TbTetherbot
from .window import Window
from .planner_window import PlannerWindow
from .arm_window import ArmWindow
from .planner_window import PlannerWindow
from .platform_window import PlatformWindow
from .wireless_servo_window import WirelessServoWindow
from .aruco_detector_window import ArucoDetectorWindow
from .serial_manager_window import SerialManagerWindow
from .can_network_motor_controller_window import CanNetworkMotorControllerWindow
from .gripper_window import GripperWindow
from .sequencer_window import SequencerWindow

sys.path.append(os.path.join(get_package_prefix('tetherbot_gui'), 'lib/python3.8/site-packages/tetherbot_gui'))
# NOTE: tbotlib's load function is based on pickle, which has to import tbotlib in order to function
#       we add the path to the tbotlib inside the tetherbot install to make tbotlib importable

class App(tk.Tk):
    # A class representing an application

    def __init__(self):
        super().__init__()
        # Initialize the superclass tk.Tk

        # Create menubar
        menubar = tk.Menu(self)
        self.config(menu=menubar)

        # Icon for application and all other windows
        self.iconphoto(True, tk.PhotoImage(file=os.path.join(get_package_share_directory('tetherbot_gui'), 'icons/srl_icon.png')))

        # Create file menu
        file_menu = tk.Menu(menubar)
        file_menu.add_command(label='Save as...', command=self.save_as_callback)
        file_menu.add_command(label='Save', command=self.save_callback)
        file_menu.add_command(label='Load', command=self.load_callback)
        file_menu.add_separator()
        file_menu.add_command(label='Exit', command=self.destroy)

        # Create tool menu
        tool_menu = tk.Menu(menubar)
        tool_menu.add_command(label='Serial Manager', command=lambda: self.open_window_callback(SerialManagerWindow))
        tool_menu.add_command(label='CAN Network Motor Controller', command=lambda: self.open_window_callback(CanNetworkMotorControllerWindow))
        tool_menu.add_command(label='Wireless Servo', command=lambda: self.open_window_callback(WirelessServoWindow))
        tool_menu.add_command(label='Aruco Detector', command=lambda: self.open_window_callback(ArucoDetectorWindow))
        tool_menu.add_command(label='Gripper', command=lambda: self.open_window_callback(GripperWindow))
        tool_menu.add_command(label='Arm', command=lambda: self.open_window_callback(ArmWindow))
        tool_menu.add_command(label='Platform', command=lambda: self.open_window_callback(PlatformWindow))
        tool_menu.add_command(label='Planner', command=lambda: self.open_window_callback(PlannerWindow))
        tool_menu.add_command(label='Sequencer', command=lambda: self.open_window_callback(SequencerWindow))
        tool_menu.add_separator()
        tool_menu.add_command(label='Close Windows', command=self.close_windows_callback)

        # Add menus to menubar
        menubar.add_cascade(label='File', menu=file_menu)
        menubar.add_cascade(label='Tools', menu=tool_menu)

        # Initialize attributes
        self.windows: list[Window] = []
        self.save_file = '*'
        
        # load tetherbot object from config path
        tbot_config_path_param = rclpy.Parameter('config_path', value = '/home/climb/ros2_ws/config/tetherbot_light.pkl')
        tbot_config_path = tbot_config_path_param.get_parameter_value().string_value
        try:
            self.tbot: TbTetherbot = TbTetherbot.load(tbot_config_path)
        except Exception as exc:
            self.get_logger().error('Failed loading tetherbot object: ' + str(exc))

        # add ctrlC support
        signal.signal(signal.SIGINT, self.destroy)
        #self.bind_all('<Control-c>', self.destroy)
        self.check() # causes the terminal to be sampled for events in regular intervals

    def check(self): 
        self.after(500, self.check) 

    @property
    def save_file(self) -> str:
        # Getter for the save_file property

        return self._save_file

    @save_file.setter
    def save_file(self, value: str):
        # Setter for the save_file property

        self._save_file = value
        self.set_title(value)

    def destroy(self, *_):

        super().destroy()

    def save_callback(self):
        # Callback function for the save command

        if self.save_file == '*':
            self.save_as_callback()
            # If save_file is '*', invoke the save_as_callback function

        save_data = {'windows': []}

        for window in self.windows:
            save_data['windows'].append({})
            save_data['windows'][-1]['data'] = window.get_data()
            save_data['windows'][-1]['type'] = window.__class__.__name__
        # Create a dictionary to store save data for each window

        with open(self.save_file, 'w+') as stream:
            try:
                yaml.dump(save_data, stream)
            except Exception as exc:
                self.get_logger().error('Error while loading project file :' + exc)
        # Dump the save data to a YAML file

    def save_as_callback(self):
        # Callback function for the save as command

        if self.save_file == '*':
            save_file = '/home/srl-orin/app.yaml'
        else:
            save_file = self.save_file

        save_file = filedialog.asksaveasfilename(confirmoverwrite=True, defaultextension='yaml',
                                                 initialdir=os.path.dirname(save_file),
                                                 initialfile=os.path.basename(save_file))
        # Open a save file dialog to choose a file

        if save_file != ():
            self.save_file = save_file
            self.save_callback()
            # If a file is chosen, update the save_file attribute and invoke the save_callback function

    def load_callback(self):
        # Callback function for the load command

        if self.save_file == '*':
            save_file = '/home/srl-orin/app.yaml'
        else:
            save_file = self.save_file

        save_file = filedialog.askopenfilename(defaultextension='yaml', initialdir=os.path.dirname(save_file),
                                               initialfile=os.path.basename(save_file))
        # Open a file dialog to choose a file to load

        with open(self.save_file, 'r') as stream:
            try:
                save_data = yaml.safe_load(stream)
            except Exception as exc:
                self.get_logger().error('Error while loading project file :' + exc)
            else:
                self.save_file = save_file

                self.close_windows_callback()

                for window_data in save_data['windows']:
                    window = self.add_window(window_data['type'])
                    window.set_data(window_data['data'])
        # Load the save data and create windows based on the data

    def open_window_callback(self, string: str):
        # Callback function for opening a window

        for window in self.windows:
            if type(window) == getattr(sys.modules[__name__], string):
                self.bring_window_to_front(window)
                return

        self.add_window(string)
        # Check if the window is already open, if not, add a new window

    def close_windows_callback(self):
        # Callback function for closing all windows

        for window in reversed(self.windows):
            window.destroy()
        # Destroy all windows in reverse order

    def add_window(self, window_cls: type[Window]) -> Window:
        # Add a new window

        window = window_cls(self)
        self.windows.append(window)

        return window

    def remove_window(self, window: Window):
        # Remove a window from the window references in the App class

        self.windows.remove(window)

    def bring_window_to_front(self, window: Window):
        # Bring a window to the front

        window.lift()
        window.focus_set()

    def set_title(self, prefix: str):
        # Set the application title

        self.title(prefix + ' - Tetherbot UI')

    def get_logger(self) -> rclpy.impl.rcutils_logger.RcutilsLogger:
        # Get the application logger

        return rclpy.logging.get_logger('tetherbot_gui')


def main():

    rclpy.init()

    try:
        app = App()
        app.mainloop()
    except:
        pass

    rclpy.shutdown()

if __name__ == '__main__':

    main()
