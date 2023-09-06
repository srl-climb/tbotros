from __future__ import annotations

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def latest_subdirectory(directory: str) -> str:

    subdirectories = []
    for item in os.listdir(directory):
        item = os.path.join(directory,item)
        if os.path.isdir(item):
            subdirectories.append(item)

    return max(subdirectories, key=os.path.getmtime)

def generate_launch_description():

    bag_dir = latest_subdirectory(os.path.expanduser('~') + '/ros2_ws/bag/')

    return LaunchDescription([ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_dir], output="screen")])