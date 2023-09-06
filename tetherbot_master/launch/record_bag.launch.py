from __future__ import annotations

import os
import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    topics = ['/platform0/platform_state_publisher/stability', 
              '/platform0/platform_state_publisher/pose', 
              '/platform0/platform_controller/target_pose', 
              '/rpparm0/arm_state_publisher/pose',
              '/rpparm0/arm_state_publisher/target_pose']
    topics = ['/platform0/platform_state_publisher/stability']
    bag_file =  os.path.expanduser('~') + '/ros2_ws/bag/' + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + '/'

    return LaunchDescription([ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_file, topics], output="screen")])