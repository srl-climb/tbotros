from __future__ import annotations

import os
from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription, Shutdown
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# LINKS:
# https://answers.ros.org/question/374926/ros2-how-to-launch-rviz2-with-config-file/
# https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/multi_tb3_simulation_launch.py

commands_path = '/home/climb/ros2_ws/commands/commands.pkl'
enable_gui = False
enable_optitrack = True
enable_rviz = False

logging.get_logger('launch').info(''' 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %%%%%%       %%%%%%%%%%%%%  
 %%%                      %%%%
 %%%                       %%%
  %%%%%%%%%%%%%           %%%%
     %%%%%%%%%%%%%   %%%%%%%  
                %%%   %%%%    
               %%%%     %%%    Space Robotics Laboratory
  %%%%%%%%%%%%%%%        %%%%  Kyushu Institute of Technology

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
 
 Climbing Robots Research Group
 ROS2 Tether Climbing Robot Software
 Tetherbot Master - User Interface Launcher

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

''')


def generate_launch_description():

    executables = []

    tbot_desc_path = os.path.join(get_package_share_directory('tbotros_description'), 'desc/tetherbot.pkl')

    # user interface
    executables.append(Node(
        package = 'tetherbot_gui',
        executable = 'tetherbot_gui',
        on_exit = Shutdown(),
        parameters = [{'desc_file': tbot_desc_path}],
    ))

    return LaunchDescription(executables)