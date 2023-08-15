from __future__ import annotations

import os
from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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
 Tetherbot Master - Tetherbot Launcher

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

''')
                                  
def generate_launch_description():

    executables = []

    executables.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tetherbot_control'), 'launch', 'tetherbot_control.launch.py'))))
    
    return LaunchDescription(executables)
