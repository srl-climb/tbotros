from __future__ import annotations

import os
from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription, Shutdown
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# LINKS:
# https://answers.ros.org/question/374926/ros2-how-to-launch-rviz2-with-config-file/
# https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/multi_tb3_simulation_launch.py

commands_path = '/home/climb/ros2_ws/commands/commands.pkl'
enable_gui = False
enable_optitrack = False
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
 Tetherbot Master - Ground Station Launcher

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

''')
                                  
def action_remap(_from: str, _to: str) -> list[tuple]:

    return [(_from + '/_action/feedback', _to + '/_action/feedback'),
            (_from + '/_action/status', _to + '/_action/status'),
            (_from + '/_action/cancel_goal', _to + '/_action/cancel_goal'),
            (_from + '/_action/get_result', _to + '/_action/get_result'),
            (_from + '/_action/send_goal', _to + '/_action/send_goal')]

def generate_launch_description():

    executables = []

    planner_config_path = os.path.join(get_package_share_directory('tbotros_config'), 'config/planner.yaml')
    tbot_desc_path = os.path.join(get_package_share_directory('tbotros_description'), 'desc/tetherbot.pkl')

    # path planner
    executables.append(Node(
        package = 'tetherbot_planner',
        executable = 'tetherbot_planner',
        parameters = [{'commands_path': commands_path,
                       'config_file': tbot_desc_path,
                       'planner_config_path': planner_config_path}],
        output = 'both'
    ))

    # command sequencer
    executables.append(Node(
        package = 'tetherbot_sequencer',
        executable = 'tetherbot_sequencer',
        parameters = [{'commands_path': commands_path,
                       'config_file': tbot_desc_path}]
    ))

    # user interface
    if enable_gui:
        executables.append(Node(
            package = 'tetherbot_gui',
            executable = 'tetherbot_gui',
            on_exit = Shutdown()
        ))

    # optitrack
    if enable_optitrack:
        executables.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('tetherbot_optitrack'), 'launch', 'tetherbot_optitrack.launch.xml'))
            ))
        
    # rviz
    if enable_rviz:
        executables.append(Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('tbotros_config'), 'config', 'rviz.rviz')]
            ))

    return LaunchDescription(executables)
