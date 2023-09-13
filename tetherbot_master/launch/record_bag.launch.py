from __future__ import annotations

import os
import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    topics = ['/platform0/platform_controller/target_pose',
              '/platform0/platform_state_publisher/pose',
              '/platform0/platform_state_publisher/stability',
              '/rpparm0/arm_controller/target_pose',
              '/rpparm0/arm_state_publisher/pose',
              '/motor0/faulhaber_motor/current',
              '/motor0/faulhaber_motor/velocity',
              '/motor0/faulhaber_motor/position',
              '/motor1/faulhaber_motor/current',
              '/motor1/faulhaber_motor/velocity',
              '/motor1/faulhaber_motor/position',
              '/motor2/faulhaber_motor/current',
              '/motor2/faulhaber_motor/velocity',
              '/motor2/faulhaber_motor/position',
              '/motor3/faulhaber_motor/current',
              '/motor3/faulhaber_motor/velocity',
              '/motor3/faulhaber_motor/position',
              '/motor4/faulhaber_motor/current',
              '/motor4/faulhaber_motor/velocity',
              '/motor4/faulhaber_motor/position',
              '/motor5/faulhaber_motor/current',
              '/motor5/faulhaber_motor/velocity',
              '/motor5/faulhaber_motor/position',
              '/motor6/faulhaber_motor/current',
              '/motor6/faulhaber_motor/velocity',
              '/motor6/faulhaber_motor/position',
              '/motor7/faulhaber_motor/current',
              '/motor7/faulhaber_motor/velocity',
              '/motor7/faulhaber_motor/position',
              '/motor8/faulhaber_motor/current',
              '/motor8/faulhaber_motor/velocity',
              '/motor8/faulhaber_motor/position',
              '/motor9/faulhaber_motor/current',
              '/motor9/faulhaber_motor/velocity',
              '/motor9/faulhaber_motor/position',
              '/motor10/faulhaber_motor/current',
              '/motor10/faulhaber_motor/velocity',
              '/motor10/faulhaber_motor/position',
              '/motor11/faulhaber_motor/current',
              '/motor11/faulhaber_motor/velocity',
              '/motor11/faulhaber_motor/position',
              '/motor12/faulhaber_motor/current',
              '/motor12/faulhaber_motor/velocity',
              '/motor12/faulhaber_motor/position',
              '/gripper0/gripper_state_publisher/pose',
              '/gripper1/gripper_state_publisher/pose',
              '/gripper2/gripper_state_publisher/pose',
              '/gripper3/gripper_state_publisher/pose',
              '/gripper4/gripper_state_publisher/pose',
              '/gripper0/gripper_state_publisher/hold_name',
              '/gripper1/gripper_state_publisher/hold_name',
              '/gripper2/gripper_state_publisher/hold_name',
              '/gripper3/gripper_state_publisher/hold_name',
              '/gripper4/gripper_state_publisher/hold_name',
              '/mocap_optitrack/markers',
              '/mocap_optitrack/rigid_bodies']
    
    bag_file =  os.path.expanduser('~') + '/ros2_ws/bag/' + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + '/'

    return LaunchDescription([ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_file, topics], output="screen")])