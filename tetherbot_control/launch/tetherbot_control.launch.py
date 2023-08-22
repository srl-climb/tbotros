from __future__ import annotations

import os
import sys
from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from tbotlib import TbTetherbot

# LINKS:
# https://answers.ros.org/question/374926/ros2-how-to-launch-rviz2-with-config-file/
# https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/multi_tb3_simulation_launch.py

enable_tf = True
enable_zed = False 
enable_cameras = False
enable_motors = True
enable_servos = False
enable_arm = True
enable_grippers = True
enable_platform = True
enable_other = True

#sys.path.append(os.path.join(get_package_prefix('tetherbot_control'), 'lib/python3.8/site-packages/tetherbot_control'))
# NOTE: tbotlib's load function is based on pickle, which has to import tbotlib in order to function
#       we add the path to the tbotlib inside the tetherbot_control install to make tbotlib importable
                              
def action_remap(_from: str, _to: str) -> list[tuple]:

    return [(_from + '/_action/feedback', _to + '/_action/feedback'),
            (_from + '/_action/status', _to + '/_action/status'),
            (_from + '/_action/cancel_goal', _to + '/_action/cancel_goal'),
            (_from + '/_action/get_result', _to + '/_action/get_result'),
            (_from + '/_action/send_goal', _to + '/_action/send_goal')]

def generate_launch_description():

    config_path = os.path.join(get_package_share_directory('tbotros_config'), 'config')
    desc_path = os.path.join(get_package_share_directory('tbotros_description'), 'desc')

    executables = []

    # tetherbot config
    tbot: TbTetherbot = TbTetherbot.load(os.path.join(desc_path, 'tetherbot_light.pkl'))
    with open(os.path.join(desc_path, 'tetherbot.urdf'), 'r') as infp: 
        robot_desc = infp.read()
  
    # === TF ===
    if enable_tf: 
        # robot state publisher
        executables.append(Node(
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            name = 'robot_state_publisher',
            parameters = [{'robot_description': robot_desc}]))
 
    # === CAMERAS AND ARUCO MARKERS ===
    # zed camera
    if enable_zed:

        # ZED camera
        executables.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('zed_wrapper'), 'launch/include', 'zed_camera.launch.py')),
            launch_arguments = {'camera_model': 'zedm', 
                                'config': os.path.join(config_path, 'zedm.yaml'),
                                'cam_pose': '[-0.27645,0,0.15625,0,3.14159,0]', #tbot.platform.depthsensor.T_local.decompose()
                               }.items()))
        # NOTE: The pose of the zed camera can be accessed via the pose topic.
        #       The pose refers to the base_link of the camera (see the urdf file of the zedm in the zed_wrapper package)
        #       The base_link is offset to the zedm_base_link (frame attached to the camera body) by the 'cam_pose' transformation parameter
        #       We set 'cam_pose' sothat the transformation platform frame -> zedm_base_link is the same as base_link -> zedm_base_link (-> platform frame and base_link become the same)

    # basler cameras/aruco detectors
    if enable_cameras:
        for i in range(1):
            camera_name = 'camera' + str(i)

            # pylon camera node
            executables.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pylon_ros2_camera_wrapper'),'launch', 'pylon_ros2_camera.launch.py')),
                    launch_arguments={'config_file': os.path.join(config_path, camera_name + '.yaml'),
                                      'camera_id': camera_name,
                                      'node_name': 'pylon_ros2_camera_node'}.items()))
            
            # camera state puslisher
            with open(os.path.join(desc_path, camera_name + '.urdf'), 'r') as infp: 
                camera_desc = infp.read()
            
            executables.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace = camera_name,
                parameters=[{'robot_description': camera_desc}]))
        
            # aruco detectors
            executables.append(Node(
                namespace = camera_name,
                package = 'aruco_detector',
                executable = 'aruco_detector',
                parameters = [{'marker_size': float(0.013),
                               'calibration_file': os.path.join(config_path, camera_name + '_calibration.yaml'), 
                               'transform_file': os.path.join(config_path, camera_name, '_transform.yaml'),
                               'camera_name': camera_name}],
                remappings = [('/my_camera/pylon_ros2_camera_node/image_raw', '/' + camera_name + '/pylon_ros2_camera_node/image_raw'),
                              ('/base_pose', '/' + tbot.platform.name + '/platform_state_publisher/pose'),
                              ('/' + camera_name + '/get_marker_pose', '/' + tbot.grippers[i].name + '/gripper_state_publisher/get_marker_pose'),
                              ('/' + camera_name + '/tf_static', '/tf_static'),
                              ('/' + camera_name + '/tf', '/tf')]))
            
    # === WIRELESS SERVOS === 
    if enable_servos:
        # serial manager
        executables.append(Node(
            package = 'serial_manager',
            executable = 'serial_manager'))

        # wireless servo central
        executables.append(Node(
            package = 'wireless_servo',
            executable = 'wireless_servo_central'))
        
        # wireless servo peripherals (gripper)
        for i in range(tbot.k):      
            executables.append(Node(
                package = 'wireless_servo',
                executable = 'wireless_servo_peripheral',
                parameters = [{'arduino_local_name': 'wireless_servo' + str(i)}],
                namespace = tbot.grippers[i].name,
                remappings = [('/' + tbot.grippers[i].name + '/wireless_servo_central/arduino_peripherals_connected', '/wireless_servo_central/arduino_peripherals_connected'),
                              ('/' + tbot.grippers[i].name +'/serial_manager/message', '/serial_manager/message')]))
        
        # wireless servo peripheral (arm)
        executables.append(Node(
            package = 'wireless_servo',
            executable = 'wireless_servo_peripheral',
            parameters = [{'arduino_local_name': 'wireless_servo' + str(i+1)}],
            namespace = tbot.platform.arm.name,
            remappings = [('/' + tbot.platform.arm.name + '/wireless_servo_central/arduino_peripherals_connected', '/wireless_servo_central/arduino_peripherals_connected'),
                          ('/' + tbot.platform.arm.name +'/serial_manager/message', '/serial_manager/message')]))
    
    # === CANOPEN AND MOTORS ===
    if enable_motors:
        executables.append(Node(
            package = 'canopen_network',
            executable = 'canopen_network'))

    # === GRIPPERS ===
    if enable_grippers:
        for i in range(tbot.k):

            # gripper state publisher
            executables.append(Node(
                package = 'tetherbot_control',
                executable = 'tetherbot_control_gripper_state_publisher',
                namespace = tbot.grippers[i].name,
                remappings = [('/' + tbot.grippers[i].name + '/marker_pose', '/camera' + str(i) + '/aruco_detector/marker_pose'),
                              ('/' + tbot.grippers[i].name + '/tf_static', '/tf_static'),
                              ('/' + tbot.grippers[i].name + '/tf', '/tf')],
                parameters = [{'config_file': os.path.join(config_path, 'tetherbot_light.pkl'),
                               'gripper_id': tbot.grippers[i].name,
                               'hold_id': str(i),
                               'config_file': os.path.join(desc_path, 'tetherbot_light.pkl'), 
                               'default_transform_source': 'hold',
                               'marker_frame_id': 'camera' + str(i) + '_marker'}]))
            
            # gripper controller
            executables.append(Node(
                package = 'tetherbot_control',
                executable = 'tetherbot_control_gripper_controller',
                namespace = tbot.grippers[i].name,
                remappings = [('/' + tbot.grippers[i].name + '/gripper_controller/contactswitch', '/' + tbot.grippers[i].name + '/wireless_servo_peripheral/limitswitch0')]))
        
    # === ARM ===
    if enable_arm:
        # arm state publisher
        executables.append(Node(
            package = 'tetherbot_control',
            namespace = tbot.platform.arm.name,
            executable = 'tetherbot_control_arm_state_publisher',
            parameters = [{'config_file': os.path.join(desc_path, 'tetherbot_light.pkl')}],
            remappings = [('/' + tbot.platform.arm.name + '/motor0/position', '/motor10/faulhaber_motor/position'),
                          ('/' + tbot.platform.arm.name + '/motor1/position', '/motor11/faulhaber_motor/position'),
                          ('/' + tbot.platform.arm.name + '/motor2/position', '/motor12/faulhaber_motor/position'),
                          ('/' + tbot.platform.arm.name + '/tf_static', '/tf_static'),
                          ('/' + tbot.platform.arm.name + '/tf', '/tf')]
            ))

        # arm controller
        executables.append(Node(
            package = 'tetherbot_control',
            namespace = tbot.platform.arm.name,
            executable = 'tetherbot_control_arm_controller',
            parameters = [{'config_file': os.path.join(desc_path, 'tetherbot_light.pkl')}],
            remappings = [*action_remap('/' + tbot.platform.arm.name + '/motor10/faulhaber_motor/move', '/motor10/faulhaber_motor/move'),
                          *action_remap('/' + tbot.platform.arm.name + '/motor11/faulhaber_motor/move', '/motor11/faulhaber_motor/move'),
                          *action_remap('/' + tbot.platform.arm.name + '/motor12/faulhaber_motor/move', '/motor12/faulhaber_motor/move'),
                          ('/' + tbot.platform.arm.name + '/motor10/faulhaber_motor/target_position', '/motor10/faulhaber_motor/target_position'),
                          ('/' + tbot.platform.arm.name + '/motor11/faulhaber_motor/target_position', '/motor11/faulhaber_motor/target_position'),
                          ('/' + tbot.platform.arm.name + '/motor12/faulhaber_motor/target_position', '/motor12/faulhaber_motor/target_position'),
                          ('/' + tbot.platform.arm.name + '/motor10/faulhaber_motor/running', '/motor10/faulhaber_motor/running'),
                          ('/' + tbot.platform.arm.name + '/motor11/faulhaber_motor/running', '/motor11/faulhaber_motor/running'),
                          ('/' + tbot.platform.arm.name + '/motor12/faulhaber_motor/running', '/motor12/faulhaber_motor/running'),
                          ('/' + tbot.platform.arm.name + '/motor10/faulhaber_motor/mode', '/motor10/faulhaber_motor/mode'),
                          ('/' + tbot.platform.arm.name + '/motor11/faulhaber_motor/mode', '/motor11/faulhaber_motor/mode'),
                          ('/' + tbot.platform.arm.name + '/motor12/faulhaber_motor/mode', '/motor12/faulhaber_motor/mode'),
                          ('/' + tbot.platform.arm.name + '/' + tbot.platform.name + '/platform_state_publisher/pose', '/' + tbot.platform.name + '/platform_state_publisher/pose'),
                          ('/' + tbot.platform.arm.name + '/arm_controller/actual_pose', '/' + tbot.platform.arm.name + '/arm_state_publisher/pose'),
                          ('/' + tbot.platform.arm.name + '/tf_static', '/tf_static'),
                          ('/' + tbot.platform.arm.name + '/tf', '/tf')]
            ))
        
        # docking controller
        executables.append(Node(
            package = 'tetherbot_control',
            executable = 'tetherbot_control_gripper_controller',
            name = 'docking_controller',
            namespace = tbot.platform.arm.name,
            remappings = [('/' + tbot.platform.arm.name + '/docking_controller/contactswitch', '/' + tbot.platform.arm.name + '/wireless_servo_peripheral/limitswitch0')]))
        
    # === PLATFORM ===
    if enable_platform:
        # platform state publisher
        executables.append(Node(
            package = 'tetherbot_control',
            namespace = tbot.platform.name,
            executable='tetherbot_control_platform_state_publisher',
            parameters=[{'config_file': os.path.join(desc_path, 'tetherbot_light.pkl')}],
            remappings = [('/' + tbot.platform.name + '/motor0/position', '/motor0/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor1/position', '/motor1/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor2/position', '/motor2/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor3/position', '/motor3/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor4/position', '/motor4/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor5/position', '/motor5/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor6/position', '/motor6/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor7/position', '/motor7/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor8/position', '/motor8/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/motor9/position', '/motor9/faulhaber_motor/position'),
                          ('/' + tbot.platform.name + '/set_pose', '/zedm/zed_node/set_pose'),
                          ('/' + tbot.platform.name + '/tf_static', '/tf_static'),
                          ('/' + tbot.platform.name + '/tf', '/tf')]
            ))

        # platform controller
        executables.append(Node(
            package = 'tetherbot_control',
            namespace = tbot.platform.name,
            executable = 'tetherbot_control_platform_controller',
            parameters = [{'config_file': os.path.join(desc_path, 'tetherbot_light.pkl')}],
            remappings = [*action_remap('/' + tbot.platform.name + '/motor0/faulhaber_motor/move', '/motor0/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor1/faulhaber_motor/move', '/motor1/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor2/faulhaber_motor/move', '/motor2/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor3/faulhaber_motor/move', '/motor3/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor4/faulhaber_motor/move', '/motor4/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor5/faulhaber_motor/move', '/motor5/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor6/faulhaber_motor/move', '/motor6/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor7/faulhaber_motor/move', '/motor7/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor8/faulhaber_motor/move', '/motor8/faulhaber_motor/move'),
                        *action_remap('/' + tbot.platform.name + '/motor9/faulhaber_motor/move', '/motor9/faulhaber_motor/move'),
                        ('/' + tbot.platform.name + '/motor0/faulhaber_motor/target_position', '/motor0/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor1/faulhaber_motor/target_position', '/motor1/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor2/faulhaber_motor/target_position', '/motor2/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor3/faulhaber_motor/target_position', '/motor3/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor4/faulhaber_motor/target_position', '/motor4/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor5/faulhaber_motor/target_position', '/motor5/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor6/faulhaber_motor/target_position', '/motor6/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor7/faulhaber_motor/target_position', '/motor7/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor8/faulhaber_motor/target_position', '/motor8/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor9/faulhaber_motor/target_position', '/motor9/faulhaber_motor/target_position'),
                        ('/' + tbot.platform.name + '/motor0/faulhaber_motor/running', '/motor0/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor1/faulhaber_motor/running', '/motor1/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor2/faulhaber_motor/running', '/motor2/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor3/faulhaber_motor/running', '/motor3/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor4/faulhaber_motor/running', '/motor4/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor5/faulhaber_motor/running', '/motor5/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor6/faulhaber_motor/running', '/motor6/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor7/faulhaber_motor/running', '/motor7/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor8/faulhaber_motor/running', '/motor8/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor9/faulhaber_motor/running', '/motor9/faulhaber_motor/running'),
                        ('/' + tbot.platform.name + '/motor0/faulhaber_motor/mode', '/motor0/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor1/faulhaber_motor/mode', '/motor1/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor2/faulhaber_motor/mode', '/motor2/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor3/faulhaber_motor/mode', '/motor3/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor4/faulhaber_motor/mode', '/motor4/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor5/faulhaber_motor/mode', '/motor5/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor6/faulhaber_motor/mode', '/motor6/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor7/faulhaber_motor/mode', '/motor7/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor8/faulhaber_motor/mode', '/motor8/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/motor9/faulhaber_motor/mode', '/motor9/faulhaber_motor/mode'),
                        ('/' + tbot.platform.name + '/' + tbot.grippers[0].name + '/gripper_state_publisher/pose', '/' + tbot.grippers[0].name + '/gripper_state_publisher/pose'),
                        ('/' + tbot.platform.name + '/' + tbot.grippers[1].name + '/gripper_state_publisher/pose', '/' + tbot.grippers[1].name + '/gripper_state_publisher/pose'),
                        ('/' + tbot.platform.name + '/' + tbot.grippers[2].name + '/gripper_state_publisher/pose', '/' + tbot.grippers[2].name + '/gripper_state_publisher/pose'),
                        ('/' + tbot.platform.name + '/' + tbot.grippers[3].name + '/gripper_state_publisher/pose', '/' + tbot.grippers[3].name + '/gripper_state_publisher/pose'),
                        ('/' + tbot.platform.name + '/' + tbot.grippers[4].name + '/gripper_state_publisher/pose', '/' + tbot.grippers[4].name + '/gripper_state_publisher/pose'),
                        ('/' + tbot.platform.name + '/platform_controller/actual_pose', '/' + tbot.platform.name + '/platform_state_publisher/pose'),
                        ('/' + tbot.platform.name + '/tf_static', '/tf_static'),
                        ('/' + tbot.platform.name + '/tf', '/tf')]
            ))

    # === OTHER ===
    if enable_other:
        # tetherbot visualization publisher for tethers etc...
        executables.append(Node(
            package = 'tetherbot_control',
            parameters = [{'config_file': os.path.join(desc_path, 'tetherbot_light.pkl')}],
            executable = 'tetherbot_control_visualization_publisher'
        ))

    return LaunchDescription(executables)
