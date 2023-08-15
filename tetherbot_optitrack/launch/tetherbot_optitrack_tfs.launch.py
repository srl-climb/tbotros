from launch import LaunchDescription
from launch_ros.actions import Node

# Rotation Order: zyx and clockwise is positive, anti-clockwise negative

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0.1', '0', '1.5708', '0', '1.5708', 'map', 'motive_origin']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.12', '0.28495', '0.068', '1.5708', '0', '0', 'map', 'calibration_target']
        ),
        Node(
            package='tetherbot_optitrack',
            executable='ros2_tetherbot_motive_transform_broadcaster'
        ),
        Node(
            package='tetherbot_optitrack',
            executable='ros2_tetherbot_tbot_pose_publisher'
        )
    ])