from launch import LaunchDescription
from launch_ros.actions import Node

# Rotation Order: zyx and clockwise is positive, anti-clockwise negative

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace = 'optitrack',
            package = 'tetherbot_optitrack',
            executable = 'calibration_node',
            remappings = [('/optitrack/tf', '/tf'), ('/optitrack/tf_static', '/tf_static')]
        ),
        Node(
            namespace = 'optitrack',
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            arguments = ['0', '0.1', '0', '1.5708', '0', '1.5708', 'map', 'motive_origin']
        ),
        Node(
            namespace = 'optitrack',
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            arguments = ['0.12', '0.28495', '0.068', '1.5708', '0', '0', 'map', 'calibration_target']
        ),
        Node(
            namespace = 'optitrack',
            package = 'tetherbot_optitrack',
            executable = 'motive_transform_broadcaster',
            remappings = [('/optitrack/tf', '/tf'), ('/optitrack/tf_static', '/tf_static')]
        ),
        Node(
            namespace = 'optitrack',
            package = 'tetherbot_optitrack',
            executable = 'pose_publisher',
            remappings = [('/optitrack/tf', '/tf'), ('/optitrack/tf_static', '/tf_static')]
        )
    ])