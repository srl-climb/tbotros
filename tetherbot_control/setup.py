import os
from glob import glob
from setuptools import setup

package_name = 'tetherbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.stl')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.config')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Harms',
    maintainer_email='harms.simon759@mail.kyutech.jp',
    description='Package to provide control functions for a tethered climbing robot',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'tetherbot_control_gripper_state_publisher = tetherbot_control.tetherbot_control_gripper_state_publisher_node:main',
        'tetherbot_control_gripper_controller = tetherbot_control.tetherbot_control_gripper_controller_node:main',
        'tetherbot_control_platform_state_publisher = tetherbot_control.tetherbot_control_platform_state_publisher_node:main',
        'tetherbot_control_arm_state_publisher = tetherbot_control.tetherbot_control_arm_state_publisher_node:main',
        'tetherbot_control_visualization_publisher = tetherbot_control.tetherbot_control_visualization_publisher_node:main',
        'tetherbot_control_arm_controller = tetherbot_control.tetherbot_control_arm_controller_node:main',
        'tetherbot_control_platform_controller = tetherbot_control.tetherbot_control_platform_controller_node:main',
        'tetherbot_control_sequencer = tetherbot_control.tetherbot_control_sequencer_node:main',
        'tetherbot_control_planner = tetherbot_control.tetherbot_control_planner_node:main'
        ],
    },
)
