import os
from glob import glob
from setuptools import setup

package_name = 'tetherbot_optitrack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', 'calibration_offsets.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Giese',
    maintainer_email='climb@todo.todo',
    description='Visual Based Navigation with Optitrack and TF reconstruction',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_tetherbot_motive_transform_broadcaster = tetherbot_optitrack.ros2_tetherbot_motive_transform_broadcaster:main',
            'ros2_tetherbot_tbot_pose_publisher = tetherbot_optitrack.ros2_tetherbot_tbot_pose_publisher:main',
            'tetherbot_tbot_calibration = tetherbot_optitrack.ros2_tetherbot_tbot_calibration:main',
        ],
    },
)
