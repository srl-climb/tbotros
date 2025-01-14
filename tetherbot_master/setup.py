import os
from glob import glob
from setuptools import setup

package_name = 'tetherbot_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Harms',
    maintainer_email='harms.simon759@mail.kyutech.jp',
    description='Package with main launch files for the tethered climbing robot',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
