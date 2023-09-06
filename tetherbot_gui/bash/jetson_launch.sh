#!/bin/bash

gnome-terminal --disable-factory -- bash -ic " \
echo 'Launching tbot.launch.py on Jetson...' && \
ssh -t srl-orin@192.168.1.2 ' \
source /opt/ros/foxy/setup.bash && source ~/ros2_base/install/setup.bash && source ~/ros2_ws/install/setup.bash && /opt/ros/foxy/bin/ros2 launch tetherbot_master tbot.launch.py ; exec bash'"
