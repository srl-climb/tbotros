#!/bin/bash

gnome-terminal --disable-factory -- bash -ic "echo 'Launching play_bag.launch.py on host...' && ros2 launch tetherbot_master play_bag.launch.py ; exec bash"
