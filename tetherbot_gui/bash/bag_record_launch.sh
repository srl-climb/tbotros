#!/bin/bash

gnome-terminal --disable-factory -- bash -ic "echo 'Launching record_bag.launch.py on host...' && ros2 launch tetherbot_master record_bag.launch.py ; exec bash"
