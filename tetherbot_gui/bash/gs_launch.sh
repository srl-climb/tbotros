#!/bin/bash

gnome-terminal --disable-factory -- bash -ic "echo 'Launching gs.launch.py on host...' && ros2 launch tetherbot_master gs.launch.py ; exec bash"
