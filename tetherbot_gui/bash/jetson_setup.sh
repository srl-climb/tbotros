#!/bin/bash

gnome-terminal --disable-factory -- bash -ic "echo 'Seting up Jetson...' && ssh -t srl-orin@192.168.1.2 'cd ~/ros2_ws/src/tbotros_hw/bash && sudo sh setup.sh'; sleep 2"
