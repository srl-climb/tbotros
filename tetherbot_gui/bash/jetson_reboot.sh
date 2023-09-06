#!/bin/bash

gnome-terminal --disable-factory -- bash -ic "echo 'Rebooting Jetson...' && ssh -t srl-orin@192.168.1.2 'sudo reboot'"
