#!/bin/bash

gnome-terminal --disable-factory -- bash -ic "echo 'Opening Jetson monitor...' && ssh -t srl-orin@192.168.1.2 'jtop'"
