#!/bin/bash

gnome-terminal --disable-factory -- bash -ic "echo 'Opening Jetson terminal...' && ssh srl-orin@192.168.1.2 ; exec bash"
