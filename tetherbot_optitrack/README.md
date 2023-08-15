# Installation

For installation you will need GitHub code from Mocap.

Use

source install.sh

to install everything you need, you need admin rights for this.

# Changes

Things to change on first use depending on your configuration:

- Rviz file inside main launch file has local path
- Mocap Configuration file (inside config folder) needs to be copied into mocap_optitrack_driver config folder and substitutes the existing config
- Motive needs to be running and the markers need to be combined into a rigid body so that the correct data is sent
- Inside Motive the coordinate system needs to be Y-Up

# Run

ros2 launch tetherbot_optitrack main.launch.xml

To start everything needed to publish the pose of Tetherbot