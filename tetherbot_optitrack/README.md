# Installation

For installation you will need GitHub code from Mocap.

Use

`source install.sh`

to install everything you need, you need admin rights for this.

# Client Side

## Configurations

Things to change on first use depending on your configuration:

- Rviz file inside main launch file has local path
- Mocap Configuration file (inside config folder) needs to be copied into mocap_optitrack_driver config folder and substitutes the existing config
- Motive needs to be running and the markers need to be combined into a rigid body so that the correct data is sent
- Inside Motive the coordinate system needs to be Y-Up

## Running the nodes

`ros2 launch tetherbot_optitrack main.launch.xml`

To start everything needed to publish the pose of Tetherbot

## Calibration
Place the robot in the corner and call the calibration service

# Server Side
On the Windows PC open the Motive Software

## First Steps
To calibrate Optitrack from scratch follow the [documentation](https://docs.optitrack.com/v/v2.3/motive/calibration) (we are running version 1 but documentation for version 2 is close enough).
1. Set up the cameras, remove all markers from the robot and check the camera output for reflections.
2. If there are reflections that you cannot remove, press `Mask Visible` in the `Calibration Tab` as stated in the documentation, these points will not be recognized by Optitrack even if the markers lie within them
3. Make sure to select the correct calibration wand and start the calibration by pressing `Start Wanding` in the `Calibration Tab`. Collect enough samples until the field below turns green and the button `Calculate` gets selectable.
4. Now place the Calibration Square (the triangle) in the corner and switch to the `Ground Plane` tab. Set vertical offset to 0 and press `Set Ground Plane`, the cameras should be on the right position now.
5. Last, take out the calibration square and place the markers on the robot. You should be able to see the markers now in the Motive Software. Select all the markers and combine them to a rigid body by right clicking on them and selecting `Add selected markers to rigid body` as stated in the [docs](https://docs.optitrack.com/v/v2.3/motive/rigid-body-tracking#adding-or-removing-markers).
6. If the data streaming has been set up already, the information is now streamed to the client at the defined IP address. If not follow the instructions from the [documentation](https://docs.optitrack.com/v/v2.3/motive/data-streaming). The only thing that needs to be done is setting the IP and enabling the data sending of rigid bodies.









