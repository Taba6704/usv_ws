# MOB-SEA USV ROS 2 Workspace

This repository contains the ROS 2 workspace used for the Raspberry Pi 4b in the MOB-SEA unmanned surface vehicle.

## Workspace Location
This repository is intended to live at: usv_ws/

## Packages
-usv_thrusters
-rc_control

## Build Instructions
'''bash
cd ~/usv_ws
colcon build
source install/setup.bash

## Platform
Ubuntu 22.04
ROS 2 Humble
Raspberry Pi 4b

## Commit Locally
'''bash
git add .
git commit -m "Initial commit: MOB-SE USV ROS 2 workspace"
