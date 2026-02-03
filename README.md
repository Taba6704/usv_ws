# MOB-SEA USV ROS 2 Workspace

This repository contains the ROS 2 workspace used for the MOB-SEA unmanned surface vehicle.

## Workspace Location
This repository is intended to live at: mobsea_ws/usv_ws

## Packages
-usv_thrusters
-navigation
-localization
-perception

## Build Instructions
'''bash
cd mobsea_ws/usv_ws
colcon build
source install/setup.bash

## Platform
Ubuntu 22.04
ROS 2 Humble
Raspberry Pi 4B

## Commit Locally
'''bash
git add .
git commit -m "Initial commit: MOB-SE USV ROS 2 workspace"
