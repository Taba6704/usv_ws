#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/mobsea/usv_ws/install/setup.bash
exec ros2 launch usv_bringup pi_bringup.launch.py