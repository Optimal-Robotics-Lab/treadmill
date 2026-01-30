#!/bin/bash
# Set ROS2 domain ID
export ROS_DOMAIN_ID=11

# source ROS2
source /opt/ros/jazzy/setup.bash
# source workspace (put your path here)
source install/setup.bash

# launch control node
ros2 launch treadmill_control treadmill_control.launch.py
