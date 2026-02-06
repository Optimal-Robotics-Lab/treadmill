#!/bin/bash
# Set ROS2 domain ID
export ROS_DOMAIN_ID=11

# add pi definition to allow for conditional launching of status led node
PI='false'
# source ROS2
source /opt/ros/jazzy/setup.bash
# source workspace (put your path here)
source install/setup.bash

MODEL_NAME=$(cat /sys/firmware/device-tree/base/model 2>/dev/null)

# check if running on Raspberry Pi
if [[ "$MODEL_NAME" == *"Raspberry Pi"* ]]; then
	PI='true'
else
	PI='false'

fi
# launch control node
ros2 launch treadmill_control treadmill_control.launch.py is_pi:=$PI
# is pi flag is used to determine if we are running on a Raspberry Pi, if we are it also runs the status led node
