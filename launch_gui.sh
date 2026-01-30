#!/bin/bash
# Set ROS2 domain ID
export ROS_DOMAIN_ID=11

# Source ROS2
source opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 run treadmill_gui treadmill_gui_node
