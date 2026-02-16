#!/bin/bash

source install/setup.bash

ros2 topic pub treadmill/special_cmd std_msgs/msg/String "data: on" --once

ros2 topic pub treadmill/special_cmd std_msgs/msg/String "data: start" --once

ros2 topic pub treadmill/special_cmd std_msgs/msg/Float32 "data: start" --once

ros2 topic pub --once treadmill/cmd_speed_mps std_msgs/msg/Float32 "{data: 1.00}"
