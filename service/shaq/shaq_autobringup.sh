#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /home/shaq/shaq_and_koby_ws/robot_ws/install/setup.bash
source /home/shaq/shaq_and_koby_ws/shaq_and_koby_venv/bin/activate
source /home/shaq/uros_ws/install/setup.bash

# Set ROS domain
export ROS_DOMAIN_ID=10

# Launch ROS2 bringup
ros2 launch shaq_core bot_bringup.launch.py

# /etc/systemd/system/auto_bringup.service