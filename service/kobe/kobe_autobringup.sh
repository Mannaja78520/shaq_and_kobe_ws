#!/bin/bash

# Source ROS2 enviroment
source /opt/ros/jazzy/setup.bash
source /home/reaiBot/shaq_and_koby_ws/shaq_and_koby_venv/bin/activate
source /home/reaiBot/microros_ws/install/local_setup.bash
source /home/reaiBot/shaq_and_koby_ws/robot_ws/install/setup.bash

# Set ROS domain 
export ROS_DOMAIN_ID=10

# Launch ROS2 bringup
ros2 launch kobe_core bot_bringup.launch.py

# /etc/systemd/system/auto_bringup.service
