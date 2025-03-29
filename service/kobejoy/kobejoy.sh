#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /home/kobejoy/shaq_and_koby_ws/robot_ws/install/setup.bash
source /home/kobejoy/shaq_and_koby_ws/shaq_and_koby_venv/bin/activate

# Set ROS domain
export ROS_DOMAIN_ID=10

# Launch ROS2 bringup
ros2 launch kobe_core joystickController.launch.py

# /etc/systemd/system/joystick_controller.service