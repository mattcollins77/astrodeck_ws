#!/bin/bash

# Source the setup script to set up the ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/astrodeck_ws/install/setup.bash

# Run colcon to build your ROS workspace (optional, if not already built)
colcon build --symlink-install

# Execute the ROS launch file
ros2 launch my_package talker.launch.py
