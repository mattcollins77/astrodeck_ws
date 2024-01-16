#!/bin/bash

# Source the setup script to set up the ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Run colcon to build your ROS workspace (optional, if not already built)
colcon build --symlink-install
source install/setup.bash
# Execute the ROS launch file
/bin/bash