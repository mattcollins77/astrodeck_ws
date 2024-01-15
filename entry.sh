#!/bin/bash

# Source the setup script to set up the ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/astrodeck_ws/install/setup.bash
source install/setup.bash

# Run colcon to build your ROS workspace (optional, if not already built)
colcon build --symlink-install
source /home/astrodeck_ws/install/setup.bash
source install/setup.bash
# Execute the ROS launch file
/bin/bash