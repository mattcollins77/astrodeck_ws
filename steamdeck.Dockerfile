FROM ros:iron

RUN  apt-get update &&  apt-get upgrade -y &&  apt-get install -y \
ros-${ROS_DISTRO}-demo-nodes-cpp \
ros-${ROS_DISTRO}-demo-nodes-py

RUN mkdir -p /home/astrodeck_ws
WORKDIR /home/astrodeck_ws

# Build your ROS workspace using colcon
RUN colcon build --symlink-install

# Source the setup script to set up the ROS environment

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/astrodeck_ws/install/setup.bash" >> ~/.bashrc

COPY entry.sh /entry.sh
RUN chmod +x /entry.sh
ENTRYPOINT ["/entry.sh"]
