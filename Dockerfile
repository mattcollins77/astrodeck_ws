FROM arm64v8/ros:iron

RUN  apt-get update &&  apt-get upgrade -y &&  apt-get install -y \
ros-${ROS_DISTRO}-demo-nodes-cpp \
ros-${ROS_DISTRO}-demo-nodes-py

RUN mkdir -p csrc
WORKDIR /home/astrodeck_ws

# Build your ROS workspace using colcon
RUN colcon build

# Source the setup script to set up the ROS environment
RUN /bin/bash -c "source /home/astrodeck_ws/install/setup.bash && echo 'source /home/astrodeck_ws/install/setup.bash' >> ~/.bashrc"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

