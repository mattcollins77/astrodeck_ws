FROM arm64v8/ros:iron

RUN  apt-get update &&  apt-get upgrade -y &&  apt-get install -y \
ros-${ROS_DISTRO}-demo-nodes-cpp \
ros-${ROS_DISTRO}-demo-nodes-py

RUN mkdir -p /home/astrodeck_ws/src
WORKDIR /home/astrodeck_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

