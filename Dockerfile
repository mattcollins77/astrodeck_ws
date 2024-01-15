FROM arm64v8/ros:iron

RUN sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get install -y \
ros-${ROS_DISTRO}-demo-nodes-cpp \
ros-${ROS_DISTRO}-demo-nodes-py

RUN mkdir -p /home/astrodeck_ws/src
WORKDIR /home/astrodeck_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

