FROM osrf/ros2:iron-desktop

RUN  apt-get update &&  apt-get upgrade -y &&  apt-get install -y \
ros-${ROS_DISTRO}-demo-nodes-cpp \
ros-${ROS_DISTRO}-demo-nodes-py \
ros-iron-rviz2

RUN mkdir -p /home/astrodeck_ws
WORKDIR /home/astrodeck_ws

# Build your ROS workspace using colcon
RUN colcon build --symlink-install

# Source the setup script to set up the ROS environment

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/astrodeck_ws/install/setup.bash" >> ~/.bashrc

ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

COPY entry.sh /entry.sh
RUN chmod +x /entry.sh
ENTRYPOINT ["/entry.sh"]
