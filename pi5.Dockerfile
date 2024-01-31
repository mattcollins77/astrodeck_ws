FROM arm64v8/ros:iron

RUN  apt-get update &&  apt-get upgrade -y &&  apt-get install -y \
ros-${ROS_DISTRO}-demo-nodes-cpp \
ros-${ROS_DISTRO}-demo-nodes-py \
ros-${ROS_DISTRO}-joy \
joystick \
python3-serial \
python3-pip \
ros-iron-foxglove-bridge

RUN mkdir -p /home/astrodeck_ws
WORKDIR /home/astrodeck_ws
RUN python3 -m pip install pyserial


COPY entry.sh /entry.sh
RUN chmod +x /entry.sh
ENTRYPOINT ["/entry.sh"]
