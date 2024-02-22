1. When connecting ESP32 it intefers with brltty, so need to remove it.
   sudo apt-get remove brltty

2. Need to setup aliases for USB devices so they always connect
   sudo nano /etc/udev/rules.d/99-usb-aliases.rules
   and add this to it. then reboot.

SUBSYSTEM=="usb", ATTRS{idVendor}=="1ffb", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", SYMLINK+="MyTeensy"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1ffb", SYMLINK+="MyMaestro"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="MyESP32"

3. Follow this for the RS485 hat
   https://www.waveshare.com/wiki/2-CH_RS485_HAT but need to add line to sudo nano /boot/firmware/config.txt

4. Install ROS Iron

5. Source the install by adding to bash
   echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

6. Clone my github
   git clone https://github.com/mattcollins77/astrodeck_ws.git

7. Install these
   sudo apt install ros-iron-joy
   sudo apt install joystick
   sudo apt install ros-iron-foxglove-bridge
   python3 -m pip install pyserial
