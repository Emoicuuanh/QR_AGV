#!/bin/bash

# First Update
echo "========================================================================="
echo "                           Starting Install                              "
echo "========================================================================="

# TODO: Check ROS version to determine Python version and which pkg need for
# each version
sudo apt-get install -y ros-$ROS_DISTRO-ecl-threads \
                     -y ros-$ROS_DISTRO-ecl-formatters \
                     -y ros-$ROS_DISTRO-ecl-linear-algebra \

# For joystick_driver
sudo apt-get install libspnav-dev libbluetooth-dev libcwiid-dev