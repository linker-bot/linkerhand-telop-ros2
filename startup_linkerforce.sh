#!/bin/bash
set -e

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
source "$(dirname "$0")/install/setup.bash"
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=auto_calibrate
