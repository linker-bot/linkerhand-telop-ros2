#!/bin/bash
set -e

# LinkerHand teleoperation startup script
# Note: serial permission handling in this helper is only for the LinkerFFG module.

# Grant serial permissions for LinkerFFG devices when present.
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || true
sudo chmod 666 /dev/ttyUSB1 2>/dev/null || true
source "$(dirname "$0")/install/setup.bash"

# Run with calibration when needed:
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=True

# Run without calibration:
# ros2 run linkerhand_retarget handretarget
