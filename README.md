# LinkerHand ROS2 Teleop SDK

ROS2-only teleoperation and motion retargeting repository.

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run linkerhand_retarget handretarget
```

For calibration:

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=auto_calibrate
```

## Workspace Layout

```
linkerhand-telop-ros2/
├── src/
└── README.md
```

## Notes

- Build from a ROS2 source workspace only.
- Use `colcon build --symlink-install`.
- Launch through `ros2 run`; launch-file installation is not used in this repository.

