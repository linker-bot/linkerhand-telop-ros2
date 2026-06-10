# Version Notes

## v2.12.5

- Added independent O20 and O30 robot hand model registration, runtime routing, hand wrappers, and URDF assets.
- Added O20-specific LinkerFFG mapping: O20 open maps to motor `0`, fist maps to motor `255`, and thumb opose output is calibrated separately from URDF pose targets.
- Preserved calibration anchor accuracy by returning exact robot URDF targets when glove input matches `original`, `opose`, or `fist` calibration states.
- Updated ROS2 documentation and LinkerFFG module documentation with O20/O30 support notes.
- Added regression coverage for O20/O30 assets, O20 motor direction, O20 thumb opose output, and calibration anchor behavior.

## v2.12.4

- Standalone ROS2 repository split from the mixed workspace.
- Root documentation simplified to ROS2-only build and run instructions.
- Standardized all release-facing version references to `2.12.4`.
- Removed launch-install guidance in favor of `ros2 run`.
- Kept subtree history for the ROS2 package.
