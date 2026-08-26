# LinkerMCG_G7 Data Glove Module

LinkerMCG_G7 data glove ROS2 driver module, receives glove data via UDP and publishes to ROS2 topics.

## Features

- Receives glove data via UDP protocol
- Supports left and right hand data
- 50Hz publish rate

## Usage

### 1. Config File

Edit `config/base_config.yml`:

```yaml
system:
  motion_type: linkermcg_g7

udp:
  ip: "<target-ip>"
  port: 9011
```

`udp.ip` must be the target host address that receives the UDP stream. Do not use `0.0.0.0`.

The legacy value `motion_type: linkermcg` remains supported as an alias of `linkermcg_g7`.

**Important:** enable both `debug.joint_motor_debug_l` and `debug.joint_motor_debug_r` to print UDP data in the debug output.

### 2. Launch

```bash
ros2 run linkerhand_retarget handretarget
```

## ROS2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/cb_right_hand_control_cmd` | `sensor_msgs/JointState` | Right hand control data |
| `/cb_left_hand_control_cmd` | `sensor_msgs/JointState` | Left hand control data |

### Message Format

```python
# sensor_msgs/JointState
msg.header.stamp  # Timestamp
msg.name          # Joint name list
msg.position      # Joint position values
```

## File Structure

```
motion/linkermcg/
├── __init__.py
├── retarget.py      # ROS2 integration
└── README.md        # English documentation
```

## Notes

1. Ensure UDP port is not occupied
2. Glove and host must be on the same network
3. Enable both `debug.joint_motor_debug_l` and `debug.joint_motor_debug_r` to show the UDP data in the debug output
