# UdexReal (LinkerTG) Data Glove Module

UdexReal / LinkerTG data glove ROS2 driver module, receives glove data via UDP and publishes to ROS2 topics.

## Features

- Receives glove data via UDP protocol
- Supports left and right hand data
- 50Hz publish rate

## Usage

### 1. Config File

Edit `config/base_config.yml`:

```yaml
system:
  motion_type: udexreal

udp:
  ip: "0.0.0.0"
  port: 8888
```

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
motion/udexreal/
├── __init__.py
├── retarget.py      # ROS2 integration
└── README.md        # English documentation
```

## Notes

1. Ensure UDP port is not occupied
2. Glove and host must be on the same network
