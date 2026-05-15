# LinkerEG Teleoperation Glove Module

LinkerEG teleoperation glove ROS2 driver module, receives glove data via serial port and publishes to ROS2 topics.

## Changelog

### 2026-02-04
- Fixed dead loop bug in initialization
- Added support for G20 hand

## Features

- **Publish rate**: 50Hz (all topics unified)
- Auto scan serial port for glove connection
- Supports left and right hand data
- Supports sensor raw data publishing (switchable via topic)

## Control Modes

| motion_type | Mode | Description | Receiver needs robot hand connection |
|-------------|------|-------------|-------------------------------------|
| `linkereg2` | SDK Control Mode | Glove data publishes to ROS topics directly, SDK controls robot hand | ❌ Not required |
| `linkereg1` | Receiver Control Mode | Receiver controls robot hand directly, also publishes data to ROS topics | ✅ Required |

## Usage

### 1. Config File

Edit `config/base_config.yml`:

```yaml
system:
  # SDK control mode (receiver doesn't need robot hand connection)
  motion_type: linkereg2

  # Or receiver control mode (receiver must be connected to robot hand)
  motion_type: linkereg1
```

#### Serial Port Permission Password

If auto-fixing serial port permissions is needed, configure `password` under `linkereg` (sudo password).
Note: Values without quotes in YAML are parsed as numbers, use quotes for strings.

```yaml
linkereg:
  port: null
  password: "123456"  # sudo password for fixing serial permissions
```

### 2. Launch

```bash
ros2 run linkerhand_retarget handretarget
```

### 3. Enable Debug Print

Edit `config/base_config.yml`:

```yaml
debug:
  joint_pub_debug: true
```

## ROS2 Topics

> **Publish rate**: All topics unified at 50Hz

### Published Topics

| Topic | Message Type | Description | Default State |
|-------|-------------|-------------|---------------|
| `/cb_right_hand_control_cmd` | `sensor_msgs/JointState` | Right hand control data | ✅ Enabled |
| `/cb_left_hand_control_cmd` | `sensor_msgs/JointState` | Left hand control data | ✅ Enabled |
| `/cb_right_hand_raw_data` | `sensor_msgs/JointState` | Right hand sensor raw data | ❌ Disabled |
| `/cb_left_hand_raw_data` | `sensor_msgs/JointState` | Left hand sensor raw data | ❌ Disabled |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/cb_hand_setting_cmd` | `std_msgs/String` | Settings command (control raw data on/off) |

## Raw Data Feature

Raw data is **disabled** by default, enable via topic command.

### Enable/Disable Raw Data

```bash
# Enable raw data publishing
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/String "data: 'on'"

# Disable raw data publishing
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/String "data: 'off'"

# Listen to raw data
ros2 topic echo /cb_left_hand_raw_data
```

## ROS2 Topic Data Format

### Raw Data Format (15 int32 values)

| Index | Joint Name | Index | Joint Name | Index | Joint Name |
|-------|------------|-------|------------|-------|------------|
| 0 | Thumb abduction | 1 | Thumb flexion | 2 | Thumb tip |
| 3 | Index abduction | 4 | Index flexion | 5 | Index tip |
| 6 | Middle abduction | 7 | Middle flexion | 8 | Middle tip |
| 9 | Ring abduction | 10 | Ring flexion | 11 | Ring tip |
| 12 | Pinky abduction | 13 | Pinky flexion | 14 | Pinky tip |

### Control Data Format

#### L21 (25 joint output)

For robot hand models: L21

| Joint | Joint Name | Description |
|-------|------------|-------------|
| joint1 | Thumb flexion | |
| joint2 | Index flexion | |
| joint3 | Middle flexion | |
| joint4 | Ring flexion | |
| joint5 | Pinky flexion | |
| joint6 | Thumb abduction | |
| joint7 | Index abduction | |
| joint8 | Middle abduction | |
| joint9 | Ring abduction | |
| joint10 | Pinky abduction | |
| joint11 | Thumb roll | |
| joint12 | Reserved | value 0 |
| joint13 | Reserved | value 0 |
| joint14 | Reserved | value 0 |
| joint15 | Reserved | value 0 |
| joint16 | Thumb middle | value 0 |
| joint17 | Reserved | value 0 |
| joint18 | Reserved | value 0 |
| joint19 | Reserved | value 0 |
| joint20 | Reserved | value 0 |
| joint21 | Thumb tip | |
| joint22 | Index tip | |
| joint23 | Middle tip | |
| joint24 | Ring tip | |
| joint25 | Pinky tip | |

#### L20/G20 (20 joint output)

For robot hand models: L20, Industrial 20, G20

| Joint | Joint Name | Description |
|-------|------------|-------------|
| joint1 | Thumb flexion | |
| joint2 | Index flexion | |
| joint3 | Middle flexion | |
| joint4 | Ring flexion | |
| joint5 | Pinky flexion | |
| joint6 | Thumb abduction | |
| joint7 | Index abduction | |
| joint8 | Middle abduction | |
| joint9 | Ring abduction | |
| joint10 | Pinky abduction | |
| joint11 | Thumb roll | |
| joint12 | Reserved | value 0 |
| joint13 | Reserved | value 0 |
| joint14 | Reserved | value 0 |
| joint15 | Reserved | value 0 |
| joint16 | Thumb tip | |
| joint17 | Index tip | |
| joint18 | Middle tip | |
| joint19 | Ring tip | |
| joint20 | Pinky tip | |

#### L10 (10 joints)

For robot hand models: L10

| Joint | Joint Name |
|-------|------------|
| joint1 | Thumb flexion |
| joint2 | Thumb abduction |
| joint3 | Index flexion |
| joint4 | Middle flexion |
| joint5 | Ring flexion |
| joint6 | Pinky flexion |
| joint7 | Index abduction |
| joint8 | Ring abduction |
| joint9 | Pinky abduction |
| joint10 | Thumb roll |

#### L6 (6 joints)

For robot hand models: L6, O6

| Joint | Joint Name |
|-------|------------|
| joint1 | Thumb flexion |
| joint2 | Thumb abduction |
| joint3 | Index flexion |
| joint4 | Middle flexion |
| joint5 | Ring flexion |
| joint6 | Pinky flexion |

#### O7 (7 joints)

For robot hand models: O7

| Joint | Joint Name |
|-------|------------|
| joint1 | Thumb flexion |
| joint2 | Thumb abduction |
| joint3 | Index flexion |
| joint4 | Middle flexion |
| joint5 | Ring flexion |
| joint6 | Pinky flexion |
| joint7 | Thumb roll |

### Message Format

```python
# sensor_msgs/JointState
msg.header.stamp  # Timestamp
msg.name          # ['joint1', 'joint2', ..., 'jointN']
msg.position      # [0-255, ...] Motor position values
msg.velocity      # [255, ...] Velocity values
```

## File Structure

```
motion/linkereg/
├── __init__.py       # Module export
├── linkeregcore.py   # Serial communication core
├── retarget.py       # ROS2 integration
└── README.md        # English documentation
```

## Notes

1. **Robot hand model**: No need to specify in config file, glove automatically identifies protocol type in data frame

2. **Hardware connection**: Receiver must be connected to host (where the program runs)

3. **Data rate**: Glove pushes data at 50Hz

4. **Mode selection**:
   - `linkereg2` (SDK control mode): Receiver doesn't need robot hand connection, glove data publishes to ROS topics, upper-layer robot hand SDK subscribes to control robot hand
   - `linkereg1` (Receiver control mode): Receiver must be connected to real robot hand, receiver directly controls robot hand motion, current SDK is only for collecting glove data.
