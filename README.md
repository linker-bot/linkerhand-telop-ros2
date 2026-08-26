# LinkerHand ROS2 Teleop SDK

ROS2-only teleoperation and motion retargeting SDK for LinkerHand robot hands and supported data gloves.

## Project Scope

This repository contains the standalone ROS2 source workspace for LinkerHand teleoperation:

- ROS2 node entry point and package metadata
- Motion retargeting adapters for supported data gloves
- Robot hand configuration files
- Robot model assets
- Offline unit tests and integration test examples

Current SDK version: `2.12.10`. See [VERSION.md](VERSION.md) for release notes.

<p style="color: #b7791f;"><em>Warning: This release uses a source-workspace delivery path. After <code>colcon build --symlink-install</code>, configuration and robot model assets are still read from the package source tree, so keep the workspace layout shown below. Wheel-style installation or copying only the <code>install/</code> directory is not supported for this release.</em></p>

## Supported Devices

### Data Gloves and Robot Hand Models

| Data Glove | O6 | L6 | L7 | L10/L10v7 | L20 | L21/L25 | G20 | O20 | O30 | Module | Docs |
|------------|----|----|----|-----------|-----|---------|-----|-----|-----|--------|------|
| LinkerFFG | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | `motion/linkerforce` | [README](src/linkerhand_retarget/linkerhand_retarget/motion/linkerforce/README.md) |
| VTR-DYN | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | - | `motion/vtrdyn` | [README](src/linkerhand_retarget/linkerhand_retarget/motion/vtrdyn/README.md) |
| LinkerTG / UdexReal | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | - | `motion/udexreal` | [README](src/linkerhand_retarget/linkerhand_retarget/motion/udexreal/README.md) |
| LinkerMCG | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | - | `motion/linkermcg` | [README](src/linkerhand_retarget/linkerhand_retarget/motion/linkermcg/README.md) |
| LinkerEG | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | - | `motion/linkereg` | [README](src/linkerhand_retarget/linkerhand_retarget/motion/linkereg/README.md) |

LinkerMCG M7 adds the shared M-series UDP path for O20. LinkerMCG M11 extends that path with the dedicated O20 16-dof schema and O30 support.

## ROS2 Compatibility

| ROS2 Version | Python | x86_64 | ARM64 | Windows |
|--------------|--------|--------|-------|---------|
| Foxy | 3.8 | ✅ | ✅ | ✅ |
| Humble | 3.10 | ✅ | ✅ | ✅ |
| Jazzy | 3.12 | ✅ | ✅ | ✅ |

## Workspace Layout

```text
linkerhand-telop-ros2/
├── src/
│   ├── requirements.txt
│   └── linkerhand_retarget/
│       ├── package.xml
│       ├── setup.py
│       ├── linkerhand_retarget/
│       │   ├── handretarget.py
│       │   ├── linkerhand/
│       │   ├── motion/
│       │   ├── config/
│       │   ├── assets/
│       │   └── launch/
│       └── tests/
├── run_linkerhand_teleop.sh
├── README.md
└── README_zh.md
```

`launch/` is kept as source reference only. This repository runs through `ros2 run`; launch-file installation is not used.

## Quick Start

Install Python dependencies as needed:

```bash
python3 -m pip install -r src/requirements.txt
```

Build from the repository root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Run the ROS2 node:

```bash
ros2 run linkerhand_retarget handretarget
```

Run with calibration:

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=True
```

For LinkerFFG serial devices, confirm the USB device names and grant permissions when needed:

```bash
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

The helper script `run_linkerhand_teleop.sh` is provided for repository-level teleoperation startup. It uses the current `base_config.yml` motion source and runs without calibration by default. Its serial permission handling is only for the LinkerFFG module; other modules should use their own network, serial, or device setup described in the module documentation.

## Configuration

Main configuration file:

```text
src/linkerhand_retarget/linkerhand_retarget/config/base_config.yml
```

### `system`

| Config | Description | Options |
|--------|-------------|---------|
| `motion_type` | Data glove type | `linkerforce`, `vtrdyn`, `udexreal`, `udexrealv2t`, `linkermcg`, `linkermcg_m7`, `linkermcg_m11`, `linkereg1`, `linkereg2` |
| `datasource_type` | Data source type | `motion` |
| `retargeting_type` | Retargeting type | `projection` |
| `robotname_r` | Right hand robot model | `o6`, `l6`, `l7`, `l10`, `l10v7`, `l20`, `l21`, `l25`, `g20`, `o20`, `o30` (`o20` is supported by `motion_type: linkerforce`, `linkermcg_m7`, or `linkermcg_m11`; `o30` is supported by `linkerforce` or `linkermcg_m11`) |
| `robotname_l` | Left hand robot model | same as above |
| `motion_device` | Motion device ID, used by selected motion sources | e.g. `eric` |

### `serial` - LinkerFFG

| Config | Description | Default |
|--------|-------------|---------|
| `auto_scan` | Auto scan serial ports | `false` |
| `baudrates` | Candidate baudrate list | `[2000000, 460800, 1000000, 921600]` |
| `left.port` / `right.port` | Left/right hand serial path | e.g. `/dev/ttyUSB0` |
| `left.baudrate` / `right.baudrate` | Left/right hand baudrate | `460800` |
| `exclude_ports` | Ports to exclude | `[]` |
| `serial_debug` | Serial debug switch | `false` |

### `udp` - LinkerTG / LinkerMCG / VTR-DYN

| Config | Description | Default |
|--------|-------------|---------|
| `ip` | Glove UDP server IP | `192.168.11.88` |
| `port` | Glove UDP port | `8888` |
| `serverport` | Local listening port | `5551` |

### `mujoco` - Optional Display

| Config | Description | Default |
|--------|-------------|---------|
| `enabled` | Start the MuJoCo viewer during SDK startup | `false` |
| `hands` | Hand viewers to start. `auto` follows the hands loaded by the selected motion module when available; set `[right]`, `[left]`, or `[right, left]` to override. | `auto` |
| `fps` | Viewer update rate setting reserved for display integration | `30` |

The display uses the same robot mapping as the SDK and loads URDF paths from the selected `robotname_r` and `robotname_l`. With `hands: auto`, the SDK starts one or two MuJoCo viewer instances after the motion module initializes and reports the loaded hands. MuJoCo is an optional Python dependency; if `mujoco`, `mujoco.viewer`, or a mapped URDF file is unavailable, startup prints a warning for that hand and the SDK continues.

### `calibration` - LinkerFFG

| Config | Description | Default |
|--------|-------------|---------|
| `show_fist` | Show fist calibration | `true` |
| `fist_extend_ratio` | Fist extend ratio | `0.5` |

### `debug`

| Config | Description | Default |
|--------|-------------|---------|
| `mapper_debug` | Mapper debug switch | `false` |
| `joint_motor_debug_r` | Right hand joint debug | `false` |
| `joint_motor_debug_l` | Left hand joint debug | `false` |
| `joint_pub_debug` | Topic publish debug | `false` |

### LinkerEG

```yaml
linkereg:
  port: null
  password: "i"
```

## Example Configuration

```yaml
system:
  motion_type: linkerforce
  robotname_r: l25
  robotname_l: l25

serial:
  auto_scan: false
  baudrates: [2000000, 460800, 1000000, 921600]
  left:
    port: /dev/ttyUSB1
    baudrate: 460800
  right:
    port: /dev/ttyUSB0
    baudrate: 460800
```

## ROS2 Topics

Common output topics:

```bash
ros2 topic echo /cb_left_hand_control_cmd
ros2 topic echo /cb_right_hand_control_cmd
```

Some modules publish or subscribe to additional topics. See the module documentation linked in [Supported Devices](#supported-devices).

## Tests

Offline unit tests can be run from the repository root:

```bash
PYTHONPATH=src/linkerhand_retarget python3 -m pytest src/linkerhand_retarget/tests/unit -q
```

Integration tests may require hardware, serial devices, network devices, or a full ROS2 runtime setup.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
