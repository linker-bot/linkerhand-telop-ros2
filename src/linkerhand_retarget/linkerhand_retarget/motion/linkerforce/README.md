# LinkerFFG Robot Hand Driver Module

LinkerFFG (O6/L7/L10/G20/O20/O30/O30i/R20/L25) robot hand ROS2 driver module, controls robot hands via serial port with real-time data glove mapping.

---

## Quick Start (Step-by-Step Guide)

### Step 1: Install

```bash
# From the linkerhand-telop-ros2 repository root
colcon build --symlink-install
source install/setup.bash
```

### Step 2: Connect Serial Port

Connect LinkerFFG robot hand to your PC via USB, then verify serial port permissions:

```bash
# Add current user to dialout group (requires re-login)
sudo usermod -a -G dialout $USER

# List serial devices
ls -l /dev/ttyUSB*
```

### Step 3: Configure Robot Hand Model

Edit `config/base_config.yml` to set robot hand model:

```yaml
system:
  motion_type: linkerforce    # Data glove type: linkerforce (required)
  robotname_r: l25            # Right hand model: o6 / l7 / l10 / g20 / o20 / o30 / o30i / r20 / l25
  robotname_l: l25            # Left hand model

serial:
  auto_scan: false            # Enable auto serial scan
  baudrates: [2000000, 460800, 1000000, 921600]  # 2000000 recommended
  left:
    port: /dev/ttyUSB1        # Left hand serial port
    baudrate: 460800          # Wireless: 460800, Wired: 2000000
  right:
    port: /dev/ttyUSB0        # Right hand serial port
    baudrate: 460800          # Wireless: 460800, Wired: 2000000
```

### Step 4: Run

**Method 1: Run node directly**

```bash
ros2 run linkerhand_retarget handretarget
```

**Method 2: Specify serial port (without modifying config file)**

```bash
ros2 run linkerhand_retarget handretarget --ros-args \
  -p ports:='["/dev/ttyUSB0", "/dev/ttyUSB1"]'
```

### Step 5: Calibration (if needed)

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=auto_calibrate
```

During calibration, follow the current `show_fist` setting:

- `show_fist: true`: **Make fist** → **O-pose** → **Open hand**
- `show_fist: false`: **O-pose** → **Open hand**

Hold each gesture stable for about 5 seconds. The terminal progress bar advances to the next step after the current gesture is stable.

---

## Robot Hand Models

| Model | DOF | Description |
|-------|-----|-------------|
| O6 | 6 | 6-DOF basic model |
| L7 | 7 | 7-DOF (thumb with roll) |
| L10 | 10 | 10-DOF industrial model |
| G20 | 20 | 20-DOF industrial model |
| O20 | 20 | Independent 20-DOF O-series model |
| O30 | 20 | Independent O30 model initialized from the G20 template |
| O30i | 20 | Independent O30i model with 20 movable URDF joints, mapped from the G20 template |
| R20 | 20 | 20-DOF research model |
| L25 | 25 | 25-DOF full-featured model |

`robotname_r` and `robotname_l` in config must match the actual connected robot hand models.

### O20 Mapping Notes

O20 is an independent O-series model and does not share the G20 motor direction table.

- O20 open hand maps to motor value `0`.
- O20 fist maps to motor value `255`.
- O20 `opose` URDF pose targets remain configured as robot joint angles in `o20_config.py`.
- O20 thumb motor output is calibrated separately from the URDF pose target: `thumb_cmc_roll` outputs about `165` at opose, and `thumb_cmc_yaw` outputs about `138` at opose.
- Calibration anchor inputs (`original`, `opose`, `fist`) are returned as exact URDF pose targets before live filtering is applied.

---

## Serial Connection Details

### Method 1: Command-line Port List (most common)

Suitable when multiple serial ports exist, system auto-detects left/right hand:

```bash
ros2 run linkerhand_retarget handretarget --ros-args \
  -p ports:='["/dev/ttyUSB0", "/dev/ttyUSB1"]'
```

### Serial Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `ports` | Candidate port list | empty (use config file) |
| `baudrate` | Specified baudrate (overrides config) | use config file |
| `auto_scan` | Auto scan when preset fails | `false` |

---

## Force Feedback

LinkerFFG subscribes to tactile matrix topics and converts the peak value of each finger matrix into five force-feedback values sent back to the glove:

| Input Topic | Hand | Data Requirement |
|-------------|------|------------------|
| `/cb_left_hand_matrix_touch` | Left | JSON string containing five finger tactile matrices |
| `/cb_right_hand_matrix_touch` | Right | JSON string containing five finger tactile matrices |

The matrix field names are fixed:

- `thumb_matrix`
- `index_matrix`
- `middle_matrix`
- `ring_matrix`
- `little_matrix`

Each finger feedback value is calculated as:

```text
max_force = min(max(matrix) * 4, 500)
```

The value `500` is the force-feedback ratio ceiling and corresponds to 50% torque output. It does not mean 500 N. The driver packs the five `max_force` values into `forcelist` and sends them to the glove through the LinkerFFG force-feedback frame.

Using a 4.8 V servo torque of `3.5 kg·cm` and a `10 cm` lever arm:

```text
Output force = 3.5 kg·cm * 50% / 10 cm
             = 0.175 kgf
             ≈ 0.18 kgf
             ≈ 1.8 N
```

So when the tactile matrix peak reaches `500` after conversion, the current feedback path drives about 50% torque, which is roughly `1.8 N` at the fingertip under this estimate. Actual perceived force depends on glove mechanics, friction, strap tightness, supply voltage, and tactile matrix calibration. During field tuning, start from lower tactile input or a lower feedback ratio.

---

## Calibration Details

### Calibration Config

In `config/base_config.yml`:

```yaml
calibration:
  show_fist: true       # Whether to show fist calibration step
  fist_extend_ratio: 0.5  # Fist extend ratio (only effective when show_fist=false)
```

### Start Calibration

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=auto_calibrate
```

### Calibration Process

The current calibration order depends on `show_fist`:

| Config | Calibration Order |
|--------|-------------------|
| `show_fist: true` | Make fist → O-pose → Open hand |
| `show_fist: false` | O-pose → Open hand |

Each step requires holding the gesture stable for about 5 seconds. When `show_fist: false`, real fist data is not collected; the fist anchor is calculated from the configured extension ratio after calibration.

For most existing models, open and fist follow the model-specific command table in `hand_config.yml`. For O20, open is motor `0` and fist is motor `255`.

### Difference between show_fist=true and show_fist=false

| Item | `show_fist: true` | `show_fist: false` |
|------|-------------------|-------------------|
| Calibration steps | Fist → O-pose → Open (3 steps) | O-pose → Open (2 steps) |
| Fist data source | User actually performs fist gesture | Calculated from O-pose by ratio |
| Fist formula | N/A | `fist = opose + (opose - original) × fist_extend_ratio` |
| Mapping precision | Three-segment linear interpolation, most accurate | Two-segment interpolation, relies on extension |
| fist_extend_ratio | Not used | Controls extension ratio (default 0.5) |

### fist_extend_ratio Details

Only effective when `show_fist: false`, used to calculate fist value from O-pose:

- `fist_extend_ratio = 0.5` (default): O-pose extends 50% toward fist
- `fist_extend_ratio = 0.0`: fist value = O-pose value (no extension)
- `fist_extend_ratio = 1.0`: fist value = O-pose + full (O-pose - Open) extension
- Recommended range: `0.3 ~ 0.7`, adjust based on actual results

### Stability Detection

- Auto-detects gesture stability (variance < 0.03)
- Requires **5 seconds continuous stability** to complete
- Resets on instability, no timeout limit
- Progress bar shows real-time stability duration

### Calibration Data Storage

- Location: `motion/linkerforce/tmp/jointangle_data.tmp`
- Format: JSON
- Auto-loads preset sample data on first use
- Re-calibration overwrites old data

---

## Topic Parameter Control (Runtime Dynamic Adjustment)

Dynamically adjust parameters via `/hand_teleop_param` topic without restarting the node.

### Adjustable Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `mapper_debug` | Mapper debug switch | `true` / `false` / `["thumb_rotate"]` |
| `mapper_exp_factor` | Extrapolation factor | `2.0` (global) or `{"thumb_rotate": 2.0}` (per finger) |
| `mapper_scale_factor` | Scale factor | `1.5` (global) or `{"index_root_flexion": 1.5}` (per finger) |
| `force_glove_pose` | Force glove data source | `open` / `fist` / `opose` / `none` |

### Usage Examples

```bash
# Enable mapper debug for all fingers
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": true}"}'

# Debug specific thumb fingers
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": [\"thumb_rotate\", \"thumb_abduction\"]}"}'

# Disable debug
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": false}"}'

# Adjust global extrapolation factor (higher = faster to target)
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_exp_factor\": 2.0}"}'

# Adjust per-finger extrapolation factor
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_exp_factor\": {\"thumb_rotate\": 2.0, \"index_root_flexion\": 1.5}}"}'

# Adjust global scale factor
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_scale_factor\": 1.5}"}'

# Use calibration data instead of glove data (for testing)
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"opose\"}"}'

# Restore real-time glove data
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"none\"}"}'
```

### Available Finger Names

| Finger | Names |
|--------|-------|
| Thumb | `thumb_rotate`, `thumb_abduction`, `thumb_root_flexion`, `thumb_end_flexion` |
| Index | `index_roll`, `index_root_flexion`, `index_end_flexion` |
| Middle | `middle_roll`, `middle_root_flexion`, `middle_end_flexion` |
| Ring | `ring_roll`, `ring_root_flexion`, `ring_end_flexion` |
| Pinky | `pinky_roll`, `pinky_root_flexion`, `pinky_end_flexion` |

---

## Mapping Parameters Details

### Extrapolation Factor (exp_factor)

Controls mapping extrapolation speed to target pose:
- `= 1.0`: Linear extrapolation
- `> 1.0`: Accelerated (faster to target)
- `< 1.0`: Decelerated (smoother)

### Scale Factor (scale_factor)

Controls input-to-output mapping ratio:
- `= 1.0`: 1:1 mapping
- `> 1.0`: Amplified output range
- `< 1.0`: Reduced output range

### Force Glove Data Source (force_glove_pose)

Use calibration data instead of real-time glove data for testing:

| Value | Description |
|-------|-------------|
| `open` | Use open hand calibration data |
| `fist` | Use fist calibration data |
| `opose` | Use O-pose calibration data |
| `none` | Use real-time glove data (default) |

---

## Configuration Details

See the "Configuration" section in the main README for full config reference. LinkerFFG-specific configs:

### system System Config

| Config | Description | Options |
|--------|-------------|---------|
| `motion_type` | Data glove type | `linkerforce` (required) |
| `robotname_r` | Right hand robot model | `o6`, `l7`, `l10`, `g20`, `o20`, `o30`, `o30i`, `r20`, `l25` |
| `robotname_l` | Left hand robot model | same as above |
| `retargeting_type` | Retargeting type | `projection` |

### calibration Calibration Config

| Config | Description | Default |
|--------|-------------|---------|
| `show_fist` | Show fist calibration step | `true` |
| `fist_extend_ratio` | Fist extend ratio | `0.5` |

### debug Debug Config

| Config | Description | Default |
|--------|-------------|---------|
| `mapper_debug` | Mapper debug switch | `false` |
| `joint_motor_debug_r` | Right hand joint motor debug | `false` |
| `joint_motor_debug_l` | Left hand joint motor debug | `false` |

---

## Troubleshooting

### Cannot open serial port

```bash
# Check serial permissions
ls -l /dev/ttyUSB*
# Grant permissions
sudo chmod 666 /dev/ttyUSB0
```

### Robot hand not responding (Key Topic Monitoring)

Follow these steps in order:

**Step 1: Check if topics are published**

```bash
# List all related topics
ros2 topic list | grep cb_

# Check if data is being output (is the node publishing normally)
ros2 topic echo /cb_right_hand_control_cmd --once
ros2 topic echo /cb_left_hand_control_cmd --once
```

**Step 2: Check glove data topics**

```bash
# LinkerFFG driver does not publish glove data topics - this step can be skipped
# To verify data source, check if /cb_right_hand_control_cmd has data output
ros2 topic echo /cb_right_hand_control_cmd --once
```

**Step 3: Check LinkerFFG joint control topics**

```bash
# Check if joint control commands have output
ros2 topic echo /cb_right_hand_control_cmd --once
ros2 topic echo /cb_left_hand_control_cmd --once

# Check topic frequency (should be around 50Hz)
ros2 topic hz /cb_right_hand_control_cmd
```

**Step 4: Enable debug output**

```bash
# Enable all debug
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": true}"}'

# Observe terminal output, check if glove data is changing
# If data doesn't change, glove is not connected or topic is not published
```

**Step 5: Check calibration data**

```bash
# Check current calibration data in use
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"opose\"}"}'
# Observe if robot hand responds

ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"fist\"}"}'
# Observe if robot hand grips

ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"none\"}"}'
# Restore normal data source
```

**Step 6: Check serial connection**

1. Verify baudrate matches robot hand settings (wireless: 460800, wired: 2000000)
2. Try `auto_scan: true` for auto detection
3. Check robot hand power supply

**Step 7: Check logs**

```bash
# View node logs
ros2 run linkerhand_retarget handretarget
# Observe debug info in terminal output
```

**Quick Problem Identification**

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| No topic data | Glove not connected or topic name wrong | Check glove connection, verify topic name |
| Data always 0 or 255 | Calibration data abnormal | Re-calibrate or delete `tmp/jointangle_data.tmp` |
| Data fluctuates wildly | Serial signal interference | Check serial cable, use shielded cable |
| No terminal output | Node didn't start successfully | Check for errors, verify dependencies installed |
| Topic has data but robot hand doesn't move | Robot hand SDK not receiving commands | Check SDK connection, verify topic is subscribed |

---

## File Structure

```
motion/linkerforce/
├── config/              # Hand model configurations
│   ├── o6_config.py     # O6 config
│   ├── l6_config.py     # L6 config
│   ├── l7_config.py     # L7 config
│   ├── l10_config.py    # L10 config
│   ├── l20_config.py    # L20 config
│   ├── g20_config.py    # G20 config
│   ├── o20_config.py    # O20 config
│   ├── o30_config.py    # O30 config
│   ├── o30i_config.py   # O30i config
│   └── o7_config.py     # O7 config
├── hand/                # Robot hand drivers
│   ├── linkerforce_o6.py
│   ├── linkerforce_l6.py
│   ├── linkerforce_l7.py
│   ├── linkerforce_l10.py
│   ├── linkerforce_l20.py
│   ├── linkerforce_g20.py
│   ├── linkerforce_o20.py
│   ├── linkerforce_o30.py
│   └── linkerforce_o30i.py
├── tmp/                 # Temp files (calibration data, etc.)
├── retarget.py          # ROS integration
└── README.md           # English documentation
```
