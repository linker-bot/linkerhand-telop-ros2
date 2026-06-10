# LinkerFFG 机械手驱动模块

LinkerFFG (O6/L7/L10/G20/O20/O30/R20/L25) 机械手的 ROS2 驱动模块，通过串口控制机械手，支持数据手套实时映射。

---

## 快速入门
### 步骤 1：安装

```bash
# 从 linkerhand-telop-ros2 仓库根目录执行
colcon build --symlink-install
source install/setup.bash
```

### 步骤 2：连接串口

将 LinkerFFG 机械手通过 USB 连接到电脑，确认串口权限：

```bash
# 添加当前用户到 dialout 组（需要重新登录生效）
sudo usermod -a -G dialout $USER

# 查看串口设备
ls -l /dev/ttyUSB*
```

### 步骤 3：配置机械手型号

编辑 `config/base_config.yml`，设置机械手型号：

```yaml
system:
  motion_type: linkerforce    # 数据手套类型：linkerforce（必须）
  robotname_r: l25            # 右手机械手型号：o6 / l7 / l10 / g20 / o20 / o30 / r20 / l25
  robotname_l: l25            # 左手机械手型号

serial:
  auto_scan: false            # 是否自动扫描串口
  baudrates: [2000000, 460800, 1000000, 921600]  # 波特率列表，2000000 优先
  left:
    port: /dev/ttyUSB1        # 左手套接的串口
    baudrate: 460800          # 无线460800  有线2000000
  right:
    port: /dev/ttyUSB0        # 右手套接的串口
    baudrate: 460800          # 无线460800  有线2000000
```

### 步骤 4：启动

**方式一：直接运行节点**

```bash
ros2 run linkerhand_retarget handretarget
```

**方式二：指定串口启动（不修改配置文件）**

```bash
ros2 run linkerhand_retarget handretarget --ros-args \
  -p ports:='["/dev/ttyUSB0", "/dev/ttyUSB1"]'
```

### 步骤 5：标定（如需要）

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=auto_calibrate
```

标定时按照提示做三个动作：
1. **五指张开** → 保持 5 秒
2. **握紧拳头** → 保持 5 秒
3. **O 型手势** → 保持 5 秒

---

## 机械手型号说明

| 型号 | 关节数 | 说明 |
|------|--------|------|
| O6 | 6 | 6 自由度基础款 |
| L7 | 7 | 7 自由度（拇指增加横滚） |
| L10 | 10 | 10 自由度工业款 |
| G20 | 20 | 20 自由度工业款 |
| O20 | 20 | O 系列独立 20 自由度型号 |
| O30 | 20 | 基于 G20 模板初始化的 O30 独立型号 |
| L25 | 25 | 25 自由度全功能款 |

配置中的 `robotname_r` 和 `robotname_l` 必须与实际连接的机械手型号匹配。

### O20 映射说明

O20 是 O 系列独立型号，不共用 G20 的电机方向表。

- O20 五指张开对应电机值 `0`。
- O20 握拳对应电机值 `255`。
- O20 `opose` 的 URDF 姿态目标仍按 `o20_config.py` 中的机器人关节角配置。
- O20 拇指电机输出与 URDF 姿态目标分层标定：`thumb_cmc_roll` 在 opose 时约输出 `165`，`thumb_cmc_yaw` 在 opose 时约输出 `138`。
- `original`、`opose`、`fist` 标定锚点输入会在实时滤波前直接返回对应 URDF 姿态目标，保证锚点姿态准确。

---

## 串口参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `ports` | 候选串口列表 | 空（使用配置文件） |
| `baudrate` | 指定波特率（优先级高于配置） | 使用配置文件 |
| `auto_scan` | 预设失败后是否自动扫描 | `false` |

---

## 自动标定详解

### 标定配置

在 `config/base_config.yml` 中：

```yaml
calibration:
  show_fist: true       # 是否显示握拳标定步骤
  fist_extend_ratio: 0.5  # 握拳延伸比例（仅 show_fist=false 时生效）
```

### 启动标定

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=auto_calibrate
```

### 标定流程

1. **五指张开** → 保持 5 秒
2. **O 型手势** → 保持 5 秒

如果 `show_fist: true`，还会出现第三步：

3. **握紧拳头** → 保持 5 秒

多数已有型号的张开/握拳输出方向由 `hand_config.yml` 中的型号电机表决定。O20 单独定义为张开输出 `0`，握拳输出 `255`。

### show_fist=true 与 show_fist=false 的区别

| 对比项 | `show_fist: true` | `show_fist: false` |
|--------|-------------------|-------------------|
| 标定步骤 | 张开 → O 型 → 握拳（共 3 步） | 张开 → O 型（共 2 步） |
| 握拳数据来源 | 用户实际做握拳动作采集 | 从 O 型按比例延伸计算 |
| 握拳延伸公式 | 无 | `fist = opose + (opose - original) × fist_extend_ratio` |
| 映射精度 | 三段线性插值，最精确 | 两段插值，依赖延伸估算 |
| fist_extend_ratio | 不生效 | 控制延伸比例（默认 0.5） |

### fist_extend_ratio 详解

仅在 `show_fist: false` 时生效，用于从 O 型自动计算握拳值：

- `fist_extend_ratio = 0.5`（默认）：O 型向握拳方向延伸 50%
- `fist_extend_ratio = 0.0`：握拳值 = O 型值（无延伸）
- `fist_extend_ratio = 1.0`：握拳值 = O 型 + (O 型 - 张开) 的全量延伸
- 推荐值范围：`0.3 ~ 0.7`，需要根据实际效果调整

### 稳定性检测

- 标定时自动检测手势稳定性（方差 < 0.03）
- 需**连续稳定 5 秒**才完成采集
- 不稳定时清空重来，无超时限制
- 终端显示进度条，实时反馈稳定时长

### 标定数据存储

- 存储位置：`motion/linkerforce/tmp/jointangle_data.tmp`
- 格式：JSON
- 首次使用自动加载内置样本数据
- 重新标定会覆盖旧数据

---

## 话题参数控制（运行时动态调整）

通过 `/hand_teleop_param` 话题动态调整运行参数，无需重启节点。

### 可调参数

| 参数 | 说明 | 示例值 |
|------|------|--------|
| `mapper_debug` | 映射器调试开关 | `true` / `false` / `["thumb_rotate"]` |
| `mapper_exp_factor` | 延伸指数因子 | `2.0`（全局）或 `{"thumb_rotate": 2.0}`（单指） |
| `mapper_scale_factor` | 缩放因子 | `1.5`（全局）或 `{"index_root_flexion": 1.5}`（单指） |
| `force_glove_pose` | 强制手套数据源 | `open` / `fist` / `opose` / `none` |

### 使用示例

```bash
# 开启全部手指的映射器调试
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": true}"}'

# 只调试拇指相关手指
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": [\"thumb_rotate\", \"thumb_abduction\"]}"}'

# 关闭调试
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": false}"}'

# 调整全局延伸指数（值越大到达目标越快）
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_exp_factor\": 2.0}"}'

# 调整单指延伸指数
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_exp_factor\": {\"thumb_rotate\": 2.0, \"index_root_flexion\": 1.5}}"}'

# 调整全局缩放因子
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_scale_factor\": 1.5}"}'

# 用标定数据替代手套数据（用于测试）
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"opose\"}"}'

# 恢复实时手套数据
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"none\"}"}'
```

### 可用手指名称

| 手指 | 参数名称 |
|------|----------|
| 拇指 | `thumb_rotate`, `thumb_abduction`, `thumb_root_flexion`, `thumb_end_flexion` |
| 食指 | `index_roll`, `index_root_flexion`, `index_end_flexion` |
| 中指 | `middle_roll`, `middle_root_flexion`, `middle_end_flexion` |
| 无名指 | `ring_roll`, `ring_root_flexion`, `ring_end_flexion` |
| 小指 | `pinky_roll`, `pinky_root_flexion`, `pinky_end_flexion` |

---

## 映射参数详解

### 延伸指数因子 (exp_factor)

控制映射延伸到目标姿态的速度：
- `= 1.0`：线性延伸
- `> 1.0`：加速延伸（更快到达目标）
- `< 1.0`：减速延伸（更平滑）

### 缩放因子 (scale_factor)

控制输入到输出的映射比例：
- `= 1.0`：1:1 映射
- `> 1.0`：放大输出范围
- `< 1.0`：缩小输出范围

### 强制手套数据源 (force_glove_pose)

用标定数据替代实时手套数据，用于调试和测试：

| 值 | 说明 |
|-----|------|
| `open` | 使用五指张开标定数据 |
| `fist` | 使用握拳标定数据 |
| `opose` | 使用 O 型手势标定数据 |
| `none` | 使用实时手套数据（默认） |

---

## 配置文件详解

完整配置项见主 README 的「配置说明」章节。LinkerFFG 驱动专用配置：

### system 系统配置

| 配置项 | 说明 | 可选值 |
|--------|------|--------|
| `motion_type` | 数据手套类型 | `linkerforce`（必须） |
| `robotname_r` | 右手机械手型号 | `o6`, `l7`, `l10`, `g20`, `o20`, `o30`, `r20`, `l25` |
| `robotname_l` | 左手机械手型号 | 同上 |
| `retargeting_type` | 重定向类型 | `projection` |

### calibration 标定配置

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `show_fist` | 是否显示握拳标定步骤 | `true` |
| `fist_extend_ratio` | 握拳延伸比例 | `0.5` |

### debug 调试配置

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `mapper_debug` | 映射器调试开关 | `false` |
| `joint_motor_debug_r` | 右手关节电机调试 | `false` |
| `joint_motor_debug_l` | 左手关节电机调试 | `false` |

---

## 故障排除

### 串口无法打开

```bash
# 检查串口权限
ls -l /dev/ttyUSB*
# 添加权限
sudo chmod 666 /dev/ttyUSB0
```

### 机械手无反应（重点监测话题）

按以下顺序逐项检查：

**步骤 1：检查 LinkerFFG 输出话题**

```bash
# 查看 LinkerFFG 驱动发布的话题
ros2 topic list | grep cb_

# 查看是否有数据输出（机械手节点是否正常发布数据）
ros2 topic echo /cb_right_hand_control_cmd --once
ros2 topic echo /cb_left_hand_control_cmd --once
```

**步骤 2：检查手套数据话题**

```bash
# 查看手套数据是否到达（LinkerFFG 驱动不发布此话题，此步骤可跳过）
# 如需验证数据源，请检查 /cb_right_hand_control_cmd 是否有数据输出
ros2 topic echo /cb_right_hand_control_cmd --once
```

**步骤 3：检查 LinkerFFG 关节控制话题**

```bash
# 查看关节控制指令是否有输出
ros2 topic echo /cb_right_hand_control_cmd --once
ros2 topic echo /cb_left_hand_control_cmd --once

# 查看话题频率是否正常（应该 50Hz 左右）
ros2 topic hz /cb_right_hand_control_cmd
```

**步骤 4：启用调试打印**

```bash
# 开启全部调试
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"mapper_debug\": true}"}'

# 查看终端输出，是否有手套数据变化
# 如果数据不变，说明手套未连接或话题未发布
```

**步骤 5：检查标定数据**

```bash
# 查看当前使用的标定数据
ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"opose\"}"}'
# 观察机械手是否有反应

ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"fist\"}"}'
# 观察机械手是否握紧

ros2 topic pub --once /hand_teleop_param std_msgs/msg/String '{"data": "{\"force_glove_pose\": \"none\"}"}'
# 恢复正常数据源
```

**步骤 6：检查串口连接**

1. 确认波特率配置与机械手一致（无线 460800，有线 2000000）
2. 尝试使用 `auto_scan: true` 自动检测
3. 检查机械手电源是否正常

**步骤 7：检查日志**

```bash
# 查看节点日志
ros2 run linkerhand_retarget handretarget
# 观察终端输出的 debug 信息
```

**常见问题快速定位**

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 话题无数据 | 手套未连接或话题名错误 | 检查手套连接，确认话题名 |
| 数据一直是 0 或 255 | 标定数据异常 | 重新标定或删除 `tmp/jointangle_data.tmp` |
| 数据跳变剧烈 | 串口信号干扰 | 检查串口线，使用屏蔽线 |
| 终端无输出 | 节点未启动成功 | 检查是否报错，检查依赖是否安装 |
| 话题有数据但机械手不动 | 机械手SDK未收到指令 | 检查机械手SDK连接，确认话题被正确订阅 |

---

## 文件结构

```
motion/linkerforce/
├── config/              # 手型配置文件
│   ├── o6_config.py     # O6 手型配置
│   ├── l6_config.py     # L6 手型配置
│   ├── l7_config.py     # L7 手型配置
│   ├── l10_config.py    # L10 手型配置
│   ├── l20_config.py    # L20 手型配置
│   ├── g20_config.py    # G20 手型配置
│   ├── o20_config.py    # O20 手型配置
│   ├── o30_config.py    # O30 手型配置
│   └── o7_config.py     # O7 手型配置
├── hand/                # 机械手驱动
│   ├── linkerforce_o6.py
│   ├── linkerforce_l6.py
│   ├── linkerforce_l7.py
│   ├── linkerforce_l10.py
│   ├── linkerforce_l20.py
│   ├── linkerforce_g20.py
│   ├── linkerforce_o20.py
│   └── linkerforce_o30.py
├── tmp/                 # 临时文件（标定数据等）
├── retarget.py          # ROS 集成层
└── README.md            # 本文档
```
