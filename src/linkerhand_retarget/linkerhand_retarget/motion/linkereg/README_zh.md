# LinkerEG 遥操作手套模块

LinkerEG 遥操作手套的 ROS2 驱动模块，通过串口接收手套数据并发布到 ROS2 话题。

## 更新日志

### 2026-02-04
- 修复了初始化流程中的死循环 bug
- 增加了对 G20 手的支持

## 特性

- **发布频率**: 50Hz (所有话题统一频率)
- 自动扫描串口连接手套
- 支持左右手数据
- 支持传感器原始数据推送 (可通过话题开关)

## 控制模式

| motion_type | 模式 | 说明 |接收器是否需要连接灵巧手 |
|-------------|------|------|-------------------|
| `linkereg2` | SDK控制模式 | 手套数据直接发布到ROS话题，由SDK控制灵巧手 | ❌ 不需要 |
| `linkereg1` | 接收器控制模式 | 接收器直接控制灵巧手，同时发布数据到ROS话题 | ✅ 必须连接 |

## 使用方法

### 1. 配置文件

修改 `config/base_config.yml`:

```yaml
system:
  # SDK控制模式 (接收器不需要连接灵巧手)
  motion_type: linkereg2
  
  #或 接收器控制模式 (接收器必须连接灵巧手)
  motion_type: linkereg1
```

#### 串口权限密码

如果需要自动修复串口权限，请在 `linkereg` 下配置 `password`（sudo 密码）。
注意：YAML 里不加引号会被解析成数字，建议加引号以确保是字符串。

```yaml
linkereg:
  port: null
  password: "123456"  # sudo密码，用于修复串口权限，如果密码全是数字 需要加引号，比如 "123456"
```

### 2. 运行

```bash
ros2 run linkerhand_retarget handretarget
```

### 3. 启用调试打印

修改 `config/base_config.yml`:

```yaml
debug:
  joint_pub_debug: true
```

## ROS2 话题

> **发布频率**: 所有话题统一 50Hz

### 发布话题

| 话题名 | 消息类型 | 说明 | 默认状态 |
|-------|---------|------|---------|
| `/cb_right_hand_control_cmd` | `sensor_msgs/JointState` | 右手控制数据 | ✅ 启用 |
| `/cb_left_hand_control_cmd` | `sensor_msgs/JointState` | 左手控制数据 | ✅ 启用 |
| `/cb_right_hand_raw_data` | `sensor_msgs/JointState` | 右手传感器原始数据 | ❌ 禁用 |
| `/cb_left_hand_raw_data` | `sensor_msgs/JointState` | 左手传感器原始数据 | ❌ 禁用 |

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|-------|---------|------|
| `/cb_hand_setting_cmd` | `std_msgs/String` | 设置命令 (控制原始数据开关) |

## 原始数据功能

原始数据默认**禁用**，需要通过话题命令启用。

### 启用/禁用原始数据

```bash
# 启用原始数据推送
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/String "data: 'on'"

# 禁用原始数据推送  
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/String "data: 'off'"

# 监听原始数据
ros2 topic echo /cb_left_hand_raw_data
```

## ROS2 话题数据格式

### 原始数据格式 (15个int32值)

| 索引 | 关节名称 | 索引 | 关节名称 | 索引 | 关节名称 |
|------|----------|------|----------|------|----------|
| 0 | 大拇指横摆 | 1 | 大拇指弯曲 | 2 | 大拇指指尖 |
| 3 | 食指横摆 | 4 | 食指弯曲 | 5 | 食指指尖 |
| 6 | 中指横摆 | 7 | 中指弯曲 | 8 | 中指指尖 |
| 9 | 无名指横摆 | 10 | 无名指弯曲 | 11 | 无名指指尖 |
| 12 | 小指横摆 | 13 | 小指弯曲 | 14 | 小指指尖 |

### 控制数据格式
#### L21 (25关节输出)

适用灵巧手型号：L21

| Joint | 关节名称 | 说明 |
|-------|----------|------|
| joint1 | 大拇指弯曲 | |
| joint2 | 食指弯曲 | |
| joint3 | 中指弯曲 | |
| joint4 | 无名指弯曲 | |
| joint5 | 小拇指弯曲 | |
| joint6 | 大拇指横摆 | |
| joint7 | 食指横摆 | |
| joint8 | 中指横摆 | |
| joint9 | 无名指横摆 | |
| joint10 | 小拇指横摆 | |
| joint11 | 大拇指横滚 | |
| joint12 | 预留 | 值为0 |
| joint13 | 预留 | 值为0 |
| joint14 | 预留 | 值为0 |
| joint15 | 预留 | 值为0 |
| joint16 | 大拇指中部 | 值为0 |
| joint17 | 预留 | 值为0 |
| joint18 | 预留 | 值为0 |
| joint19 | 预留 | 值为0 |
| joint20 | 预留 | 值为0 |
| joint21 | 大拇指指尖 | |
| joint22 | 食指指尖 | |
| joint23 | 中指指尖 | |
| joint24 | 无名指指尖 | |
| joint25 | 小指指尖 | |

#### L20/G20 (20关节输出)

适用灵巧手型号：L20、工业版20、G20

| Joint | 关节名称 | 说明 |
|-------|----------|------|
| joint1 | 拇指弯曲 | |
| joint2 | 食指弯曲 | |
| joint3 | 中指弯曲 | |
| joint4 | 无名指弯曲 | |
| joint5 | 小指弯曲 | |
| joint6 | 拇指横摆 | |
| joint7 | 食指横摆 | |
| joint8 | 中指横摆 | |
| joint9 | 无名指横摆 | |
| joint10 | 小指横摆 | |
| joint11 | 拇指横滚 | |
| joint12 | 预留 | 值为0 |
| joint13 | 预留 | 值为0 |
| joint14 | 预留 | 值为0 |
| joint15 | 预留 | 值为0 |
| joint16 | 拇指指尖 | |
| joint17 | 食指指尖 | |
| joint18 | 中指指尖 | |
| joint19 | 无名指指尖 | |
| joint20 | 小指指尖 | |

#### L10 (10关节)

适用灵巧手型号：L10

| Joint | 关节名称 |
|-------|----------|
| joint1 | 大拇指弯曲 |
| joint2 | 大拇指横摆 |
| joint3 | 食指弯曲 |
| joint4 | 中指弯曲 |
| joint5 | 无名指弯曲 |
| joint6 | 小指弯曲 |
| joint7 | 食指横摆 |
| joint8 | 无名指横摆 |
| joint9 | 小指横摆 |
| joint10 | 大拇指横滚 |

#### L6 (6关节)

适用灵巧手型号：L6、O6

| Joint | 关节名称 |
|-------|----------|
| joint1 | 大拇指弯曲 |
| joint2 | 大拇指横摆 |
| joint3 | 食指弯曲 |
| joint4 | 中指弯曲 |
| joint5 | 无名指弯曲 |
| joint6 | 小指弯曲 |

#### O7 (7关节)

适用灵巧手型号：O7

| Joint | 关节名称 |
|-------|----------|
| joint1 | 大拇指弯曲 |
| joint2 | 大拇指横摆 |
| joint3 | 食指弯曲 |
| joint4 | 中指弯曲 |
| joint5 | 无名指弯曲 |
| joint6 | 小指弯曲 |
| joint7 | 大拇指横滚 |

### 消息格式

```python
# sensor_msgs/JointState
msg.header.stamp  # 时间戳
msg.name          # ['joint1', 'joint2', ..., 'jointN']
msg.position      # [0-255, ...] 电机位置值
msg.velocity      # [255, ...] 速度值
```

## 文件结构

```
motion/linkereg/
├── __init__.py       # 模块导出
├── linkeregcore.py   # 串口通讯核心
├── retarget.py       # ROS2 集成
└── README.md         # 本文档
```

## 注意事项

1. **灵巧手型号**: 无需在配置文件中指定，手套会在数据帧中自动标识协议类型

2. **硬件连接**：接收器必须连接到主机上（当前程序所在主机）

3. **数据频率**: 手套以 50Hz 频率推送数据

4. **模式选择**:
   - `linkereg2` (SDK控制模式): 接收器不需要连接灵巧手，手套数据通过ROS话题发布，由上层灵巧手SDK订阅对应话题来控制灵巧手
   - `linkereg1` (接收器控制模式): 接收器必须连接真实灵巧手，接收器直接控制灵巧手运动，当前SDK只是为了采集手套数据。