# LinkerMCG_G7 数据手套模块

LinkerMCG_G7 数据手套的 ROS2 驱动模块，通过 UDP 接收手套数据并发布到 ROS2 话题。

## 特性

- 通过 UDP 协议接收手套数据
- 支持左右手数据
- 发布频率 50Hz

## 使用方法

### 1. 配置文件

修改 `config/base_config.yml`:

```yaml
system:
  motion_type: linkermcg_g7

udp:
  ip: "<目标地址>"
  port: 9011
```

`udp.ip` 必须填写接收 UDP 数据的目标主机地址，不要使用 `0.0.0.0`。

旧配置值 `motion_type: linkermcg` 仍作为 `linkermcg_g7` 的兼容别名保留。

**重要：** 必须同时打开 `debug.joint_motor_debug_l` 和 `debug.joint_motor_debug_r`，调试输出中才会打印 UDP 数据。

### 2. 运行

```bash
ros2 run linkerhand_retarget handretarget
```

## ROS2 话题

### 发布话题

| 话题名 | 消息类型 | 说明 |
|-------|---------|------|
| `/cb_right_hand_control_cmd` | `sensor_msgs/JointState` | 右手控制数据 |
| `/cb_left_hand_control_cmd` | `sensor_msgs/JointState` | 左手控制数据 |

### 消息格式

```python
# sensor_msgs/JointState
msg.header.stamp  # 时间戳
msg.name          # 关节名称列表
msg.position      # 关节位置值
```

## 文件结构

```
motion/linkermcg/
├── __init__.py
├── retarget.py      # ROS2 集成
└── README.md        # 本文档
```

## 注意事项

1. 确保 UDP 端口未被占用
2. 手套与主机需在同一网络
3. 只有同时打开 `debug.joint_motor_debug_l` 和 `debug.joint_motor_debug_r`，才会在调试输出中显示来自 UDP 的数据
