# VTR-DYN 数据手套模块

VTR-DYN 数据手套的 ROS2 驱动模块，通过 UDP 接收手套数据并发布到 ROS2 话题。

## 特性

- 通过 UDP 协议接收手套数据
- 支持左右手数据
- 发布频率 50Hz

## 使用方法

### 1. 配置文件

修改 `config/base_config.yml`:

```yaml
system:
  motion_type: vtrdyn

udp:
  ip: "0.0.0.0"
  port: 8888
```

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
motion/vtrdyn/
├── __init__.py
├── vtrdyncore.py    # UDP 通讯核心
├── retarget.py      # ROS2 集成
└── README.md        # 本文档
```

## 注意事项

1. 确保 UDP 端口未被占用
2. 手套与主机需在同一网络
