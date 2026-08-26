# LinkerHand ROS2 遥操作 SDK

仅面向 ROS2 的 LinkerHand 遥操作与运动重定向 SDK，支持多种数据手套和 LinkerHand 机械手型号。

## 项目范围

本仓库是独立 ROS2 源码工作区，包含：

- ROS2 节点入口和包元数据
- 多种数据手套的运动重定向适配模块
- 机械手配置文件
- 机器人模型资源
- 离线单元测试和集成测试示例

当前 SDK 版本：`2.12.9`。版本说明见 [VERSION.md](VERSION.md)。

<p style="color: #b7791f;"><em>警告：当前交付方式基于源码工作区。执行 <code>colcon build --symlink-install</code> 后，配置和机器人模型资源仍从包源码树读取，因此请保持下方工作区结构。标准 wheel 安装或仅复制 <code>install/</code> 目录不是本版本支持的部署方式。</em></p>

## 支持设备

### 数据手套与机械手型号

| 数据手套 | O6 | L6 | L7 | L10/L10v7 | L20 | L21/L25 | G20 | O20 | O30 | 模块 | 文档 |
|----------|----|----|----|-----------|-----|---------|-----|-----|-----|------|------|
| LinkerFFG | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | `motion/linkerforce` | [README_zh](src/linkerhand_retarget/linkerhand_retarget/motion/linkerforce/README_zh.md) |
| VTR-DYN | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | - | `motion/vtrdyn` | [README_zh](src/linkerhand_retarget/linkerhand_retarget/motion/vtrdyn/README_zh.md) |
| LinkerTG | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | - | `motion/udexreal` | [README_zh](src/linkerhand_retarget/linkerhand_retarget/motion/udexreal/README_zh.md) |
| LinkerMCG_G7 | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | `motion/linkermcg` | [README_zh](src/linkerhand_retarget/linkerhand_retarget/motion/linkermcg/README_zh.md) |
| LinkerMCG_M7 | ✅ | ✅ | - | ✅ | ✅ | ✅ | ✅ | ✅ | - | `motion/linkermcg_m7` | [协议](LinkerHand_UDP_M7&M11.md) |
| LinkerMCG_M11 | ✅ | ✅ | - | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | `motion/linkermcg_m11` | [协议](LinkerHand_UDP_M7&M11.md) |
| LinkerEG | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | - | - | `motion/linkereg` | [README_zh](src/linkerhand_retarget/linkerhand_retarget/motion/linkereg/README_zh.md) |

`motion_type: linkermcg` 保留为 `linkermcg_g7` 的向后兼容别名。

## ROS2 兼容性

| ROS2 版本 | Python | x86_64 | ARM64 | Windows |
|-----------|--------|--------|-------|---------|
| Foxy | 3.8 | ✅ | ✅ | ✅ |
| Humble | 3.10 | ✅ | ✅ | ✅ |
| Jazzy | 3.12 | ✅ | ✅ | ✅ |

## 工作区结构

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

`launch/` 仅作为源码参考保留。本仓库统一通过 `ros2 run` 启动，不安装 launch 入口。

## 快速开始

按需安装 Python 依赖：

```bash
python3 -m pip install -r src/requirements.txt
```

从仓库根目录编译：

```bash
colcon build --symlink-install
source install/setup.bash
```

启动 ROS2 节点：

```bash
ros2 run linkerhand_retarget handretarget
```

启用标定：

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=True
```

LinkerFFG 串口设备可先确认 USB 设备名并按需赋权：

```bash
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

仓库根目录提供 `run_linkerhand_teleop.sh`，作为仓库级遥操作快速启动脚本。脚本按当前 `base_config.yml` 中的 motion source 启动，默认不执行标定。脚本中的串口权限处理仅适用于 LinkerFFG 模块；其他模块请按照对应模块文档完成网络、串口或设备配置。

## 配置说明

主要配置文件：

```text
src/linkerhand_retarget/linkerhand_retarget/config/base_config.yml
```

### `system`

| 配置项 | 说明 | 可选值 |
|--------|------|--------|
| `motion_type` | 数据手套类型 | `linkerforce`, `vtrdyn`, `udexreal`, `udexrealv2t`, `linkermcg_g7`, `linkermcg_m7`, `linkermcg_m11`, `linkereg1`, `linkereg2`（`linkermcg` 是 `linkermcg_g7` 的兼容别名） |
| `datasource_type` | 数据源类型 | `motion` |
| `retargeting_type` | 重定向类型 | `projection` |
| `robotname_r` | 右手机械手型号 | `o6`, `l6`, `l7`, `l10`, `l10v7`, `l20`, `l21`, `l25`, `g20`, `o20`, `o30`（`o20` 适用于 `linkerforce`、`linkermcg_g7`、`linkermcg_m7` 或 `linkermcg_m11`；`o30` 适用于 `linkerforce` 或 `linkermcg_m11`） |
| `robotname_l` | 左手机械手型号 | 同上 |
| `motion_device` | 运动设备标识，由所选 motion source 使用 | 例如 `eric` |

### `serial` - LinkerFFG

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `auto_scan` | 是否自动扫描串口 | `false` |
| `baudrates` | 候选波特率列表 | `[2000000, 460800, 1000000, 921600]` |
| `left.port` / `right.port` | 左右手串口路径 | 如 `/dev/ttyUSB0` |
| `left.baudrate` / `right.baudrate` | 左右手波特率 | `460800` |
| `exclude_ports` | 排除的串口列表 | `[]` |
| `serial_debug` | 串口调试开关 | `false` |

### `udp` - LinkerTG / LinkerMCG / VTR-DYN

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `ip` | 手套 UDP 服务器 IP | `192.168.11.88` |
| `port` | 手套 UDP 端口 | `8888` |
| `serverport` | 本机监听端口 | `5551` |

### `mujoco` - 可选显示模块

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `enabled` | SDK 启动时是否拉起 MuJoCo viewer | `false` |
| `hands` | 要启动的手部 viewer。`auto` 优先跟随当前 motion 模块实际加载到的手套；也可手动设置 `[right]`、`[left]` 或 `[right, left]`。 | `auto` |
| `fps` | 显示模块预留刷新率配置 | `30` |

显示模块复用 SDK 现有机器人映射，根据 `robotname_r` 和 `robotname_l` 自动加载对应 URDF。使用 `hands: auto` 时，会在 motion 模块初始化完成后，根据实际加载到的手套自动启动一个或两个 MuJoCo viewer 实例。MuJoCo 是可选 Python 依赖；缺少 `mujoco`、`mujoco.viewer` 或某只手对应 URDF 时，会针对该实例输出 warning，并继续运行 SDK 主流程。

### `calibration` - LinkerFFG

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `show_fist` | 是否显示握拳标定 | `true` |
| `fist_extend_ratio` | 握拳延伸比例 | `0.5` |

### `debug`

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| `mapper_debug` | 映射器调试开关 | `false` |
| `joint_motor_debug_r` | 右手关节电机调试 | `false` |
| `joint_motor_debug_l` | 左手关节电机调试 | `false` |
| `joint_pub_debug` | 话题发布调试 | `false` |

### LinkerEG

```yaml
linkereg:
  port: null
  password: "i"
```

## 配置示例

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

## ROS2 话题

常用输出话题：

```bash
ros2 topic echo /cb_left_hand_control_cmd
ros2 topic echo /cb_right_hand_control_cmd
```

部分模块会发布或订阅额外话题，详见 [支持设备](#支持设备) 中链接的模块文档。

## 测试

离线单元测试可从仓库根目录运行：

```bash
PYTHONPATH=src/linkerhand_retarget python3 -m pytest src/linkerhand_retarget/tests/unit -q
```

集成测试可能需要真实硬件、串口设备、网络设备或完整 ROS2 运行环境。

## 许可证

本项目使用 MIT License，详见 [LICENSE](LICENSE)。
