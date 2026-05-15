# LinkerForce 集成测试报告

> **历史硬件测试记录**：本文档记录早期真实设备测试结果，不属于默认 CI。当前默认 CI 只运行 ROS2 离线单元测试；硬件、串口、UDP 和标定测试需要在实验室环境手动执行。

测试日期: 2026-03-03  
测试设备: LinkerForce 左手数据手套  
测试结果: 6/6 通过

---

## 启动命令

```bash
cd /home/linker-brunt/project/linkerhand_telop_sdk/old_git/new/ros2/src/linkerhand_retarget
python3 tests/integration/test_linkerforce.py
```

前提条件:
- LinkerForce 设备已连接到 `/dev/ttyUSB0`
- 波特率: 2000000

---

## 测试环境

- 串口: /dev/ttyUSB0
- 波特率: 2000000
- 设备类型: Left (左手)
- 固件版本: 1.2.12
- 总接收帧数: 1266

---

## 测试结果

| 测试项 | 结果 | 关键指标 |
|--------|------|----------|
| 设备信息 | 通过 | handtype=Left, version=1.2.12 |
| 数据稳定性 | 通过 | 标准差 0.00007 rad (0.004°) |
| 数据包统计 | 通过 | 315帧/10秒, 数据变化率 95% |
| 响应间隔 | 通过 | 平均 30.82ms, 帧率 20Hz |
| 连续读取 | 通过 | 485帧/15秒, 稳定无断开 |
| 协议测试 | 通过 | 6/6 协议通过 |

---

## 详细数据

### 1. 设备信息测试

```
handtype: Left
version: 1.2.12
connflag: True
```

### 2. 位置数据稳定性测试 (5秒)

- 采样数: 10
- 通道数: 10
- 平均标准差: 0.000071 rad (0.0041°)
- 最大标准差: 0.000254 rad (0.0145°)

数据稳定性良好。

### 3. 数据包统计测试 (10秒)

- 发送请求: 20
- 接收帧数: 315
- 数据变化: 19
- 帧率: ~31.5 Hz

设备持续输出数据。

### 4. 响应间隔测试 (5秒)

- 接收帧数: 100
- 平均帧间隔: 30.82 ms
- 最小帧间隔: 12.11 ms
- 最大帧间隔: 42.87 ms
- 帧率: 20.0 Hz

响应及时稳定。

### 5. 连续读取稳定性测试 (15秒)

- 接收帧数: 485
- 平均帧间隔: 500.68 ms
- 帧率: 2.0 Hz (受测试间隔限制)
- 设备断开: 0 次

稳定运行，无断开无错误。

### 6. 协议测试

| 协议 | 功能 | 状态 |
|------|------|------|
| 0x01 | 设备信息 | 通过 |
| 0x02 | 控制命令 | 通过 |
| 0x03 | 位置数据 | 通过 (21 floats) |
| 0x04 | 力数据 | 通过 |
| 0xA4 | 力发送 | 通过 |
| 0xA7 | 力发送变体 | 通过 |

---

## 协议说明

### 支持的协议命令

| 命令码 | 功能 | 方向 | 说明 |
|--------|------|------|------|
| 0x01 | 设备信息 | 主机→设备 | 查询设备类型和版本 |
| 0x02 | 控制命令 | 主机→设备 | 发送控制参数 |
| 0x03 | 位置数据 | 设备→主机 | 返回21个关节角度(float) |
| 0x04 | 力数据 | 设备→主机 | 返回力传感器数据(int16) |
| 0xA4 | 力发送 | 主机→设备 | 发送力反馈数据 |
| 0xA7 | 力发送变体 | 主机→设备 | 发送力反馈数据(备用) |

### 数据格式

位置数据 (0x03):
- 格式: 21个 float (小端序)
- 单位: 弧度
- 更新频率: ~20-30 Hz

力数据 (0x04):
- 格式: 5个 int16
- 单位: 原始ADC值

---

## 测试结论

设备工作正常:
- 通信稳定: 无断开、无错误
- 数据精确: 角度标准差 < 0.015°
- 响应及时: 平均帧间隔 30ms, 帧率 20Hz
- 协议完整: 6个协议全部通过

---

## 测试文件结构

```
tests/
├── TEST_LINKERFORCE.md           # 本测试报告
├── unit/
│   ├── test_linkerforce.py       # LinkerForce 单元测试
│   ├── test_filter.py            # 滤波器测试
│   ├── test_constants.py         # 常量测试
│   ├── test_handcore.py          # HandCore测试
│   ├── test_handcoreex.py        # HandCoreEx测试
│   ├── test_linkermcgcore.py     # LinkerMCG测试
│   ├── test_sensenovacore.py     # SenseNova测试
│   ├── test_udexrealcore.py      # UdexReal测试
│   ├── test_utils.py             # 工具函数测试
│   └── test_vtrdyncore.py        # VtrDyn测试
└── integration/
    ├── test_linkerforce.py       # LinkerForce 集成测试
    └── test_config.py            # 测试配置
```

### LinkerForce 测试文件

单元测试 `tests/unit/test_linkerforce.py`:
- CircularBuffer 测试 (7项)
- FrameParser 测试 (5项)
- 常量测试 (4项)

集成测试 `tests/integration/test_linkerforce.py`:
- 设备信息测试
- 数据稳定性测试
- 数据包统计测试
- 响应间隔测试
- 连续读取测试
- 协议测试

## 运行测试

```bash
# 进入测试目录
cd ros2/src/linkerhand_retarget

# 运行单元测试
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit/ -v

# 运行集成测试 (需连接设备)
python3 tests/integration/test_linkerforce.py
```
