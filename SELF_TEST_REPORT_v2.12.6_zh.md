# LinkerHand Teleop ROS2 v2.12.6 自测报告

## 1. 测试对象

- 项目：`linkerhand-telop-ros2`
- 版本：`2.12.6`
- 分支：`feat-linker-bot/linkerhand-telop-ros2#o20-init`
- 远程分支：`origin/docs-linker-bot/linkerhand-telop-ros2#5`
- 测试提交：`d27133b`
- 测试时间：`2026-06-30 15:28:28 CST`

## 2. 测试环境

- 系统：`Linux 6.17.0-29-generic x86_64 GNU/Linux`
- Python：`3.12.3`
- 测试命令：

```bash
PYTHONPATH=src/linkerhand_retarget:src/linkerhand_retarget/linkerhand_retarget pytest src/linkerhand_retarget/tests -q
```

## 3. 自动化测试结果

自动化测试通过：

```text
327 passed, 2 skipped, 41 warnings in 1.44s
```

当前警告主要来自：

- `yourdfpy.py` 中 NumPy 标量转换的弃用提示。
- `handcore.py` 中部分投影测试触发的除零运行时警告。

以上警告未导致测试失败，本次未在功能范围内处理。

## 4. 本次自测覆盖范围

1. 版本号读取与启动日志  
   验证 SDK 可从根目录 `VERSION.md` 读取 `2.12.6`，并在节点启动时输出当前版本号。

2. 机械手型号注册  
   验证 O20、O30、O30i、L21 的型号枚举、名称映射、自由度长度映射和运行时路由。

3. LinkerFFG 映射  
   验证 O20/O30/O30i 按 `original`、`opose`、`fist` 标定锚点映射到 URDF 弧度，并按 URDF 上下限截断。

4. 电机输出归一化  
   验证 O30i 不再走 `HandCore.trans_to_motor_left/right`，而是按 MuJoCo/URDF 弧度归一化到 `0..255`；roll 类关节的反向输出逻辑已覆盖。

5. 标定流程  
   验证 LinkerFFG 标定顺序调整为：启用握拳采集时 `fist -> opose -> open`，禁用握拳采集时 `opose -> open`；同时覆盖右手单手连接、标定缓存机型校验、标定失败退出路径。

6. 串口增强  
   验证串口配置支持 `auto_scan`、`baudrates`、`exclude_ports`、命令行候选串口、固定串口和左右手 `handtype/version` 识别；验证位置帧计数用于判断真实连接状态。

7. MuJoCo 显示链路  
   验证无手套连接时不启动 MuJoCo，单手/双手连接时按实际加载手套选择模型；验证 URDF 资源映射、显示旋转/平移配置、mimic 关节和当前弧度输出同步。

8. 调试与文档  
   验证 O6 右手小指原始数据发布前跟踪日志、LinkerMCG O20/O30 路由、MuJoCo 依赖说明、第三方机械手弧度映射说明和 v2.12.6 版本说明。

## 5. 关键用例结论

| 模块 | 自测项 | 结果 |
| --- | --- | --- |
| 版本 | `VERSION.md` 读取与启动日志 | 通过 |
| 常量 | O20/O30/O30i/L21 注册和长度映射 | 通过 |
| LinkerFFG | O20 标定锚点、拇指 roll/yaw、四指侧摆 | 通过 |
| LinkerFFG | O30i topic 顺序、弧度到电机 `0..255` 输出 | 通过 |
| LinkerFFG | 标定顺序、右手单手标定、缓存机型校验 | 通过 |
| 串口 | 自动扫描、波特率列表、排除列表、左右手识别 | 通过 |
| MuJoCo | 手套自动检测、模型选择、mimic 同步、无手套不显示 | 通过 |
| 文档 | v2.12.6 更新说明、MuJoCo 安装清单、第三方映射说明 | 通过 |

## 6. 现场复测项

以下项目依赖真实数据手套、机械手、串口设备或图形环境，本次自动化测试只能覆盖代码路径和模拟输入，仍建议现场复测：

1. LinkerFFG 真实串口连接：固定串口、`auto_scan: true`、不同波特率组合、左右手同时连接和单手连接。
2. O6 右手小指根部原始数据跳变：开启调试后观察日志是否能稳定捕捉异常帧。
3. O20/O30i 实机动作：重点检查拇指 roll/yaw、四指侧摆、open/opose/fist 三个姿态的方向和边界。
4. MuJoCo 图形显示：真实启动后确认单手/双手模型选择、模型朝向、mimic 关节和实时弧度同步。
5. LinkerMCG UDP 显示：确认目标地址和 `9011` 端口配置，打开 `joint_motor_debug_l` / `joint_motor_debug_r` 后检查数据显示。

## 7. 自测结论

基于当前自动化测试结果，v2.12.6 的软件侧回归测试通过，已覆盖版本号、型号注册、LinkerFFG/O20/O30i 映射、标定流程、串口增强、MuJoCo 自动显示和文档说明。

硬件链路相关内容仍需按第 6 节进行现场复测，尤其是串口自动识别、真实手套输入抖动、实机动作边界和 MuJoCo 图形显示。
