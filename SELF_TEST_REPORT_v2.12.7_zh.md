# LinkerHand Teleop ROS2 v2.12.7 自测报告

## 1. 测试对象

- 项目：`linkerhand-telop-ros2`
- 版本：`2.12.7`
- 分支：`feat-linker-bot/linkerhand-telop-ros2#o20-init`
- 远程分支：`origin/docs-linker-bot/linkerhand-telop-ros2#5`
- 功能基线提交：`4374b76`
- 测试时间：`2026-08-03 11:02:59 CST`

## 2. 测试环境

- 系统：`Linux 6.17.0-29-generic x86_64 GNU/Linux`
- Python：`3.12.3`
- 测试命令：

```bash
PYTHONPATH=src/linkerhand_retarget:src/linkerhand_retarget/linkerhand_retarget /usr/bin/python3 -m pytest src/linkerhand_retarget/tests/unit/test_linkerforce.py src/linkerhand_retarget/tests/unit/test_linkerforce_o30i.py -q
```

## 3. 自动化测试结果

本次针对 LinkerForce 串口解析和 O30i 映射相关单测执行后，结果如下：

```text
54 passed, 2 failed in 0.43s
```

失败用例集中在 O30i 右手拇指实时 open-to-fist 方向映射：

- `test_o30i_right_thumb_cmc_roll_maps_realtime_open_to_fist_direction`
- `test_o30i_right_thumb_cmc_yaw_maps_realtime_open_to_fist_direction`

当前失败现象为实时输入接近 fist 时，`thumb_cmc_roll` 和 `thumb_cmc_yaw` 输出弧度未达到测试期望阈值。该问题属于 O30i 实时映射标定/期望值仍需继续核对的遗留项，本次版本记录不将其标记为通过。

## 4. 本次更新覆盖范围

1. LinkerForce 位置帧解析  
   校验 `0x03` 与 `0xA6` 位置帧必须为 21 路关节数据，拒绝长度异常、解包异常和非有限数值，减少坏帧污染实时姿态。

2. LinkerForce 串口帧重同步
   解析器在帧头后的命令字阶段发现非法命令时立即丢弃候选帧，继续寻找下一个有效帧头，避免数据区 `0x5D` 误触发 `Unknown command: 0x66` 并吞掉真实 `0x03` 位置帧。

3. LinkerForce 异常日志
   未知命令帧仅写入 `linkerforce_abnormal.log`，记录命令、长度、校验和完整帧 hex，不再通过 ROS warn 刷屏；发布值出现 `255->0` 或 `0->255` 端点跳变时，记录触发关节的上一值、当前值和前后帧原始数据。`0.35 rad` 原始弧度小跳变不再落日志。

4. 串口写入链路
   运行线程查询包、初始化版本查询包和力反馈包统一走 `ForceSerialReader` 的写入封装，串口写入由锁保护。

5. O6 映射状态
   已按参考项目同步 O6 配置，默认多段映射状态调整为 `original -> fist`，保留 `opose` 状态配置。

6. O30i 输出方向与归一化
   更新 O30i 拇指旋转和四指 roll 的电机归一化测试，当前仍需继续核对右手拇指实时 open-to-fist 的期望边界。

7. 现场测试配置
   `base_config.yml` 默认切换到 LinkerForce/O6 左右手，右手串口示例为 `/dev/ttyUSB1`；`run_linkerhand_teleop.sh` 默认进入标定模式。

## 5. 关键结论

| 模块 | 自测项 | 结果 |
| --- | --- | --- |
| 版本 | `VERSION.md` 读取与包版本号同步 | 待本次版本更新后验证 |
| LinkerForce | 位置帧长度、通道数、非有限值保护 | 通过 |
| LinkerForce | 非法命令候选帧重同步 | 通过 |
| LinkerForce | 异常帧与 `255<->0` 端点跳变日志 | 通过 |
| LinkerForce | 串口运行线程写入走统一写锁 | 通过 |
| O6 | 默认映射状态切换为 `original -> fist` | 通过 |
| O30i | URDF 弧度到 `0..255` 电机归一化 | 部分通过 |
| O30i | 右手拇指实时 open-to-fist roll/yaw 边界 | 未通过 |

## 6. 现场复测项

1. 使用真实 LinkerForce 手套复测异常日志，重点观察 `Unknown command` 的完整帧 hex，以及 `255->0`、`0->255` 端点跳变的 `prev/current/raw_previous/raw_current`。
2. 使用真实 O6 左右手复测默认配置和标定启动脚本，确认 `/dev/ttyUSB0`、`/dev/ttyUSB1` 与现场接线一致。
3. 使用真实 O30i 右手复测拇指 `thumb_cmc_roll`、`thumb_cmc_yaw` 的 open、opose、fist 三个姿态边界。
4. 若 O30i 右手拇指实机方向正确但单测仍失败，需要重新确认测试里的标定输入和期望阈值。

## 7. 自测结论

v2.12.7 已完成版本号、更新说明和自测报告记录。软件侧单测显示 LinkerForce 串口解析、异常日志和串口写入链路相关用例通过；O30i 右手拇指实时 open-to-fist 边界仍有 2 个失败用例，需要后续结合最新标定文件和实机表现继续处理。
