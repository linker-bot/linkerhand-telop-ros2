# L6 版本区分与延伸映射优化测试报告

> **历史硬件测试记录**：本文档记录早期真实设备测试结果，不属于默认 CI。当前默认 CI 只运行 ROS2 离线单元测试；硬件、串口、UDP 和标定测试需要在实验室环境手动执行。

**测试日期：** 2026-03-24  
**测试版本：** v2.11.7  
**测试环境：** ROS2 Foxy, LinkerForce L6 左手/右手, 手套版本 v1 (1.2.12)

## 更新内容

1. v1/v2 版本区分支持（weights, reverse_motion 字典格式）
2. 延伸映射只在 ['original', 'opose'] 模式触发
3. 移除自动拟合功能
4. 添加 MULTI_SEGMENT_CONFIG_FROZEN 配置冻结
5. 更新 README 标定配置建议
6. 修复标定结束后历史数据加载问题
7. 添加电机输出约束功能 (MOTOR_CONSTRAINTS)
8. 优化延伸映射参数
9. 更新标定样本数据

## 测试内容

| 功能 | 状态 |
|------|------|
| v1/v2 版本区分 | ✅ 通过 |
| 延伸映射触发条件 | ✅ 通过 |
| 配置冻结机制 | ✅ 通过 |
| 两段标定+延伸 (open + opose) | ✅ 通过 |
| 两段标定直连 (open + fist) | ✅ 通过 |
| 标定历史数据加载 | ✅ 通过 |
| 左手测试 | ✅ 通过 |
| 右手测试 | ✅ 通过 |
| 电机输出约束 | ✅ 通过 |
| ROS1 同步 | ✅ 通过 |

**结论：** v2.11.7 版本功能正常，可以发布。

**Gitee 分支：** https://gitee.com/ericbrunt/linkerhand_telop_python/tree/fix-linker-bot/linkerhand_telop_python%2310
