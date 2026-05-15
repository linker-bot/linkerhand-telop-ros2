# LinkerHand ROS2 遥操作 SDK

仅面向 ROS2 的遥操作与重定向仓库。

## 编译

```bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 run linkerhand_retarget handretarget
```

标定运行：

```bash
ros2 run linkerhand_retarget handretarget --ros-args -p calibration:=auto_calibrate
```

## 工作区结构

```
linkerhand-telop-ros2/
├── src/
└── README_zh.md
```

## 说明

- 仅支持 ROS2 源码工作区编译。
- 使用 `colcon build --symlink-install`。
- 统一通过 `ros2 run` 启动，不再安装 launch 入口。

