# MuJoCo 安装清单

> 这是可选补充，不写入主依赖清单。只在需要 MuJoCo 可视化显示时执行。

## 安装步骤

- [ ] 确认当前使用的 `python3` 和 ROS2 环境一致

```bash
source /opt/ros/jazzy/setup.bash
which python3
python3 --version
```

- [ ] 升级 `pip`

```bash
python3 -m pip install -U pip
```

- [ ] 安装主依赖

```bash
python3 -m pip install -r src/requirements.txt
```

- [ ] 额外安装 MuJoCo

```bash
python3 -m pip install mujoco
```

- [ ] 验证 MuJoCo 和 viewer 能导入

```bash
python3 - <<'PY'
import mujoco
import mujoco.viewer
print(mujoco.__version__)
PY
```

- [ ] 在配置里打开 MuJoCo 显示

```yaml
mujoco:
  enabled: true
```

- [ ] 启动 SDK 并确认显示模块正常

```bash
ros2 run linkerhand_retarget handretarget
```

## 说明

- MuJoCo 只影响可选显示，不影响主流程。
- 如果不需要可视化，可以跳过这份清单。
