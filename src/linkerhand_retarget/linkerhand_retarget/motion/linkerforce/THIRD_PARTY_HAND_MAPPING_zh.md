# 第三方机械手弧度映射说明

## 范围

输入为 LinkerForce 手套解析后的 21 维原始弧度：

```python
glove_current = [
    g0, g1, g2, g3, g4,
    g5, g6, g7, g8,
    g9, g10, g11, g12,
    g13, g14, g15, g16,
    g17, g18, g19, g20,
]
```

输出为第三方机械手自己的 N 维目标关节弧度：

```python
robot_current = [
    joint_0,
    joint_1,
    ...
    joint_n,
]
```

下游协议、电机值、SDK 命令由驱动层处理。本层只处理：

```text
手套原始弧度 -> 第三方机械手目标弧度
```

---

## 标定数据

手套侧标定数据来自实际采集：

```python
GLOVE_ORIGINAL = [...]  # 张手，21 维
GLOVE_OPOSE = [...]     # O 手势，21 维
GLOVE_FIST = [...]      # 握拳，21 维
```

机械手侧目标弧度由第三方机械手模型或实机姿态确定：

```python
ROBOT_ORIGINAL = [...]  # 张手，N 维
ROBOT_OPOSE = [...]     # O 手势，N 维
ROBOT_FIST = [...]      # 握拳，N 维
```

`ROBOT_*` 全部使用弧度。数组顺序与第三方机械手目标关节顺序保持一致。

---

## 关节配置

每个第三方目标关节都需要定义来源手套关节、权重和输出下标。

```python
FINGER_CONFIGS = {
    "thumb_yaw": {
        "joints": [1],
        "weights": [1.0],
        "robot_idx": 0,
    },
    "thumb_pitch": {
        "joints": [2, 3, 4],
        "weights": [0.5, 0.2, 0.3],
        "robot_idx": 1,
    },
    "index_flex": {
        "joints": [6, 7, 8],
        "weights": [0.6, 0.2, 0.2],
        "robot_idx": 2,
    },
    "middle_flex": {
        "joints": [10, 11, 12],
        "weights": [0.6, 0.2, 0.2],
        "robot_idx": 3,
    },
    "ring_flex": {
        "joints": [14, 15, 16],
        "weights": [0.6, 0.2, 0.2],
        "robot_idx": 4,
    },
    "pinky_flex": {
        "joints": [18, 19, 20],
        "weights": [0.6, 0.2, 0.2],
        "robot_idx": 5,
    },
}
```

字段说明：

| 字段 | 含义 |
|------|------|
| `joints` | 参与该目标关节计算的手套原始弧度下标 |
| `weights` | 手套关节融合权重 |
| `robot_idx` | 第三方机械手目标弧度数组下标 |

权重只影响手套侧融合值，不改变第三方机械手目标弧度。

---

## 融合值

一个目标关节可以由一个或多个手套关节融合得到：

```python
def normalize_weights(weights):
    total = sum(weights)
    if abs(total) < 1e-9:
        return [0.0 for _ in weights]
    return [value / total for value in weights]


def fused_value(glove_values, joints, weights):
    weights = normalize_weights(weights)
    return sum(
        float(glove_values[index]) * weight
        for index, weight in zip(joints, weights)
    )
```

同一个目标关节的 `current/original/opose/fist` 使用同一组 `joints` 和 `weights`：

```python
current = fused_value(glove_current, joints, weights)
original = fused_value(GLOVE_ORIGINAL, joints, weights)
opose = fused_value(GLOVE_OPOSE, joints, weights)
fist = fused_value(GLOVE_FIST, joints, weights)
```

---

## 分段线性映射

三段映射使用 `original -> opose -> fist` 三个锚点。`opose` 用于保证 O 手势能落到指定的第三方机械手弧度。

```python
def clamp(value, lower, upper):
    return min(max(value, min(lower, upper)), max(lower, upper))


def between(value, a, b):
    return min(a, b) <= value <= max(a, b)


def linear(value, source_a, source_b, target_a, target_b):
    if abs(source_b - source_a) < 1e-9:
        return target_b
    ratio = (value - source_a) / (source_b - source_a)
    ratio = clamp(ratio, 0.0, 1.0)
    return target_a + ratio * (target_b - target_a)


def piecewise_3(value,
                glove_original,
                glove_opose,
                glove_fist,
                robot_original,
                robot_opose,
                robot_fist):
    if between(value, glove_original, glove_opose):
        return linear(
            value,
            glove_original,
            glove_opose,
            robot_original,
            robot_opose,
        )

    if between(value, glove_opose, glove_fist):
        return linear(
            value,
            glove_opose,
            glove_fist,
            robot_opose,
            robot_fist,
        )

    anchors = [
        (abs(value - glove_original), robot_original),
        (abs(value - glove_opose), robot_opose),
        (abs(value - glove_fist), robot_fist),
    ]
    return min(anchors, key=lambda item: item[0])[1]
```

---

## 完整映射函数

```python
THIRD_PARTY_JOINTS = [
    "thumb_yaw",
    "thumb_pitch",
    "index_flex",
    "middle_flex",
    "ring_flex",
    "pinky_flex",
]


ROBOT_ORIGINAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ROBOT_OPOSE = [0.8, 0.6, 0.9, 0.8, 0.7, 0.6]
ROBOT_FIST = [1.1, 1.2, 1.6, 1.5, 1.4, 1.3]


def map_glove_to_third_party(glove_current,
                              glove_original,
                              glove_opose,
                              glove_fist):
    robot_current = [0.0] * len(THIRD_PARTY_JOINTS)

    for config in FINGER_CONFIGS.values():
        joints = config["joints"]
        weights = config["weights"]
        robot_idx = config["robot_idx"]

        current = fused_value(glove_current, joints, weights)
        original = fused_value(glove_original, joints, weights)
        opose = fused_value(glove_opose, joints, weights)
        fist = fused_value(glove_fist, joints, weights)

        robot_current[robot_idx] = piecewise_3(
            current,
            original,
            opose,
            fist,
            ROBOT_ORIGINAL[robot_idx],
            ROBOT_OPOSE[robot_idx],
            ROBOT_FIST[robot_idx],
        )

    return robot_current
```

---

## 方向

方向由 `ROBOT_ORIGINAL / ROBOT_OPOSE / ROBOT_FIST` 三组目标弧度决定，不额外依赖反向开关。

张手到握拳为正方向：

```python
ROBOT_ORIGINAL = [0.0]
ROBOT_OPOSE = [0.4]
ROBOT_FIST = [0.9]
```

张手到握拳为负方向：

```python
ROBOT_ORIGINAL = [0.8]
ROBOT_OPOSE = [0.4]
ROBOT_FIST = [0.1]
```

跨零范围：

```python
ROBOT_ORIGINAL = [-0.5]
ROBOT_OPOSE = [0.0]
ROBOT_FIST = [0.7]
```

三组目标值按第三方机械手真实弧度填写后，线性插值按目标锚点方向输出。

---

## 限位

映射后统一做弧度截断：

```python
JOINT_LIMITS = [
    (-0.5, 1.2),
    (0.0, 1.4),
    (0.0, 1.7),
    (0.0, 1.7),
    (0.0, 1.6),
    (0.0, 1.5),
]


def clamp_robot_joints(robot_current):
    result = []
    for value, (lower, upper) in zip(robot_current, JOINT_LIMITS):
        result.append(clamp(value, lower, upper))
    return result
```

使用方式：

```python
robot_current = map_glove_to_third_party(
    glove_current,
    GLOVE_ORIGINAL,
    GLOVE_OPOSE,
    GLOVE_FIST,
)
robot_current = clamp_robot_joints(robot_current)
```

---

## 输出平滑

原始弧度或融合值存在抖动时，在输出弧度上增加平滑和单帧步长限制。

```python
def smooth(prev, current, alpha=0.3):
    return [
        alpha * now + (1.0 - alpha) * old
        for old, now in zip(prev, current)
    ]


def limit_step(prev, current, max_step=0.05):
    result = []
    for old, now in zip(prev, current):
        delta = clamp(now - old, -max_step, max_step)
        result.append(old + delta)
    return result
```

典型顺序：

```python
robot_current = map_glove_to_third_party(...)
robot_current = clamp_robot_joints(robot_current)
robot_current = smooth(last_robot_current, robot_current, alpha=0.3)
robot_current = limit_step(last_robot_current, robot_current, max_step=0.05)
```

---

## 校验

锚点输入需要回到对应目标弧度：

```python
assert map_glove_to_third_party(
    GLOVE_ORIGINAL,
    GLOVE_ORIGINAL,
    GLOVE_OPOSE,
    GLOVE_FIST,
) == ROBOT_ORIGINAL

assert map_glove_to_third_party(
    GLOVE_OPOSE,
    GLOVE_ORIGINAL,
    GLOVE_OPOSE,
    GLOVE_FIST,
) == ROBOT_OPOSE

assert map_glove_to_third_party(
    GLOVE_FIST,
    GLOVE_ORIGINAL,
    GLOVE_OPOSE,
    GLOVE_FIST,
) == ROBOT_FIST
```

逐关节检查：

- `original -> opose` 输出方向符合第三方机械手定义。
- `opose -> fist` 输出方向符合第三方机械手定义。
- 负弧度到正弧度的关节能跨零插值。
- 输出值不会超过第三方机械手真实弧度限位。
- 平滑和步长限制不会改变标定锚点的最终目标值。
