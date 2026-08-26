# LinkerMCG M7/M11 UDP 协议

本文档说明本项目 `linkermcg_m7` 和 `linkermcg_m11` motion source 接收的 UDP JSON 数据格式。

## 连接参数

- 传输协议：UDP
- 默认端口：`9011`
- `udp.ip`：接收数据的目标主机地址，不能配置为 `0.0.0.0`
- 数组中的数值必须是有限数值，左右手数组长度必须与 `dof` 一致

## 标准报文

```json
{
  "schemaId": "linker.stroke6.flat.v1",
  "handType": "LinkerHand/O6",
  "dof": 6,
  "timestampMs": 1710000000789,
  "labels": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
  "leftHand": [0, 10, 20, 30, 40, 50],
  "rightHand": [0, 10, 20, 30, 40, 50]
}
```

实际报文中的 `labels`、`leftHand` 和 `rightHand` 必须各有 `dof` 个元素。字段定义如下：

| 字段 | 类型 | 说明 |
|---|---|---|
| `schemaId` | string | 数据结构标识 |
| `handType` | string | 机械手型号，例如 `LinkerHand/O20` |
| `dof` | int | 左右手数组长度 |
| `timestampMs` | int | Unix 毫秒时间戳 |
| `labels` | string[] | 与左右手数组同下标的关节名称 |
| `leftHand` | number[] | 左手目标值 |
| `rightHand` | number[] | 右手目标值 |

## M7

配置：

```yaml
system:
  motion_type: linkermcg_m7
```

M7 支持以下标准结构：

| schemaId | dof | 默认 handType | 数值含义 |
|---|---:|---|---|
| `linker.stroke6.flat.v1` | 6 | `LinkerHand/O6` | `0..255` 电机行程值 |
| `linker.stroke10.flat.v1` | 10 | `LinkerHand/L10` | `0..255` 电机行程值 |
| `linker.stroke20.flat.v1` | 20 | `LinkerHand/O20` | `0..255` 电机行程值 |

M7 不支持 O30。

M7 还兼容仅包含 `leftHand` 和 `rightHand` 的简化报文。两侧数组长度必须相同且只能是 6、10 或 20；接收端会按长度补全 `schemaId`、`handType`、`timestampMs` 和 `labels`。

```json
{
  "leftHand": [0, 10, 20, 30, 40, 50],
  "rightHand": [0, 10, 20, 30, 40, 50]
}
```

## M11

配置：

```yaml
system:
  motion_type: linkermcg_m11
```

M11 支持 M 系列通用的 6、10、20 路行程结构，并额外支持以下型号专用结构：

| 型号 | schemaId | dof | 数值含义 |
|---|---|---:|---|
| O20 | `linker.o20.targetpos16.flat.v1` | 16 | 有符号目标角度，单位为度 |
| O30 | `linker.o30.stroke20.flat.v1` | 20 | `0..255` 电机行程值 |

### M11 O20 示例

```json
{
  "schemaId": "linker.o20.targetpos16.flat.v1",
  "handType": "LinkerHand/O20",
  "dof": 16,
  "timestampMs": 1710000000789,
  "labels": [
    "thumb_mcp", "thumb_ip", "thumb_abd", "thumb_cmc",
    "index_abd", "index_mcp", "index_pip",
    "middle_abd", "middle_mcp", "middle_pip",
    "ring_abd", "ring_mcp", "ring_pip",
    "pinky_abd", "pinky_mcp", "pinky_dip"
  ],
  "leftHand": [0, 10, -30, 40, -20, 60, 70, 0, 90, 100, -15, 120, 130, -10, 150, 160],
  "rightHand": [1, 11, -29, 41, -19, 61, 71, 1, 91, 101, -14, 121, 131, -9, 151, 161]
}
```

### M11 O30 示例

```json
{
  "schemaId": "linker.o30.stroke20.flat.v1",
  "handType": "LinkerHand/O30",
  "dof": 20,
  "timestampMs": 1710000000789,
  "labels": [
    "joint1", "joint2", "joint3", "joint4", "joint5",
    "joint6", "joint7", "joint8", "joint9", "joint10",
    "joint11", "joint12", "joint13", "joint14", "joint15",
    "joint16", "joint17", "joint18", "joint19", "joint20"
  ],
  "leftHand": [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190],
  "rightHand": [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190]
}
```

## 状态报文

接收端会识别并忽略连接、心跳和断开等状态报文，不会将其作为动作帧发布。支持纯文本 `CONNECT`、`HEARTBEAT`、`PING`、`DISCONNECT`，以及包含 `action` 或 `status` 字段的 JSON。
