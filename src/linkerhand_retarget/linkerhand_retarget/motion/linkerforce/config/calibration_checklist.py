DEFAULT_CALIBRATION_JOINT_COUNT = 21

DEFAULT_CALIBRATION_JOINT_LABELS = {
    0: "拇指侧摆0",
    1: "拇指旋转/侧摆1",
    2: "拇指弯曲/侧摆2",
    3: "拇指弯曲3",
    4: "拇指指尖4",
    5: "食指侧摆",
    6: "食指指根",
    7: "食指指中",
    8: "食指指尖",
    9: "中指侧摆",
    10: "中指指根",
    11: "中指指中",
    12: "中指指尖",
    13: "无名指侧摆",
    14: "无名指指根",
    15: "无名指指中",
    16: "无名指指尖",
    17: "小指侧摆",
    18: "小指指根",
    19: "小指指中",
    20: "小指指尖",
}

DEFAULT_CALIBRATION_FILTER_CONFIG = {
    "tracked_joints": tuple(range(DEFAULT_CALIBRATION_JOINT_COUNT)),
    "pose_tracked_joints": {},
    "joint_labels": DEFAULT_CALIBRATION_JOINT_LABELS,
}

_POSE_ALIASES = {
    "open": "original",
    "original": "original",
    "opose": "opose",
    "fist": "fist",
}


def _normalize_indices(indices):
    seen = set()
    result = []
    for idx in indices:
        int_idx = int(idx)
        if int_idx < 0 or int_idx in seen:
            continue
        seen.add(int_idx)
        result.append(int_idx)
    return tuple(result)


def normalize_calibration_filter_config(config=None):
    config = config or {}
    tracked_joints = _normalize_indices(
        config.get("tracked_joints", DEFAULT_CALIBRATION_FILTER_CONFIG["tracked_joints"])
    )

    pose_tracked_joints = {}
    for pose_name, indices in config.get("pose_tracked_joints", {}).items():
        pose_key = _POSE_ALIASES.get(str(pose_name), str(pose_name))
        pose_tracked_joints[pose_key] = _normalize_indices(indices)

    joint_labels = dict(DEFAULT_CALIBRATION_JOINT_LABELS)
    joint_labels.update({int(k): str(v) for k, v in config.get("joint_labels", {}).items()})

    return {
        "tracked_joints": tracked_joints,
        "pose_tracked_joints": pose_tracked_joints,
        "joint_labels": joint_labels,
    }


def get_calibration_joint_indices(source=None, pose=None):
    if isinstance(source, dict):
        config = source
    else:
        config = getattr(source, "calibration_filter_config", None)

    normalized = normalize_calibration_filter_config(config)
    pose_key = _POSE_ALIASES.get(str(pose), str(pose)) if pose is not None else None
    if pose_key in normalized["pose_tracked_joints"]:
        return normalized["pose_tracked_joints"][pose_key]
    return normalized["tracked_joints"]


def get_calibration_skipped_joint_indices(
    source=None,
    pose=None,
    total_joints=DEFAULT_CALIBRATION_JOINT_COUNT,
):
    tracked = set(get_calibration_joint_indices(source, pose=pose))
    return tuple(idx for idx in range(total_joints) if idx not in tracked)
