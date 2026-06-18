from pathlib import Path

import pytest

from linkerhand_retarget.motion.linkerforce.config.o20_config import (
    ROBOT_FIST_LEFT,
    ROBOT_FIST_RIGHT,
    ROBOT_ORIGINAL_LEFT,
    ROBOT_ORIGINAL_RIGHT,
    ROBOT_OPOSE_LEFT,
    ROBOT_OPOSE_RIGHT,
)
from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o20 import (
    LeftHand,
    O20_MUJOCO_JOINT_ARC_SIGNS,
    RightHand,
    _map_o20_qpos_to_motor,
)
from linkerhand_retarget.mujoco_display import (
    extract_mujoco_joint_positions,
    get_urdf_movable_joint_names,
)


class FakeHandCore:
    def __init__(self):
        self.last_qpos_l = None
        self.last_qpos_r = None

    def trans_to_motor_left(self, qpos):
        raise AssertionError("O20 must not call handcore.trans_to_motor_left")

    def trans_to_motor_right(self, qpos):
        raise AssertionError("O20 must not call handcore.trans_to_motor_right")


def _calibration(open_yaw, opose_yaw, fist_yaw):
    original = [0.0] * 21
    opose = [0.0] * 21
    fist = [0.0] * 21
    original[1] = open_yaw
    opose[1] = opose_yaw
    fist[1] = fist_yaw
    return original, opose, fist


def _state_calibration():
    original = [0.0] * 21
    opose = [1.0] * 21
    fist = [2.0] * 21
    return original, opose, fist


def _qpos_from_robot_state(robot_state):
    qpos = [0.0] * 25
    qpos[16] = robot_state[0]
    qpos[17] = robot_state[1]
    qpos[18] = robot_state[2]
    qpos[19] = robot_state[3]
    qpos[0] = robot_state[4]
    qpos[1] = robot_state[5]
    qpos[2] = robot_state[6]
    qpos[8] = robot_state[7]
    qpos[9] = robot_state[8]
    qpos[10] = robot_state[9]
    qpos[12] = robot_state[10]
    qpos[13] = robot_state[11]
    qpos[14] = robot_state[12]
    qpos[4] = robot_state[13]
    qpos[5] = robot_state[14]
    qpos[6] = robot_state[15]
    return qpos


def _effective_qpos_from_hand(hand, state_name):
    return _qpos_from_robot_state(getattr(hand, f"effective_robot_{state_name}"))


def test_o20_right_thumb_cmc_yaw_reaches_opose_angle():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _calibration(
        4.526575977103334,
        4.760386137053435,
        5.205488511599119,
    )
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationopose)

    assert hand.g_jointpositions_arc[1] == pytest.approx(ROBOT_OPOSE_RIGHT[1])


def test_o20_right_thumb_cmc_yaw_arc_is_smoothed():
    hand = RightHand(FakeHandCore())
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _calibration(
        4.526575977103334,
        4.760386137053435,
        5.205488511599119,
    )
    hand.initialize_mapper()

    near_opose = list(hand.calibrationopose)
    near_opose[1] += 0.02

    hand.joint_update(near_opose)
    first_frame = hand.g_jointpositions_arc[1]
    hand.joint_update(near_opose)

    expected_opose_yaw = hand._map_thumb_cmc_yaw(near_opose)

    assert first_frame == pytest.approx(expected_opose_yaw * hand.smooth_alpha)
    assert hand.g_jointpositions_arc[1] == pytest.approx(
        expected_opose_yaw * hand.smooth_alpha
        + first_frame * (1 - hand.smooth_alpha)
    )


def test_o20_right_thumb_cmc_yaw_uses_filtered_input_for_mujoco_arc():
    hand = RightHand(FakeHandCore())
    hand.smooth_alpha = 1.0
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _calibration(
        4.526575977103334,
        4.760386137053435,
        5.205488511599119,
    )
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationopose)
    baseline_roll = hand.g_jointpositions_arc[0]
    baseline_yaw = hand.g_jointpositions_arc[1]

    jittered = list(hand.calibrationopose)
    jittered[1] += 0.02
    raw_yaw = hand._map_thumb_cmc_yaw(jittered)
    hand.joint_update(jittered)

    raw_delta = abs(raw_yaw - baseline_yaw)
    filtered_delta = abs(hand.g_jointpositions_arc[1] - baseline_yaw)

    assert 0 < filtered_delta < raw_delta


def test_o20_left_thumb_cmc_yaw_reaches_opose_angle():
    hand = LeftHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _calibration(
        4.098850093222668,
        3.483006318161248,
        3.175084430630538,
    )
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationopose)

    assert hand.g_jointpositions_arc[1] == pytest.approx(ROBOT_OPOSE_LEFT[1])


def test_o20_right_opose_maps_without_handcore_motor_conversion():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = [0.0] * 21
    hand.calibrationopose = [1.0] * 21
    hand.calibrationfistpose = [2.0] * 21
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationopose)

    assert hand.g_jointpositions_arc[0] == pytest.approx(hand.effective_robot_opose[0])
    assert hand.g_jointpositions_arc[1] == pytest.approx(hand.effective_robot_opose[1])
    assert hand.handcore.last_qpos_r is not None


def test_o20_right_opose_reaches_configured_thumb_and_index_angles():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationopose)

    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_OPOSE_RIGHT[0])
    assert hand.g_jointpositions_arc[5] == pytest.approx(ROBOT_OPOSE_RIGHT[4])


def test_o20_qpos_motor_mapping_does_not_require_hand_config_fields():
    handcore = FakeHandCore()

    motors = _map_o20_qpos_to_motor(handcore, _qpos_from_robot_state(ROBOT_ORIGINAL_RIGHT), "right")

    assert handcore.last_qpos_r is not None
    assert motors[5] == 0
    assert motors[10] == 0


@pytest.mark.parametrize(
    ("hand_cls", "original", "opose", "fist", "last_qpos_attr"),
    [
        (RightHand, ROBOT_ORIGINAL_RIGHT, ROBOT_OPOSE_RIGHT, ROBOT_FIST_RIGHT, "last_qpos_r"),
        (LeftHand, ROBOT_ORIGINAL_LEFT, ROBOT_OPOSE_LEFT, ROBOT_FIST_LEFT, "last_qpos_l"),
    ],
)
def test_o20_open_opose_fist_motor_direction_uses_local_o20_anchors(
    hand_cls, original, opose, fist, last_qpos_attr
):
    hand = hand_cls(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_motors = list(hand.g_jointpositions)
    hand.joint_update(hand.calibrationopose)
    opose_motors = list(hand.g_jointpositions)
    hand.joint_update(hand.calibrationfistpose)
    fist_motors = list(hand.g_jointpositions)

    assert getattr(hand.handcore, last_qpos_attr) is not None
    assert hand.g_jointpositions_arc[0] == pytest.approx(hand.effective_robot_fist[0])
    assert hand.g_jointpositions_arc[1] == pytest.approx(hand.effective_robot_fist[1])
    for motor_idx in (5, 6, 7, 8, 9, 10):
        assert open_motors[motor_idx] == 0
        assert 0 <= fist_motors[motor_idx] <= 255
    assert opose_motors[5] == 165
    assert opose_motors[10] == 138


def test_o20_motor_mapping_uses_two_point_mapping_and_thumb_three_point_calibration():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False

    qpos = _effective_qpos_from_hand(hand, "opose")
    hand.g_jointpositions = _map_o20_qpos_to_motor(
        hand.handcore,
        qpos,
        "right",
        hand.effective_robot_original,
        hand.effective_robot_fist,
    )
    before_thumb_calibration = list(hand.g_jointpositions)
    hand._apply_thumb_motor_calibration(qpos)

    assert 0 <= before_thumb_calibration[6] <= 255
    assert hand.g_jointpositions[5] == 165
    assert hand.g_jointpositions[10] == 138


def test_o20_outputs_urdf_arcs_with_mujoco_display_mirroring():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_arcs = list(hand.g_jointpositions_arc)
    hand.joint_update(hand.calibrationfistpose)
    fist_arcs = list(hand.g_jointpositions_arc)

    assert all(sign == 1.0 for sign in O20_MUJOCO_JOINT_ARC_SIGNS)
    assert open_arcs[5] <= fist_arcs[5]
    for arc_idx in (0, 9, 13, 17):
        assert -1.57 <= open_arcs[arc_idx] <= 1.57
        assert -1.57 <= fist_arcs[arc_idx] <= 1.57
    assert open_arcs[1] == pytest.approx(ROBOT_ORIGINAL_RIGHT[1])
    assert fist_arcs[1] == pytest.approx(ROBOT_FIST_RIGHT[1])
    assert open_arcs[1] > fist_arcs[1]


def test_o20_mujoco_display_clamps_mapped_yaw_to_urdf_limits_without_mirroring():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.g_jointpositions_arc[1] = 9.0
    positions = extract_mujoco_joint_positions(
        None,
        "right",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )

    assert positions["thumb_cmc_yaw"] == pytest.approx(1.57)


def test_o20_mujoco_display_maps_right_thumb_cmc_roll_through_configured_anchors():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_positions = extract_mujoco_joint_positions(
        None,
        "right",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )
    hand.joint_update(hand.calibrationfistpose)
    fist_positions = extract_mujoco_joint_positions(
        None,
        "right",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )
    hand.joint_update(hand.calibrationopose)
    positions = extract_mujoco_joint_positions(
        None,
        "right",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )

    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_OPOSE_RIGHT[0])
    assert open_positions["thumb_cmc_roll"] == pytest.approx(ROBOT_ORIGINAL_RIGHT[0])
    assert positions["thumb_cmc_roll"] == pytest.approx(ROBOT_OPOSE_RIGHT[0])
    assert fist_positions["thumb_cmc_roll"] == pytest.approx(ROBOT_FIST_RIGHT[0])


def test_o20_left_mujoco_display_preserves_thumb_cmc_yaw_direction():
    hand = LeftHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_positions = extract_mujoco_joint_positions(
        None,
        "left",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )
    hand.joint_update(hand.calibrationopose)
    opose_positions = extract_mujoco_joint_positions(
        None,
        "left",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )
    hand.joint_update(hand.calibrationfistpose)
    fist_positions = extract_mujoco_joint_positions(
        None,
        "left",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )

    assert open_positions["thumb_cmc_yaw"] == pytest.approx(ROBOT_ORIGINAL_LEFT[1])
    assert opose_positions["thumb_cmc_yaw"] == pytest.approx(ROBOT_OPOSE_LEFT[1])
    assert fist_positions["thumb_cmc_yaw"] == pytest.approx(ROBOT_FIST_LEFT[1])


def test_o20_mujoco_sync_uses_real_urdf_joint_order_for_thumb_cmc_roll():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()
    urdf_path = (
        Path(__file__).parents[2]
        / "linkerhand_retarget"
        / "assets"
        / "robots"
        / "hands"
        / "linker_hand"
        / "o20_right"
        / "linkerhand_o20_right.urdf"
    )
    joint_names = get_urdf_movable_joint_names(urdf_path)

    expected = (
        (hand.calibrationoriginal, ROBOT_ORIGINAL_RIGHT[0]),
        (hand.calibrationopose, ROBOT_OPOSE_RIGHT[0]),
        (hand.calibrationfistpose, ROBOT_FIST_RIGHT[0]),
    )
    for glove_state, expected_roll in expected:
        hand.joint_update(glove_state)
        positions = extract_mujoco_joint_positions(
            None,
            "right",
            joint_names,
            hand_model=hand,
        )
        assert positions["thumb_cmc_roll"] == pytest.approx(expected_roll)


def test_o20_uses_open_opose_state_order_while_extending_thumb_roll_to_fist():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    assert hand.multi_state_mapper.state_order == ["original", "opose"]

    hand.joint_update(hand.calibrationfistpose)
    positions = extract_mujoco_joint_positions(
        None,
        "right",
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand_model=hand,
    )

    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_FIST_RIGHT[0])
    assert positions["thumb_cmc_roll"] == pytest.approx(ROBOT_FIST_RIGHT[0])


@pytest.mark.parametrize(
    ("hand_cls", "arc_expectations"),
    [
        (
            RightHand,
            (
                (5, ROBOT_FIST_RIGHT[4]),
                (9, ROBOT_FIST_RIGHT[7]),
                (13, -0.21),
                (17, ROBOT_ORIGINAL_RIGHT[13]),
            ),
        ),
        (
            LeftHand,
            (
                (5, ROBOT_FIST_LEFT[4]),
                (9, ROBOT_FIST_LEFT[7]),
                (13, ROBOT_FIST_LEFT[10]),
                (17, ROBOT_FIST_LEFT[13]),
            ),
        ),
    ],
)
def test_o20_four_finger_rolls_extend_to_fist_targets(hand_cls, arc_expectations):
    hand = hand_cls(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationfistpose)

    for arc_idx, expected in arc_expectations:
        assert hand.g_jointpositions_arc[arc_idx] == pytest.approx(expected)


def test_o20_right_four_finger_rolls_progress_toward_fist_before_exact_fist():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    near_fist = [1.5] * 21
    hand.joint_update(near_fist)

    assert hand.g_jointpositions_arc[5] > ROBOT_OPOSE_RIGHT[4]
    assert hand.g_jointpositions_arc[13] < 0.0
    assert hand.g_jointpositions_arc[17] > ROBOT_OPOSE_RIGHT[13]
    assert hand.g_jointpositions_arc[5] < ROBOT_FIST_RIGHT[4]
    assert hand.g_jointpositions_arc[13] > -0.21
    assert hand.g_jointpositions_arc[17] < ROBOT_FIST_RIGHT[13]


def test_o20_right_ring_roll_mujoco_output_uses_reversed_zero_to_open_range():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_ring = hand.g_jointpositions_arc[13]
    hand.joint_update([1.5] * 21)
    near_fist_ring = hand.g_jointpositions_arc[13]
    hand.joint_update(hand.calibrationfistpose)
    fist_ring = hand.g_jointpositions_arc[13]

    assert open_ring == pytest.approx(0.0)
    assert -0.21 < near_fist_ring < 0.0
    assert fist_ring == pytest.approx(-0.21)


def test_o20_right_pinky_roll_mujoco_output_uses_reversed_zero_to_open_range():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_pinky = hand.g_jointpositions_arc[17]
    hand.joint_update([1.5] * 21)
    near_fist_pinky = hand.g_jointpositions_arc[17]
    hand.joint_update(hand.calibrationfistpose)
    fist_pinky = hand.g_jointpositions_arc[17]

    assert open_pinky == pytest.approx(0.0)
    assert ROBOT_ORIGINAL_RIGHT[13] < near_fist_pinky < 0.0
    assert fist_pinky == pytest.approx(ROBOT_ORIGINAL_RIGHT[13])


@pytest.mark.parametrize(
    ("hand_cls", "arc_expectations"),
    [
        (
            RightHand,
            (
                (5, ROBOT_FIST_RIGHT[4]),
                (9, ROBOT_FIST_RIGHT[7]),
                (13, -0.21),
                (17, ROBOT_ORIGINAL_RIGHT[13]),
            ),
        ),
        (
            LeftHand,
            (
                (5, ROBOT_FIST_LEFT[4]),
                (9, ROBOT_FIST_LEFT[7]),
                (13, ROBOT_FIST_LEFT[10]),
                (17, ROBOT_FIST_LEFT[13]),
            ),
        ),
    ],
)
def test_o20_calibration_fist_pose_bypasses_arc_smoothing_for_mujoco(
    hand_cls, arc_expectations
):
    hand = hand_cls(FakeHandCore())
    hand.smooth_enabled = True
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    hand.joint_update(hand.calibrationfistpose)

    for arc_idx, expected in arc_expectations:
        assert hand.g_jointpositions_arc[arc_idx] == pytest.approx(expected)


def test_o20_right_calibration_poses_reset_thumb_roll_arc_smoothing_for_mujoco():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = True
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationfistpose)
    fist_roll = hand.g_jointpositions_arc[0]
    hand.joint_update(hand.calibrationoriginal)
    open_roll = hand.g_jointpositions_arc[0]
    hand.joint_update(hand.calibrationopose)
    opose_roll = hand.g_jointpositions_arc[0]

    assert fist_roll == pytest.approx(ROBOT_FIST_RIGHT[0])
    assert open_roll == pytest.approx(ROBOT_ORIGINAL_RIGHT[0])
    assert opose_roll == pytest.approx(ROBOT_OPOSE_RIGHT[0])


def test_o20_mujoco_display_thumb_cmc_roll_remap_uses_opose_anchor():
    hand = RightHand(FakeHandCore())
    expected = (
        (ROBOT_ORIGINAL_RIGHT[0], ROBOT_ORIGINAL_RIGHT[0]),
        (ROBOT_OPOSE_RIGHT[0], ROBOT_OPOSE_RIGHT[0]),
        (ROBOT_FIST_RIGHT[0], ROBOT_FIST_RIGHT[0]),
    )

    assert len(hand.mujoco_joint_arc_remaps[0]) == len(expected)
    for actual_pair, expected_pair in zip(hand.mujoco_joint_arc_remaps[0], expected):
        assert tuple(actual_pair) == pytest.approx(expected_pair)


def test_o20_right_thumb_roll_preserves_exact_calibration_pose_outputs():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = [0.0] * 21
    hand.calibrationopose = [0.0] * 21
    hand.calibrationfistpose = [0.0] * 21
    hand.calibrationoriginal[0] = 4.007191280476122
    hand.calibrationopose[0] = 3.9944501936546426
    hand.calibrationfistpose[0] = 3.988079650243903
    hand.calibrationoriginal[1] = 4.977160939679779
    hand.calibrationopose[1] = 4.8031533404893425
    hand.calibrationfistpose[1] = 4.716149540894124
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_ORIGINAL_RIGHT[0])

    hand.joint_update(hand.calibrationopose)
    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_OPOSE_RIGHT[0])

    hand.joint_update(hand.calibrationfistpose)
    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_FIST_RIGHT[0])


def test_o20_right_thumb_roll_maps_runtime_raw_direction_to_open_and_fist():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = [0.0] * 21
    hand.calibrationopose = [0.0] * 21
    hand.calibrationfistpose = [0.0] * 21
    hand.calibrationoriginal[1] = 4.977160939679779
    hand.calibrationopose[1] = 4.8031533404893425
    hand.calibrationfistpose[1] = 4.716149540894124
    hand.initialize_mapper()

    live_open = list(hand.calibrationoriginal)
    live_open[1] = 4.172578477966643
    hand.joint_update(live_open)
    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_ORIGINAL_RIGHT[0])

    live_fist = list(hand.calibrationoriginal)
    live_fist[1] = 5.180927965710036
    hand.joint_update(live_fist)
    assert hand.g_jointpositions_arc[0] == pytest.approx(ROBOT_FIST_RIGHT[0])


def test_o20_clamps_mapped_arc_values_to_urdf_limits_before_output():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = [0.0] * 21
    hand.calibrationopose = [1.0] * 21
    hand.calibrationfistpose = [2.0] * 21
    hand.robot_fist = list(hand.robot_fist)
    hand.robot_fist[2] = 9.0
    hand.effective_robot_fist = list(hand.effective_robot_fist)
    hand.effective_robot_fist[2] = 9.0
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationfistpose)

    assert hand.g_jointpositions_arc[2] <= 1.17
