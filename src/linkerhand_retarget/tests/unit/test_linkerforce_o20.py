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

    assert hand.g_jointpositions_arc[1] == pytest.approx(hand.effective_robot_opose[1])


def test_o20_right_thumb_cmc_yaw_arc_is_smoothed():
    hand = RightHand(FakeHandCore())
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _calibration(
        4.526575977103334,
        4.760386137053435,
        5.205488511599119,
    )
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationopose)
    first_frame = hand.g_jointpositions_arc[1]
    hand.joint_update(hand.calibrationopose)

    expected_opose_yaw = hand.effective_robot_opose[1]

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

    assert hand.g_jointpositions_arc[0] != pytest.approx(baseline_roll)
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

    assert hand.g_jointpositions_arc[1] == pytest.approx(hand.effective_robot_opose[1])


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
        assert fist_motors[motor_idx] == 255
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

    assert before_thumb_calibration[6] == pytest.approx(127, abs=1)
    assert hand.g_jointpositions[5] == 165
    assert hand.g_jointpositions[10] == 138


def test_o20_corrects_reversed_arcs_before_mujoco_display():
    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_arcs = list(hand.g_jointpositions_arc)
    hand.joint_update(hand.calibrationfistpose)
    fist_arcs = list(hand.g_jointpositions_arc)

    assert all(sign == 1.0 for sign in O20_MUJOCO_JOINT_ARC_SIGNS)
    assert open_arcs[5] < fist_arcs[5]
    for arc_idx in (0, 9, 13, 17):
        assert open_arcs[arc_idx] > fist_arcs[arc_idx]
    assert open_arcs[1] < fist_arcs[1]
