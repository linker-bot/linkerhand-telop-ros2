from types import SimpleNamespace

import pytest

from linkerhand_retarget.mujoco_display import extract_mujoco_joint_positions
from linkerhand_retarget.motion.linkerforce.hand import (
    linkerforce_g20,
    linkerforce_l10,
    linkerforce_l20,
    linkerforce_l6,
    linkerforce_o6,
)


class FakeHandCore:
    pass


@pytest.mark.parametrize(
    "hand_cls, joint_names, expected",
    [
        (
            linkerforce_o6.RightHand,
            (
                "thumb_cmc_yaw",
                "thumb_cmc_pitch",
                "thumb_ip",
                "index_mcp_pitch",
                "index_dip",
                "middle_mcp_pitch",
                "middle_dip",
                "ring_mcp_pitch",
                "ring_dip",
                "pinky_mcp_pitch",
                "pinky_dip",
            ),
            {
                "thumb_cmc_yaw": 1.0,
                "thumb_cmc_pitch": 0.0,
                "thumb_ip": 0.0,
                "index_mcp_pitch": 2.0,
                "index_dip": 2.0,
                "middle_mcp_pitch": 3.0,
                "middle_dip": 3.0,
                "ring_mcp_pitch": 4.0,
                "ring_dip": 4.0,
                "pinky_mcp_pitch": 5.0,
                "pinky_dip": 5.0,
            },
        ),
        (
            linkerforce_l6.RightHand,
            (
                "thumb_cmc_roll",
                "thumb_cmc_pitch",
                "thumb_dip",
                "index_mcp_pitch",
                "index_dip",
                "middle_mcp_pitch",
                "middle_dip",
                "ring_mcp_pitch",
                "ring_dip",
                "pinky_mcp_pitch",
                "pinky_dip",
            ),
            {
                "thumb_cmc_roll": 1.0,
                "thumb_cmc_pitch": 0.0,
                "thumb_dip": 0.0,
                "index_mcp_pitch": 2.0,
                "index_dip": 2.0,
                "middle_mcp_pitch": 3.0,
                "middle_dip": 3.0,
                "ring_mcp_pitch": 4.0,
                "ring_dip": 4.0,
                "pinky_mcp_pitch": 5.0,
                "pinky_dip": 5.0,
            },
        ),
        (
            linkerforce_l10.RightHand,
            (
                "thumb_cmc_roll",
                "thumb_cmc_yaw",
                "thumb_cmc_pitch",
                "thumb_mcp",
                "thumb_ip",
                "index_mcp_roll",
                "index_mcp_pitch",
                "index_pip",
                "index_dip",
                "middle_mcp_pitch",
                "middle_pip",
                "middle_dip",
                "ring_mcp_roll",
                "ring_mcp_pitch",
                "ring_pip",
                "ring_dip",
                "pinky_mcp_roll",
                "pinky_mcp_pitch",
                "pinky_pip",
                "pinky_dip",
            ),
            {
                "thumb_cmc_roll": 9.0,
                "thumb_cmc_yaw": 1.0,
                "thumb_cmc_pitch": 0.0,
                "thumb_mcp": 0.0,
                "thumb_ip": 0.0,
                "index_mcp_roll": 6.0,
                "index_mcp_pitch": 2.0,
                "index_pip": 2.0,
                "index_dip": 2.0,
                "middle_mcp_pitch": 3.0,
                "middle_pip": 3.0,
                "middle_dip": 3.0,
                "ring_mcp_roll": 7.0,
                "ring_mcp_pitch": 4.0,
                "ring_pip": 4.0,
                "ring_dip": 4.0,
                "pinky_mcp_roll": 8.0,
                "pinky_mcp_pitch": 5.0,
                "pinky_pip": 5.0,
                "pinky_dip": 5.0,
            },
        ),
        (
            linkerforce_g20.RightHand,
            (
                "thumb_cmc_roll",
                "thumb_cmc_yaw",
                "thumb_cmc_pitch",
                "thumb_mcp",
                "thumb_ip",
                "index_mcp_roll",
                "index_mcp_pitch",
                "index_pip",
                "index_dip",
                "middle_mcp_roll",
                "middle_mcp_pitch",
                "middle_pip",
                "middle_dip",
                "ring_mcp_roll",
                "ring_mcp_pitch",
                "ring_pip",
                "ring_dip",
                "pinky_mcp_roll",
                "pinky_mcp_pitch",
                "pinky_pip",
                "pinky_dip",
            ),
            {
                "thumb_cmc_roll": 0.0,
                "thumb_cmc_yaw": 1.0,
                "thumb_cmc_pitch": 2.0,
                "thumb_mcp": 3.0,
                "thumb_ip": 3.0,
                "index_mcp_roll": 5.0,
                "index_mcp_pitch": 6.0,
                "index_pip": 7.0,
                "index_dip": 7.0,
                "middle_mcp_roll": 9.0,
                "middle_mcp_pitch": 10.0,
                "middle_pip": 11.0,
                "middle_dip": 11.0,
                "ring_mcp_roll": 13.0,
                "ring_mcp_pitch": 14.0,
                "ring_pip": 15.0,
                "ring_dip": 15.0,
                "pinky_mcp_roll": 17.0,
                "pinky_mcp_pitch": 18.0,
                "pinky_pip": 19.0,
                "pinky_dip": 19.0,
            },
        ),
        (
            linkerforce_l20.RightHand,
            (
                "thumb_cmc_roll",
                "thumb_cmc_yaw",
                "thumb_cmc_pitch",
                "thumb_mcp",
                "thumb_ip",
                "index_mcp_roll",
                "index_mcp_pitch",
                "index_pip",
                "index_dip",
                "middle_mcp_roll",
                "middle_mcp_pitch",
                "middle_pip",
                "middle_dip",
                "ring_mcp_roll",
                "ring_mcp_pitch",
                "ring_pip",
                "ring_dip",
                "pinky_mcp_roll",
                "pinky_mcp_pitch",
                "pinky_pip",
                "pinky_dip",
            ),
            {
                "thumb_cmc_roll": 0.0,
                "thumb_cmc_yaw": 1.0,
                "thumb_cmc_pitch": 2.0,
                "thumb_mcp": 3.0,
                "thumb_ip": 3.0,
                "index_mcp_roll": 5.0,
                "index_mcp_pitch": 6.0,
                "index_pip": 7.0,
                "index_dip": 7.0,
                "middle_mcp_roll": 9.0,
                "middle_mcp_pitch": 10.0,
                "middle_pip": 11.0,
                "middle_dip": 11.0,
                "ring_mcp_roll": 13.0,
                "ring_mcp_pitch": 14.0,
                "ring_pip": 15.0,
                "ring_dip": 15.0,
                "pinky_mcp_roll": 17.0,
                "pinky_mcp_pitch": 18.0,
                "pinky_pip": 19.0,
                "pinky_dip": 19.0,
            },
        ),
    ],
)
def test_linkerforce_mujoco_display_maps_active_arcs_to_urdf_joints(
    hand_cls, joint_names, expected
):
    hand = hand_cls(FakeHandCore())
    hand.g_jointpositions_arc = [float(index) for index in range(len(hand.g_jointpositions_arc))]

    positions = extract_mujoco_joint_positions(
        None,
        "right",
        joint_names,
        hand_model=hand,
    )

    assert positions == expected


def test_g20_l20_mujoco_display_routes_passive_distal_joints():
    joint_names = tuple(f"joint_{index}" for index in range(21))

    for hand in (
        linkerforce_g20.RightHand(FakeHandCore()),
        linkerforce_g20.LeftHand(FakeHandCore()),
        linkerforce_l20.RightHand(FakeHandCore()),
        linkerforce_l20.LeftHand(FakeHandCore()),
    ):
        hand.g_jointpositions_arc = [float(index) for index in range(20)]

        positions = extract_mujoco_joint_positions(
            None,
            "right",
            joint_names,
            hand_model=hand,
        )

        assert positions["joint_4"] == 3.0
        assert positions["joint_8"] == 7.0
        assert positions["joint_12"] == 11.0
        assert positions["joint_16"] == 15.0
        assert positions["joint_20"] == 19.0


def test_l20_left_uses_same_order_with_numbered_urdf_joint_names():
    hand = linkerforce_l20.LeftHand(FakeHandCore())
    hand.g_jointpositions_arc = [float(index) for index in range(20)]
    joint_names = (
        "thumb_joint0",
        "thumb_joint1",
        "thumb_joint2",
        "thumb_joint3",
        "thumb_joint4",
        "index_joint0",
        "index_joint1",
        "index_joint2",
        "index_joint3",
        "middle_joint0",
        "middle_joint1",
        "middle_joint2",
        "middle_joint3",
        "ring_joint0",
        "ring_joint1",
        "ring_joint2",
        "ring_joint3",
        "little_joint0",
        "little_joint1",
        "little_joint2",
        "little_joint3",
    )

    positions = extract_mujoco_joint_positions(
        None,
        "left",
        joint_names,
        hand_model=hand,
    )

    assert positions["middle_joint0"] == 9.0
    assert positions["little_joint3"] == 19.0
