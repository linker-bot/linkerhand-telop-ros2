import pytest

from linkerhand_retarget.motion.linkerforce.hand.simple_linear_mapper import SimpleLinearMapper


def test_maps_negative_to_positive_source_without_losing_direction():
    mapper = SimpleLinearMapper(
        {
            "joint": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
            }
        },
        ["joint"],
    )
    mapper.add_state("original", [-1.0], [1.57])
    mapper.add_state("opose", [1.0], [0.56])
    mapper.set_state_order(["original", "opose"])

    assert mapper.map_glove_to_robot([-1.0])[0] == pytest.approx(1.57)
    assert mapper.map_glove_to_robot([0.0])[0] == pytest.approx(1.065)
    assert mapper.map_glove_to_robot([1.0])[0] == pytest.approx(0.56)


def test_piecewise_three_state_mapping_preserves_target_direction():
    mapper = SimpleLinearMapper(
        {
            "joint": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
            }
        },
        ["joint"],
    )
    mapper.add_state("original", [0.0], [-0.5])
    mapper.add_state("opose", [1.0], [0.25])
    mapper.add_state("fist", [2.0], [-0.1])
    mapper.set_state_order(["original", "opose", "fist"])

    assert mapper.map_glove_to_robot([0.5])[0] == pytest.approx(-0.125)
    assert mapper.map_glove_to_robot([1.5])[0] == pytest.approx(0.075)


def test_per_joint_state_order_can_use_open_to_fist_range():
    mapper = SimpleLinearMapper(
        {
            "roll": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
                "state_order": ["original", "fist"],
            }
        },
        ["roll"],
    )
    mapper.add_state("original", [10.0], [-0.35])
    mapper.add_state("opose", [12.0], [-0.13])
    mapper.add_state("fist", [20.0], [0.09])
    mapper.set_state_order(["original", "opose"])

    assert mapper.map_glove_to_robot([15.0])[0] == pytest.approx(-0.13)


def test_extended_two_state_mapping_uses_fist_anchor_when_available():
    mapper = SimpleLinearMapper(
        {
            "flex": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
                "extended_mapping": {"enabled": True},
            }
        },
        ["flex"],
    )
    mapper.add_state("original", [0.0], [0.0])
    mapper.add_state("opose", [1.0], [0.75])
    mapper.add_state("fist", [2.0], [1.85])
    mapper.set_state_order(["original", "opose"])

    assert mapper.map_glove_to_robot([1.5])[0] == pytest.approx(1.125)


def test_extended_two_state_mapping_applies_scale_factor_before_opose():
    mapper = SimpleLinearMapper(
        {
            "flex": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
                "extended_mapping": {
                    "enabled": True,
                    "scale_factor": 1.2,
                },
            }
        },
        ["flex"],
    )
    mapper.add_state("original", [0.0], [0.0])
    mapper.add_state("opose", [1.0], [10.0])
    mapper.add_state("fist", [2.0], [30.0])
    mapper.set_state_order(["original", "opose"])

    assert mapper.map_glove_to_robot([0.5])[0] == pytest.approx(6.0)
    assert mapper.map_glove_to_robot([1.0])[0] == pytest.approx(10.0)


def test_extended_two_state_mapping_applies_exp_factor_after_opose():
    mapper = SimpleLinearMapper(
        {
            "flex": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
                "extended_mapping": {
                    "enabled": True,
                    "extended_exp_factor": 3.0,
                },
            }
        },
        ["flex"],
    )
    mapper.add_state("original", [0.0], [0.0])
    mapper.add_state("opose", [1.0], [10.0])
    mapper.add_state("fist", [2.0], [30.0])
    mapper.set_state_order(["original", "opose"])

    assert mapper.map_glove_to_robot([1.2])[0] == pytest.approx(12.8)
    assert mapper.map_glove_to_robot([3.0])[0] == pytest.approx(30.0)


def test_debug_values_track_mapped_robot_angles():
    mapper = SimpleLinearMapper(
        {
            "joint": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 3,
            }
        },
        ["joint"],
    )
    mapper.add_state("original", [0.0], [0.0, 0.0, 0.0, -1.0])
    mapper.add_state("opose", [1.0], [0.0, 0.0, 0.0, 1.0])
    mapper.set_state_order(["original", "opose"])

    mapper.map_glove_to_robot([0.75])

    assert mapper.debug_value[3] == pytest.approx(0.5)


def test_optional_kalman_filter_smooths_input_before_mapping():
    mapper = SimpleLinearMapper(
        {
            "joint": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
            }
        },
        ["joint"],
    )
    mapper.add_state("original", [0.0], [0.0])
    mapper.add_state("opose", [1.0], [1.0])
    mapper.set_state_order(["original", "opose"])

    first = mapper.map_glove_to_robot([0.5], use_filter=True)[0]
    second = mapper.map_glove_to_robot([0.7], use_filter=True)[0]

    assert first == pytest.approx(0.5)
    assert 0.5 < second < 0.7
    assert mapper.last_filtered_glove[0] == pytest.approx(second)


def test_reverse_output_direction_flips_mapped_arc_without_display_signs():
    mapper = SimpleLinearMapper(
        {
            "roll": {
                "joints": [0],
                "weights": [1.0],
                "robot_idx": 0,
                "reverse_output_direction": True,
            }
        },
        ["roll"],
    )
    mapper.add_state("original", [0.0], [-0.26])
    mapper.add_state("fist", [2.0], [0.26])
    mapper.set_state_order(["original", "fist"])

    assert mapper.map_glove_to_robot([0.0])[0] == pytest.approx(0.26)
    assert mapper.map_glove_to_robot([1.0])[0] == pytest.approx(0.0)
    assert mapper.map_glove_to_robot([2.0])[0] == pytest.approx(-0.26)
