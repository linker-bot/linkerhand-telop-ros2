import pytest
import yaml
import xml.etree.ElementTree as ET
from pathlib import Path
from linkerhand_retarget.linkerhand.constants import (
    RobotName,
    RetargetingType,
    HandType,
    DataSource,
    MotionSource,
    ROBOT_NAME_MAP,
    ROBOT_LEN_MAP,
    OPERATOR2MANO,
    OPERATOR2MANO_RIGHT,
    OPERATOR2MANO_LEFT,
    get_default_config_path,
)


def _optional_command(value):
    return None if value == "None" else value


O20_ROBOT_IDX_TO_URDF_IDX = [0, 1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19]


class TestRobotName:
    def test_enum_values(self):
        assert RobotName.o7.value is not None
        assert RobotName.l6.value is not None
        assert RobotName.l20.value is not None
        assert RobotName.o20.value is not None
        assert RobotName.o30.value is not None
        assert RobotName.o30i.value is not None

    def test_robot_names_list(self):
        from linkerhand_retarget.linkerhand.constants import ROBOT_NAMES
        assert len(ROBOT_NAMES) > 0
        assert RobotName.o7 in ROBOT_NAMES


class TestRetargetingType:
    def test_enum_values(self):
        assert RetargetingType.vector is not None
        assert RetargetingType.position is not None
        assert RetargetingType.dexpilot is not None
        assert RetargetingType.projection is not None


class TestHandType:
    def test_enum_values(self):
        assert HandType.right is not None
        assert HandType.left is not None


class TestDataSource:
    def test_enum_values(self):
        assert DataSource.motion is not None
        assert DataSource.video is not None
        assert DataSource.vr is not None


class TestMotionSource:
    def test_enum_values(self):
        assert MotionSource.vtrdyn is not None
        assert MotionSource.udexreal is not None
        assert MotionSource.linkerforce is not None


class TestRobotNameMap:
    def test_robot_name_map(self):
        assert ROBOT_NAME_MAP[RobotName.o7] == "linker_hand_o7"
        assert ROBOT_NAME_MAP[RobotName.l6] == "linker_hand_l6"
        assert ROBOT_NAME_MAP[RobotName.l20] == "linker_hand_l20"
        assert ROBOT_NAME_MAP[RobotName.l25] == "linker_hand_l25"
        assert ROBOT_NAME_MAP[RobotName.o20] == "linker_hand_o20"
        assert ROBOT_NAME_MAP[RobotName.o30] == "linker_hand_o30"
        assert ROBOT_NAME_MAP[RobotName.o30i] == "linker_hand_o30i"

    def test_robot_len_map(self):
        assert ROBOT_LEN_MAP[RobotName.o7] == 7
        assert ROBOT_LEN_MAP[RobotName.l6] == 6
        assert ROBOT_LEN_MAP[RobotName.l20] == 20
        assert ROBOT_LEN_MAP[RobotName.l25] == 20
        assert ROBOT_LEN_MAP[RobotName.o20] == 20
        assert ROBOT_LEN_MAP[RobotName.o30] == 20
        assert ROBOT_LEN_MAP[RobotName.o30i] == 20

    @pytest.mark.parametrize("robot_name", ["o20", "o30", "o30i"])
    def test_independent_20_dof_model_runtime_assets(self, robot_name):
        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        hand_config_path = package_dir / "config" / "hand_config.yml"
        hand_config = yaml.safe_load(hand_config_path.read_text())

        for side in ("right", "left"):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"{robot_name}_{side}"
                / f"linkerhand_{robot_name}_{side}.urdf"
            )
            assert urdf_path.exists()
            assert hand_config[f"commandlower_{side}_{robot_name}"]
            assert hand_config[f"commandupper_{side}_{robot_name}"]
            assert hand_config[f"commandsourcedataindex_{side}_{robot_name}"]
            assert hand_config[f"urdfdataindex_{side}_{robot_name}"]

    def test_o30i_hand_config_uses_20_dof_urdf_and_g20_motor_template(self):
        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        hand_config_path = package_dir / "config" / "hand_config.yml"
        hand_config = yaml.safe_load(hand_config_path.read_text())

        for side in ("right", "left"):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o30i_{side}"
                / f"linkerhand_o30i_{side}.urdf"
            )
            movable_joints = [
                joint
                for joint in ET.parse(urdf_path).getroot().findall("joint")
                if joint.attrib.get("type") != "fixed"
            ]

            assert len(movable_joints) == 20
            assert hand_config[f"commandlower_{side}_o30i"] == hand_config[f"commandlower_{side}_g20"]
            assert hand_config[f"commandupper_{side}_o30i"] == hand_config[f"commandupper_{side}_g20"]
            assert hand_config[f"commandsourcedataindex_{side}_o30i"] == hand_config[f"commandsourcedataindex_{side}_g20"]
            assert len(hand_config[f"urdfdataindex_{side}_o30i"]) == 20

    def test_o20_motor_commands_open_to_fist_direction(self):
        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        hand_config_path = package_dir / "config" / "hand_config.yml"
        hand_config = yaml.safe_load(hand_config_path.read_text())

        for side in ("right", "left"):
            lower = hand_config[f"commandlower_{side}_o20"]
            upper = hand_config[f"commandupper_{side}_o20"]

            for raw_lower_value, raw_upper_value in zip(lower, upper):
                lower_value = _optional_command(raw_lower_value)
                upper_value = _optional_command(raw_upper_value)
                if lower_value is None:
                    assert upper_value is None
                else:
                    assert lower_value == 0
                    assert upper_value == 255

    def test_o20_hand_config_urdf_indices_match_current_urdf(self):
        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        hand_config_path = package_dir / "config" / "hand_config.yml"
        hand_config = yaml.safe_load(hand_config_path.read_text())

        for side in ("right", "left"):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o20_{side}"
                / f"linkerhand_o20_{side}.urdf"
            )
            movable_joints = [
                joint
                for joint in ET.parse(urdf_path).getroot().findall("joint")
                if joint.attrib.get("type") != "fixed"
            ]
            source_indices = hand_config[f"commandsourcedataindex_{side}_o20"]
            urdf_indices = hand_config[f"urdfdataindex_{side}_o20"]

            assert len(source_indices) == 20
            assert len(urdf_indices) == 20
            for source_idx, urdf_idx in zip(source_indices, urdf_indices):
                source_idx = _optional_command(source_idx)
                urdf_idx = _optional_command(urdf_idx)
                if source_idx is None:
                    assert urdf_idx is None
                else:
                    assert 0 <= urdf_idx < len(movable_joints)

    def test_o20_handcore_keeps_latest_qpos_for_mujoco_display(self):
        from linkerhand_retarget.linkerhand.config import HandConfig
        from linkerhand_retarget.linkerhand.handcore import HandCore

        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        robot_dir = package_dir / "assets" / "robots" / "hands"
        handcore = HandCore(HandConfig(str(robot_dir), str(package_dir)))
        qpos = [float(index) for index in range(25)]

        handcore.trans_to_motor_right(qpos)
        handcore.trans_to_motor_left(qpos)

        assert handcore.last_qpos_r == qpos
        assert handcore.last_qpos_l == qpos

    def test_o20_thumb_rotate_uses_o20_joint_range(self):
        from linkerhand_retarget.motion.linkerforce.config.o20_config import (
            ROBOT_FIST_LEFT,
            ROBOT_FIST_RIGHT,
            ROBOT_OPOSE_LEFT,
            ROBOT_OPOSE_RIGHT,
            ROBOT_ORIGINAL_LEFT,
            ROBOT_ORIGINAL_RIGHT,
        )

        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"

        for side, original, opose, fist in (
            ("right", ROBOT_ORIGINAL_RIGHT, ROBOT_OPOSE_RIGHT, ROBOT_FIST_RIGHT),
            ("left", ROBOT_ORIGINAL_LEFT, ROBOT_OPOSE_LEFT, ROBOT_FIST_LEFT),
        ):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o20_{side}"
                / f"linkerhand_o20_{side}.urdf"
            )
            thumb_joint = ET.parse(urdf_path).getroot().find("./joint[@name='thumb_cmc_roll']")
            joint_limit = thumb_joint.find("limit")
            lower = float(joint_limit.attrib["lower"])
            upper = float(joint_limit.attrib["upper"])

            assert lower <= original[0] <= upper
            assert lower <= opose[0] <= upper
            assert lower <= fist[0] <= upper

    def test_o20_thumb_cmc_roll_open_and_fist_are_valid_urdf_targets(self):
        from linkerhand_retarget.motion.linkerforce.config.o20_config import (
            ROBOT_FIST_LEFT,
            ROBOT_FIST_RIGHT,
            ROBOT_ORIGINAL_LEFT,
            ROBOT_ORIGINAL_RIGHT,
        )

        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"

        for side, original, fist in (
            ("right", ROBOT_ORIGINAL_RIGHT, ROBOT_FIST_RIGHT),
            ("left", ROBOT_ORIGINAL_LEFT, ROBOT_FIST_LEFT),
        ):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o20_{side}"
                / f"linkerhand_o20_{side}.urdf"
            )
            roll_limit = ET.parse(urdf_path).getroot().find("./joint[@name='thumb_cmc_roll']").find("limit")
            lower = float(roll_limit.attrib["lower"])
            upper = float(roll_limit.attrib["upper"])

            assert lower <= original[0] <= upper
            assert lower <= fist[0] <= upper

    def test_o20_open_and_fist_pose_presets_match_motor_extremes(self):
        from linkerhand_retarget.motion.linkerforce.config.o20_config import (
            ROBOT_FIST_LEFT,
            ROBOT_FIST_RIGHT,
            ROBOT_ORIGINAL_LEFT,
            ROBOT_ORIGINAL_RIGHT,
        )

        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"

        for side, original, fist in (
            ("right", ROBOT_ORIGINAL_RIGHT, ROBOT_FIST_RIGHT),
            ("left", ROBOT_ORIGINAL_LEFT, ROBOT_FIST_LEFT),
        ):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o20_{side}"
                / f"linkerhand_o20_{side}.urdf"
            )
            movable_joints = [
                joint
                for joint in ET.parse(urdf_path).getroot().findall("joint")
                if joint.attrib.get("type") != "fixed"
            ]

            for robot_idx, joint in enumerate(movable_joints):
                joint_limit = joint.find("limit")
                lower = float(joint_limit.attrib["lower"])
                upper = float(joint_limit.attrib["upper"])
                assert lower <= original[robot_idx] <= upper
                assert lower <= fist[robot_idx] <= upper

    def test_o20_fist_four_finger_pitch_and_dip_are_valid_urdf_targets(self):
        from linkerhand_retarget.motion.linkerforce.config.o20_config import (
            ROBOT_FIST_LEFT,
            ROBOT_FIST_RIGHT,
        )

        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        four_finger_pitch_dip_indices = (5, 6, 8, 9, 11, 12, 14, 15)

        for side, fist in (
            ("right", ROBOT_FIST_RIGHT),
            ("left", ROBOT_FIST_LEFT),
        ):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o20_{side}"
                / f"linkerhand_o20_{side}.urdf"
            )
            movable_joints = [
                joint
                for joint in ET.parse(urdf_path).getroot().findall("joint")
                if joint.attrib.get("type") != "fixed"
            ]

            for robot_idx in four_finger_pitch_dip_indices:
                joint_limit = movable_joints[robot_idx].find("limit")
                lower = float(joint_limit.attrib["lower"])
                upper = float(joint_limit.attrib["upper"])
                assert lower <= fist[robot_idx] <= upper

    def test_o20_thumb_opose_motor_targets(self, monkeypatch):
        from linkerhand_retarget.motion.linkerforce.config.o20_config import (
            ROBOT_OPOSE_LEFT,
            ROBOT_OPOSE_RIGHT,
        )

        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        monkeypatch.syspath_prepend(str(package_dir))
        from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o20 import (
            _piecewise_motor_value,
        )

        hand_config_path = package_dir / "config" / "hand_config.yml"
        hand_config = yaml.safe_load(hand_config_path.read_text())
        expected_opose_urdf = {
            "right": (0.153, 0.561),
            "left": (-0.153, -0.561),
        }

        for side, opose in (
            ("right", ROBOT_OPOSE_RIGHT),
            ("left", ROBOT_OPOSE_LEFT),
        ):
            assert opose[0] == pytest.approx(expected_opose_urdf[side][0])
            assert opose[1] == pytest.approx(expected_opose_urdf[side][1])

            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o20_{side}"
                / f"linkerhand_o20_{side}.urdf"
            )
            robot = ET.parse(urdf_path).getroot()
            roll_limit = robot.find("./joint[@name='thumb_cmc_roll']").find("limit")
            yaw_limit = robot.find("./joint[@name='thumb_cmc_yaw']").find("limit")
            movable_joints = [
                joint
                for joint in robot.findall("joint")
                if joint.attrib.get("type") != "fixed"
            ]

            roll_output = _piecewise_motor_value(
                opose[0],
                float(roll_limit.attrib["lower"]),
                opose[0],
                float(roll_limit.attrib["upper"]),
                0,
                165,
                255,
            )
            yaw_output = _piecewise_motor_value(
                opose[1],
                float(yaw_limit.attrib["lower"]),
                opose[1],
                float(yaw_limit.attrib["upper"]),
                0,
                138,
                255,
            )

            assert roll_output == pytest.approx(165, abs=1)
            assert yaw_output == pytest.approx(138, abs=1)

            qpos = [0.0] * 25
            qpos[16] = opose[0]
            qpos[17] = opose[1]
            for motor_idx, expected in ((5, 165), (10, 138)):
                source_idx = _optional_command(
                    hand_config[f"commandsourcedataindex_{side}_o20"][motor_idx]
                )
                urdf_idx = _optional_command(hand_config[f"urdfdataindex_{side}_o20"][motor_idx])
                command_lower = _optional_command(
                    hand_config[f"commandlower_{side}_o20"][motor_idx]
                )
                command_upper = _optional_command(
                    hand_config[f"commandupper_{side}_o20"][motor_idx]
                )
                joint_limit = movable_joints[urdf_idx].find("limit")
                lower = float(joint_limit.attrib["lower"])
                upper = float(joint_limit.attrib["upper"])
                joint_angle = min(upper, max(lower, qpos[source_idx]))
                opose_motor = 165 if motor_idx == 5 else 138
                opose_angle = opose[0] if motor_idx == 5 else opose[1]

                assert _piecewise_motor_value(
                    joint_angle,
                    lower,
                    opose_angle,
                    upper,
                    command_lower,
                    opose_motor,
                    command_upper,
                ) == pytest.approx(expected, abs=1)

    @pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
    def test_o20_hand_starts_from_open_motor_position(self, hand_class_name, monkeypatch):
        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        monkeypatch.syspath_prepend(str(package_dir))

        import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o20 as linkerforce_o20

        hand_class = getattr(linkerforce_o20, hand_class_name)
        hand = hand_class(handcore=None)

        assert hand.g_jointpositions == [0] * 20
        assert hand.last_jointpositions == [0] * 20
        assert hand.smooth_positions == [0.0] * 20

    @pytest.mark.parametrize(
        "hand_class_name,side",
        [("RightHand", "right"), ("LeftHand", "left")],
    )
    def test_o20_calibration_anchor_inputs_preserve_robot_urdf_targets(
        self,
        hand_class_name,
        side,
        monkeypatch,
    ):
        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        monkeypatch.syspath_prepend(str(package_dir))

        import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o20 as linkerforce_o20

        hand_class = getattr(linkerforce_o20, hand_class_name)
        hand = hand_class(handcore=None)

        open_glove = [float(i) for i in range(21)]
        opose_glove = [value + 0.5 for value in open_glove]
        fist_glove = [value + 1.0 for value in open_glove]
        hand.calibrationoriginal = open_glove
        hand.calibrationopose = opose_glove
        hand.calibrationfistpose = fist_glove
        hand.initialize_mapper()

        for glove_state, expected_robot_state in (
            (open_glove, hand.effective_robot_original),
            (opose_glove, hand.effective_robot_opose),
        ):
            mapped = hand.multi_state_mapper.map_glove_to_robot(glove_state)
            assert list(mapped) == pytest.approx(expected_robot_state)

    def test_o20_four_finger_roll_mapping_uses_configured_source_direction(self):
        from linkerhand_retarget.motion.linkerforce.config.o20_config import FINGER_CONFIGS

        for config_name in ("index_roll", "middle_roll", "ring_roll", "pinky_roll"):
            assert FINGER_CONFIGS[config_name]["weights"]["v1"] == [1]
            assert FINGER_CONFIGS[config_name]["weights"]["v2"] == [1]
            assert FINGER_CONFIGS[config_name]["reverse_motion"]["v1"] is False
            assert FINGER_CONFIGS[config_name]["reverse_motion"]["v2"] is False

        for config_name in ("thumb_rotate", "index_roll", "middle_roll", "ring_roll", "pinky_roll"):
            assert "state_order" not in FINGER_CONFIGS[config_name]
            assert "reverse_output_direction" not in FINGER_CONFIGS[config_name]

    def test_o20_right_thumb_abduction_open_and_fist_are_valid_urdf_targets(self):
        from linkerhand_retarget.motion.linkerforce.config.o20_config import (
            ROBOT_FIST_RIGHT,
            ROBOT_ORIGINAL_RIGHT,
        )

        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        urdf_path = (
            package_dir
            / "assets"
            / "robots"
            / "hands"
            / "linker_hand"
            / "o20_right"
            / "linkerhand_o20_right.urdf"
        )
        yaw_limit = ET.parse(urdf_path).getroot().find("./joint[@name='thumb_cmc_yaw']").find("limit")
        lower = float(yaw_limit.attrib["lower"])
        upper = float(yaw_limit.attrib["upper"])

        assert lower <= ROBOT_ORIGINAL_RIGHT[1] <= upper
        assert lower <= ROBOT_FIST_RIGHT[1] <= upper

    def test_o20_mujoco_thumb_abduction_display_uses_direct_mapped_angles(self, monkeypatch):
        package_dir = Path(__file__).parents[2] / "linkerhand_retarget"
        monkeypatch.syspath_prepend(str(package_dir))

        from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o20 import (
            LeftHand,
            O20_MUJOCO_JOINT_ARC_INDICES,
            O20_MUJOCO_JOINT_ARC_SIGNS,
            RightHand,
        )

        assert O20_MUJOCO_JOINT_ARC_INDICES[1] == 1
        assert O20_MUJOCO_JOINT_ARC_SIGNS[1] == 1.0

        for hand_cls, side in ((RightHand, "right"), (LeftHand, "left")):
            urdf_path = (
                package_dir
                / "assets"
                / "robots"
                / "hands"
                / "linker_hand"
                / f"o20_{side}"
                / f"linkerhand_o20_{side}.urdf"
            )
            yaw_joint = ET.parse(urdf_path).getroot().find("./joint[@name='thumb_cmc_yaw']")
            yaw_axis = yaw_joint.find("axis")

            assert hand_cls(handcore=None).mujoco_joint_arc_mirrors[1] is None
            if side == "right":
                assert yaw_axis.attrib["xyz"] == "1 0 0"


class TestOperatorToMano:
    def test_operator2mano_right(self):
        assert OPERATOR2MANO[HandType.right].shape == (3, 3)
        assert (OPERATOR2MANO[HandType.right] == OPERATOR2MANO_RIGHT).all()

    def test_operator2mano_left(self):
        assert OPERATOR2MANO[HandType.left].shape == (3, 3)
        assert (OPERATOR2MANO[HandType.left] == OPERATOR2MANO_LEFT).all()

    def test_operator2mano_right_values(self):
        expected = [
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0],
        ]
        assert (OPERATOR2MANO_RIGHT == expected).all()


class TestGetDefaultConfigPath:
    def test_get_config_path_teleop(self):
        path = get_default_config_path(RobotName.l6, RetargetingType.vector, HandType.right)
        assert path is not None
        assert "teleop" in str(path)
        assert "l6" in str(path).lower()

    def test_get_config_path_offline(self):
        path = get_default_config_path(RobotName.l6, RetargetingType.position, HandType.right)
        assert path is not None
        assert "offline" in str(path)
