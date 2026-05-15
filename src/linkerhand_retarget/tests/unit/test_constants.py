import pytest
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


class TestRobotName:
    def test_enum_values(self):
        assert RobotName.o7.value is not None
        assert RobotName.l6.value is not None
        assert RobotName.l20.value is not None

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

    def test_robot_len_map(self):
        assert ROBOT_LEN_MAP[RobotName.o7] == 7
        assert ROBOT_LEN_MAP[RobotName.l6] == 6
        assert ROBOT_LEN_MAP[RobotName.l20] == 20
        assert ROBOT_LEN_MAP[RobotName.l25] == 20


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
