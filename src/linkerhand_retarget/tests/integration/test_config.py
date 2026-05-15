import pytest
import os
from pathlib import Path
from linkerhand.config import HandConfig


class TestHandConfig:
    @pytest.fixture
    def config_path(self):
        test_dir = Path(__file__).parent.parent.parent / "linkerhand_retarget"
        return str(test_dir)

    @pytest.fixture
    def robot_dir(self):
        test_dir = Path(__file__).parent.parent.parent / "linkerhand_retarget" / "assets" / "robots"
        return str(test_dir)

    @pytest.mark.skipif(not Path(__file__).parent.parent.parent.joinpath("linkerhand_retarget/assets").exists(), reason="Assets directory not found")
    def test_hand_config_initialization(self, config_path, robot_dir):
        config = HandConfig(robot_dir, config_path)
        assert config.handconfig is not None
        assert config.baseconfig is not None
        assert config.retagetconfig is not None
        assert config.modelconfig is not None
