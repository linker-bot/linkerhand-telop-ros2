import pytest
from linkerhand_retarget.linkerhand.sensenovacore import NODES_HAND


class TestSensenovaConstants:
    def test_nodes_hand_value(self):
        assert NODES_HAND == 30
