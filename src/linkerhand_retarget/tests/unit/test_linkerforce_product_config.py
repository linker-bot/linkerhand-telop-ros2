from linkerhand_retarget.motion.linkerforce.hand.simple_linear_mapper import (
    apply_linkerforce_product_config,
    load_linkerforce_config,
    resolve_linkerforce_finger_configs,
    resolve_linkerforce_motor_constraints,
    resolve_linkerforce_product,
)
from types import SimpleNamespace
import pytest


def _flat_config():
    return {
        "thumb": {
            "weights": {
                "v1": [1, 0],
                "v2": [0, 1],
            },
            "reverse_motion": {
                "v1": False,
                "v2": True,
            },
            "extended_mapping": {
                "enabled": True,
                "scale_factor": 1.0,
            },
        }
    }


def test_lffg2_v01_selects_lffg2_v1():
    assert resolve_linkerforce_product("LFFG2-G20-V01", "3.1.3") == (
        "LFFG2",
        "v1",
    )


def test_lffg2_non_v01_selects_lffg2_v2():
    assert resolve_linkerforce_product("LFFG2-G20-V02", "3.1.3") == (
        "LFFG2",
        "v2",
    )


def test_non_lffg2_uses_firmware_major_version():
    assert resolve_linkerforce_product("LinkerForce-G20", "1.8.2") == (
        "LFFG1",
        "v1",
    )
    assert resolve_linkerforce_product("LinkerForce-G20", "2.12.10") == (
        "LFFG1",
        "v2",
    )
    assert resolve_linkerforce_product("LinkerForce-G20", "3.1.3") == (
        "LFFG1",
        "v1",
    )


def test_empty_product_type_falls_back_to_firmware_version():
    assert resolve_linkerforce_product("", "v1.4") == ("LFFG1", "v1")
    assert resolve_linkerforce_product(None, "v2.4") == ("LFFG1", "v2")
    assert resolve_linkerforce_product(None, "v3.1.3") == ("LFFG1", "v1")


def test_resolver_reduces_flat_version_fields_without_mutating_source():
    configs = _flat_config()

    resolved = resolve_linkerforce_finger_configs(configs, "LFFG2-G20-V01", "3.1.3")

    assert resolved["thumb"]["weights"] == [1, 0]
    assert resolved["thumb"]["reverse_motion"] is False
    assert resolved["thumb"]["extended_mapping"]["scale_factor"] == 1.0
    assert configs["thumb"]["weights"]["v1"] == [1, 0]


def test_motor_constraints_are_selected_by_side_and_copied():
    constraints = {
        "left": [{"min": 10}],
        "right": [{"min": 1}],
    }

    left = resolve_linkerforce_motor_constraints(
        constraints, "left", "LFFG2-G20-V01", "3.1.3"
    )
    right = resolve_linkerforce_motor_constraints(
        constraints, "right", "LinkerForce-G20", "2.12.10"
    )

    assert left == [{"min": 10}]
    assert right == [{"min": 1}]
    left[0]["min"] = 99
    assert constraints["left"][0]["min"] == 10


def test_loader_selects_flat_lffg_family_config_module():
    lffg1 = load_linkerforce_config("g20", None, "3.1.2")
    lffg2 = load_linkerforce_config("g20", "LFFG2-G20-V01", "3.1.3")

    assert lffg1.__name__.endswith("config.lffg1.g20_config")
    assert lffg2.__name__.endswith("config.lffg2.g20_config")
    assert set(lffg1.FINGER_CONFIGS["thumb_end_flexion"]["weights"]) == {"v1", "v2"}
    assert lffg1.FINGER_CONFIGS["thumb_end_flexion"]["extended_mapping"]["scale_factor"] == 0.8
    assert lffg2.FINGER_CONFIGS["thumb_end_flexion"]["extended_mapping"]["scale_factor"] == 0.8


@pytest.mark.parametrize("model_name", ("g20", "l10", "l20", "l30", "l6", "o20", "o30", "o6", "o7"))
def test_every_model_has_flat_configs_for_both_product_families(model_name):
    lffg1 = load_linkerforce_config(model_name, None, "3.1.2")
    lffg2 = load_linkerforce_config(model_name, f"LFFG2-{model_name}-V01", "3.1.3")

    for module, family in ((lffg1, "lffg1"), (lffg2, "lffg2")):
        assert module.__name__.endswith(f"config.{family}.{model_name}_config")
        assert module.FINGER_CONFIGS
        assert isinstance(module.MOTOR_CONSTRAINTS["left"], list)
        assert isinstance(module.MOTOR_CONSTRAINTS["right"], list)
        for config in module.FINGER_CONFIGS.values():
            assert "LFFG1" not in config.get("weights", {})
            assert "LFFG2" not in config.get("weights", {})
            reverse_motion = config.get("reverse_motion", {})
            if isinstance(reverse_motion, dict):
                assert "LFFG1" not in reverse_motion
                assert "LFFG2" not in reverse_motion


def test_apply_product_config_loads_selected_family_module():
    mapper = SimpleNamespace(finger_configs={})
    hand = SimpleNamespace(multi_state_mapper=mapper)

    apply_linkerforce_product_config(
        hand,
        "g20",
        "right",
        "LFFG2-G20-V01",
        "3.1.3",
    )

    assert hand.product_family == "LFFG2"
    assert hand.glove_version == "v1"
    assert hand.product_config_module.__name__.endswith("config.lffg2.g20_config")
    assert mapper.finger_configs["thumb_end_flexion"]["extended_mapping"]["scale_factor"] == 0.8
