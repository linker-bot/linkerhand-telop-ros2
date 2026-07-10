import pytest

pytest.importorskip("rclpy")
pytest.importorskip("ament_index_python")

from linkerhand_retarget import handretarget


def test_signal_handler_ignores_already_shutdown_context(monkeypatch):
    def fail_if_called():
        raise RuntimeError("Context must be initialized before it can be shutdown")

    monkeypatch.setattr(handretarget.rclpy, "ok", lambda: False)
    monkeypatch.setattr(handretarget.rclpy, "shutdown", fail_if_called)

    handretarget.signal_handler(None, None)
