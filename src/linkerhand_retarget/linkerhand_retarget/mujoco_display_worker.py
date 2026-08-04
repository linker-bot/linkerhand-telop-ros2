import json
import queue
import sys
import threading
import time
from pathlib import Path

from linkerhand_retarget.mujoco_display import MujocoDisplay


def _read_commands(stdin, command_queue):
    for line in stdin:
        line = line.strip()
        if line:
            command_queue.put(line)
    command_queue.put(None)


def main():
    if len(sys.argv) < 2:
        raise SystemExit("missing MuJoCo display config")

    config = json.loads(sys.argv[1])
    fps = max(int(config.get("fps", 30) or 30), 1)
    interval = 1.0 / fps
    commands = queue.Queue()
    reader = threading.Thread(
        target=_read_commands,
        args=(sys.stdin, commands),
        daemon=True,
    )
    reader.start()

    display = None
    latest_positions = {}
    try:
        display = MujocoDisplay(
            Path(config["model_path"]),
            fps=fps,
            hand=str(config.get("hand", "right")),
            model_scale=float(config.get("model_scale", 1.0)),
            model_rotate_rpy=tuple(config.get("model_rotate_rpy", (0.0, 0.0, 0.0))),
            model_translate_xyz=tuple(config.get("model_translate_xyz", (0.0, 0.0, 0.0))),
        ).start()

        while display.viewer is not None and display.viewer.is_running():
            deadline = time.monotonic() + interval
            while True:
                try:
                    line = commands.get_nowait()
                except queue.Empty:
                    break

                if line is None:
                    return

                message = json.loads(line)
                command = message.get("command")
                if command == "close":
                    return
                if command == "update":
                    latest_positions = dict(message.get("positions", {}))

            if latest_positions:
                display.update_joint_positions(latest_positions)
            elif display.viewer is not None and display.viewer.is_running():
                display.viewer.sync()

            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if display is not None:
            display.close()


if __name__ == "__main__":
    main()
