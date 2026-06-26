# Dependency Manifest Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align the ROS2 package manifests with the codebase runtime imports while keeping MuJoCo as an optional display dependency.

**Architecture:** Treat `package.xml` as the ROS runtime contract and `setup.py` / `src/requirements.txt` as the Python installation contract. Add the missing ROS message/launch dependencies that the package imports directly, and keep optional display tooling such as MuJoCo outside the mandatory install path.

**Tech Stack:** ROS2 `ament_python`, Python packaging, pytest

---

### Task 1: Add the missing ROS runtime dependencies to `package.xml`

**Files:**
- Modify: `src/linkerhand_retarget/package.xml`

- [ ] **Step 1: Update the manifest**

Add the runtime packages imported by the installed code:
`ament_index_python`, `geometry_msgs`, `launch`, `launch_ros`, `rcl_interfaces`, `sensor_msgs`, and `std_msgs`.

- [ ] **Step 2: Keep optional display dependencies out**

Do not add `mujoco` here. The display layer already checks for it at runtime and continues when it is absent.

### Task 2: Align Python install requirements with shipped imports

**Files:**
- Modify: `src/linkerhand_retarget/setup.py`

- [ ] **Step 1: Extend `install_requires`**

Add the missing runtime Python packages already used by the installed modules: `pyserial`, `six`, and `tyro`.

- [ ] **Step 2: Preserve the optional-tool boundary**

Do not add `mujoco` to `install_requires`; it remains an optional display feature, not a hard dependency of the SDK.

### Task 3: Verify the dependency manifest change set

**Files:**
- Test: `src/linkerhand_retarget/tests/unit/test_mujoco_display.py`
- Test: `src/linkerhand_retarget/tests/unit/test_linkerforce_calibration.py`

- [ ] **Step 1: Run the focused tests under the ROS workspace**

Run: `bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && cd src/linkerhand_retarget && python3 -m pytest tests/unit/test_mujoco_display.py tests/unit/test_linkerforce_calibration.py -q'`

- [ ] **Step 2: Confirm the manifest diff**

Run: `git diff -- src/linkerhand_retarget/package.xml src/linkerhand_retarget/setup.py src/requirements.txt`
Expected: only dependency-list changes, with no `mujoco` added to the mandatory lists.
