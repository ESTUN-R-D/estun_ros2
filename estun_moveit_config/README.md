# estun_moveit_config

Language: [Chinese](./README.zh-CN.md) | English

`estun_moveit_config` is the MoveIt configuration package for ESTUN robots. It provides planning parameters, controller mappings, and the public demo launch entry.

## What It Contains

This package provides:

1. MoveIt planning configuration
2. model-specific `SRDF` (Semantic Robot Description Format) plus planning-specific configuration
3. MoveIt Servo configuration
4. RViz planning demo configuration
5. the public demo entry `demo.launch.py`

## Package Relationships

- `estun_description`: provides robot descriptions, model assets, and machine limits
- `estun_hardware`: provides control-layer startup and hardware interfaces

`demo.launch.py` reuses `estun_hardware/launch/estun_control.launch.py` as the control layer.

## Single Public Demo Entry

The unified public demo entry for this package is:

```text
estun_moveit_config/launch/demo.launch.py
```

## Quick Start

Build the workspace and source the environment:

```bash
cd ~/estun_ws
colcon build --packages-select estun_msgs estun_libs estun_description estun_hardware estun_moveit_config
source install/setup.bash
```

### Planned Execution Mode

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=plan
```

### Joint-Space Streaming Mode

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=apos
```

### Cartesian-Space Streaming Mode

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=cpos
```

## Key Launch Parameters

- `control_mode`: control mode, supports `plan`, `apos`, and `cpos`
- `start_moveit`: whether to start `move_group`, supports `auto|true|false`
- `start_rviz`: whether to start RViz, supports `auto|true|false`
- `start_servo`: whether to start MoveIt Servo, supports `auto|true|false`
- `planning_pipelines`: planning pipeline list, default is `ompl,pilz`
- `default_planning_pipeline`: default planning pipeline, supports `ompl|pilz`
- `controllers_file`: controller parameter file passed through to `estun_hardware`
- `model`, `robot_ip`, `use_fake_hardware`, and `prefix`: aligned with the control-layer launch parameters; `robot_ip` is required for real hardware and optional for fake hardware

## Current Behavior

1. When `start_moveit:=auto`, it is enabled by default in `plan` mode and disabled by default in `apos/cpos` modes.
2. When `start_rviz:=auto`, it is enabled by default in `plan` mode and disabled by default in `apos/cpos` modes.
3. `start_servo` is supported only when `control_mode:=apos`.
4. If Servo is enabled but MoveIt is not, the launch logic automatically starts `move_group`.

## Planning Pipelines

The package currently supports two planning pipelines:

1. `ompl`
2. `pilz`

Additional notes:

- `default_planning_pipeline` must be included in `planning_pipelines`.
- When RViz is enabled and the default planning pipeline is configured as `pilz`, the launch logic automatically falls back to `ompl` to match RViz's default planning request behavior.

## Notes

- User-side motion examples are available in `estun_user_motion_cpp`.
- If you only need the control layer and do not need MoveIt or RViz, use `estun_hardware` directly.
