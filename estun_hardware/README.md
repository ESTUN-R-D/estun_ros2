# estun_hardware

Language: [Chinese](./README.zh-CN.md) | English

`estun_hardware` is the hardware interface package in the ESTUN ROS 2 control stack. It connects to the ERI interface through `ros2_control` and provides a unified public control-layer launch entry.

## What It Contains

This package provides:

1. a `ros2_control` hardware interface plugin
2. the control-layer launch entry `estun_control.launch.py`
3. model-specific control `URDF/Xacro` entries
4. controller parameters and hardware configuration
5. runtime SDK service interfaces under `/estun/*`

## Package Relationships

- `estun_description`: provides robot models, kinematic parameters, and machine limits
- `estun_libs`: provides ERI SDK headers, shared libraries, and runtime configuration
- `estun_msgs`: defines the message and service interfaces used at runtime by this package
- `estun_moveit_config`: reuses this package as the control-layer entry in planning and demo scenarios

## Single Public Control Entry

The unified public control entry for this package is:

```text
estun_hardware/launch/estun_control.launch.py
```

## Entry Layering

This repository explicitly separates "model description" from "hardware assembly":

1. `estun_description/robot/<MODEL>.urdf.xacro`
   - pure model entry
   - only defines links, joints, meshes, kinematic parameters, and machine limits
   - does not include `ros2_control`
2. `estun_hardware/robot/<MODEL>.urdf.xacro`
   - hardware assembly entry
   - combines the pure model macros with the `ros2_control` macros to form the `robot_description` used by the control chain

For day-to-day usage, prefer `estun_control.launch.py` or `estun_moveit_config/demo.launch.py`.
If you invoke `estun_hardware/robot/<MODEL>.urdf.xacro` directly, you must explicitly pass `control_mode:=plan|apos|cpos`. There is no silent fallback to a default runtime mode.

## Control Modes

| `control_mode` | Meaning | Default Controller |
|---|---|---|
| `plan` | planned execution | `estun_arm_controller` |
| `apos` | joint-space streaming | `forward_position_controller` |
| `cpos` | Cartesian-space streaming | `cartesian_forward_controller` |

Notes:

1. `plan` and `apos` both execute through the APOS path.
2. `cpos` executes through the CPOS path.

## Quick Start

Build the workspace and source the environment:

```bash
cd ~/estun_ws
colcon build --packages-select estun_msgs estun_libs estun_description estun_hardware
source install/setup.bash
```

Start the control layer for a real robot:

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=plan
```

Start fake hardware:

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=true \
  model:=ER20-1780-A6 \
  control_mode:=apos
```

## Key Launch Parameters

- `model`: model name
- `robot_ip`: robot controller IP address; required for real hardware and optional for fake hardware
- `use_fake_hardware`: whether to enable fake hardware
- `control_mode`: control mode, supports `plan`, `apos`, and `cpos`
- `motion_period_ms`: ERI motion period in milliseconds
- `controllers_file`: controller parameter file; supports a path relative to the `estun_hardware` share directory or an absolute path. The top-level YAML key supports relative node names, `/**` / `/**/<node>` wildcard forms, and `/<namespace>/<node>` absolute forms
- `control_node_impl`: control node implementation, supports `auto`, `official`, and `estun`
- `prefix`: robot naming prefix

Advanced parameters:

- `cmd_port`: ERI command port
- `servo_port`: ERI real-time servo port
- `status_port`: ERI status port
- `use_logical_control_time`: only effective for `estun_control_node`
- `enable_control_loop_diag_log`: only effective for `estun_control_node`

## Runtime Service Interfaces

After successful configuration, this package starts the `/estun/*` service namespace. If `prefix` is set, the namespace becomes `/<prefix>estun/*`.

Current services:

1. `/estun/get_connection_status`
2. `/estun/get_robot_conn_status`
3. `/estun/get_cur_err_msg`
4. `/estun/get_world_cpos`
5. `/estun/get_joint_value`
6. `/estun/set_do`
7. `/estun/get_do`
8. `/estun/get_tool`
9. `/estun/get_user`

See `estun_msgs` for the interface definitions of these services.

## Current Scope

- This package handles control-layer startup, hardware interfaces, and SDK runtime services.
- For MoveIt planning, RViz demos, and Servo startup, use `estun_moveit_config`.
