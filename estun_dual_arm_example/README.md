# estun_dual_arm_example

Language: [Chinese](./README.zh-CN.md) | English

`estun_dual_arm_example` is the ESTUN dual-arm example package. It combines two independent ESTUN control chains into a single dual-arm demo entry and provides a unified dual-arm model, planning configuration, and a joint-state merge node for MoveIt.

## What It Contains

This package provides:

1. dual-arm control launch entry `launch/dual_control.launch.py`
2. dual-arm MoveIt launch entry `launch/dual_moveit.launch.py`
3. dual-arm `URDF/Xacro` and `SRDF`
4. dual-arm MoveIt planning configuration
5. joint-state merge script `scripts/joint_state_merger.py`

## Package Relationships

- `estun_hardware`: provides the single-arm control entry, with one control chain launched for each arm
- `estun_description`: provides single-arm model descriptions and kinematic parameters
- `estun_moveit_config`: single-arm planning demo package; this package is the example entry for dual-arm scenarios

## Purpose

This package is designed to:

1. reuse two single-arm `estun_control.launch.py` instances to form a dual-arm control scenario
2. provide unified dual-arm `robot_description`, `robot_description_semantic`, and planning parameters
3. merge left and right `/joint_states` into a global `/joint_states` topic that MoveIt can consume directly

## Launch Entries

The current dual-arm scenario has two primary entry points:

```text
estun_dual_arm_example/launch/dual_control.launch.py
estun_dual_arm_example/launch/dual_moveit.launch.py
```

Their roles are:

1. `dual_control.launch.py`
   - launches two `estun_hardware/launch/estun_control.launch.py` instances, one for each arm
   - suitable when you only want to verify that the dual-arm control chains start correctly
2. `dual_moveit.launch.py`
   - can optionally start the dual-arm control chains
   - starts `joint_state_merger.py`
   - starts `move_group`
   - can optionally start `rviz2`

## Default Dual-Arm Conventions

The current defaults are:

- right-arm namespace: `rarm`
- left-arm namespace: `larm`
- right-arm prefix: `rarm_`
- left-arm prefix: `larm_`
- default dual-arm model: `iER7-910-MI`

`joint_state_merger.py` subscribes to:

- `/rarm/joint_states`
- `/larm/joint_states`

and publishes the global topic:

- `/joint_states`

This allows MoveIt to subscribe to a single unified joint-state topic.

## Quick Start

Build the workspace and source the environment:

```bash
cd ~/estun_ws
colcon build --packages-select \
  estun_libs \
  estun_description \
  estun_hardware \
  estun_dual_arm_example
source install/setup.bash
```

### 1. Start Only the Dual-Arm Control Layer

Use this to verify the dual-arm control chains and namespace layout first:

```bash
ros2 launch estun_dual_arm_example dual_control.launch.py \
  rarm_use_fake_hardware:=true \
  larm_use_fake_hardware:=true \
  control_mode:=plan
```

### 2. Start the Dual-Arm MoveIt Demo

Use this to validate the dual-arm model, planning setup, and joint-state merge behavior with fake hardware:

```bash
ros2 launch estun_dual_arm_example dual_moveit.launch.py \
  start_control:=true \
  start_rviz:=true \
  rarm_use_fake_hardware:=true \
  larm_use_fake_hardware:=true
```

## Key Launch Parameters

### `dual_control.launch.py`

- `rarm_namespace`: ROS namespace for the right arm
- `larm_namespace`: ROS namespace for the left arm
- `rarm_prefix`: joint/link prefix for the right arm
- `larm_prefix`: joint/link prefix for the left arm
- `rarm_model`: model used by the right arm
- `larm_model`: model used by the left arm
- `rarm_ip`: controller IP for the right arm
- `larm_ip`: controller IP for the left arm
- `rarm_use_fake_hardware`: whether to enable fake hardware for the right arm
- `larm_use_fake_hardware`: whether to enable fake hardware for the left arm
- `control_mode`: control mode, supports `plan`, `apos`, and `cpos`
- `control_node_impl`: control node implementation, supports `auto`, `official`, and `estun`
- `controllers_file`: controller configuration file

### `dual_moveit.launch.py`

- `start_control`: whether to start the dual-arm control layer from the same entry
- `start_rviz`: whether to start RViz
- `rarm_namespace` / `larm_namespace`: namespaces for the right and left arms
- `rarm_prefix` / `larm_prefix`: prefixes for the right and left arms
- `rarm_model` / `larm_model`: models for the right and left arms; the `URDF/Xacro`, kinematics, and joint limits switch with the selected model automatically
- `rarm_use_fake_hardware` / `larm_use_fake_hardware`: whether to enable fake hardware for each arm
- `controllers_file`: controller configuration file passed through to `dual_control.launch.py`

## joint_state_merger Behavior

`joint_state_merger.py` is an important helper node in this package. Its job is to aggregate joint states from both arms for MoveIt.

Current behavior:

1. messages are merged in a fixed "right arm, then left arm" order
2. if one side publishes an invalid `JointState` message length, the cached data from that side is ignored to avoid contaminating the global `/joint_states`
3. `velocity` and `effort` are preserved only when all merged messages provide complete values

## Current Scope

- This package is positioned as an example for dual-arm control and planning.
- The current default approach combines two independent single-arm control chains; it does not introduce a dedicated dual-arm hardware interface in this package.
- The current dual-arm MoveIt setup loads the kinematics and joint limits for `rarm_model` / `larm_model` automatically from `estun_description/config/<model>/`.
