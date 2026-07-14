# estun_dual_arm_example

语言：中文 | [English](./README.md)

`estun_dual_arm_example` 是 ESTUN 双臂使用案例包，用于把两套独立的 ESTUN 控制链路组合成一个双臂演示入口，并向 MoveIt 提供统一的双臂模型、规划配置和关节状态合并节点。

## 包含内容

本包提供：

1. 双臂控制启动入口 `launch/dual_control.launch.py`
2. 双臂 MoveIt 启动入口 `launch/dual_moveit.launch.py`
3. 双臂 `URDF/Xacro` 与 `SRDF`
4. 双臂 MoveIt 规划配置
5. 关节状态合并脚本 `scripts/joint_state_merger.py`

## 配套关系

- `estun_hardware`：提供单臂控制层启动入口，每个机械臂各启动一套控制链路
- `estun_description`：提供单臂机型描述与运动学参数
- `estun_moveit_config`：单臂规划演示包；本包是双臂场景的案例入口

## 作用说明

本包的设计目标是：

1. 复用两套单臂 `estun_control.launch.py` 组成双臂控制场景
2. 提供双臂统一 `robot_description`、`robot_description_semantic` 和规划参数
3. 将左右臂 `/joint_states` 合并成 MoveIt 可直接消费的全局 `/joint_states`

## 启动入口

当前双臂场景有两个主要入口：

```text
estun_dual_arm_example/launch/dual_control.launch.py
estun_dual_arm_example/launch/dual_moveit.launch.py
```

它们的分工是：

1. `dual_control.launch.py`
   - 启动左右两套 `estun_hardware/launch/estun_control.launch.py`
   - 适合只验证双臂控制链路是否启动成功
2. `dual_moveit.launch.py`
   - 可选拉起双臂控制链路
   - 启动 `joint_state_merger.py`
   - 启动 `move_group`
   - 可选启动 `rviz2`

## 默认双臂约定

当前默认约定如下：

- 右臂命名空间：`rarm`
- 左臂命名空间：`larm`
- 右臂前缀：`rarm_`
- 左臂前缀：`larm_`
- 双臂默认机型：`iER7-910-MI`

`joint_state_merger.py` 默认订阅：

- `/rarm/joint_states`
- `/larm/joint_states`

并发布全局：

- `/joint_states`

这样 MoveIt 只需要订阅一个统一的关节状态主题。

## 快速开始

先完成工作区编译并加载环境：

```bash
cd ~/estun_ws
colcon build --packages-select \
  estun_libs \
  estun_description \
  estun_hardware \
  estun_dual_arm_example
source install/setup.bash
```

### 1. 仅启动双臂控制层

适合先验证双臂控制链路和命名空间划分：

```bash
ros2 launch estun_dual_arm_example dual_control.launch.py \
  rarm_use_fake_hardware:=true \
  larm_use_fake_hardware:=true \
  control_mode:=plan
```

### 2. 启动双臂 MoveIt 演示

适合在虚拟硬件下验证双臂模型、规划和关节状态合并：

```bash
ros2 launch estun_dual_arm_example dual_moveit.launch.py \
  start_control:=true \
  start_rviz:=true \
  rarm_use_fake_hardware:=true \
  larm_use_fake_hardware:=true
```

## 关键启动参数

### `dual_control.launch.py`

- `rarm_namespace`：右臂 ROS 命名空间
- `larm_namespace`：左臂 ROS 命名空间
- `rarm_prefix`：右臂关节/连杆前缀
- `larm_prefix`：左臂关节/连杆前缀
- `rarm_model`：右臂机型
- `larm_model`：左臂机型
- `rarm_ip`：右臂控制器 IP
- `larm_ip`：左臂控制器 IP
- `rarm_use_fake_hardware`：右臂是否启用虚拟硬件
- `larm_use_fake_hardware`：左臂是否启用虚拟硬件
- `control_mode`：控制模式，支持 `plan`、`apos`、`cpos`
- `control_node_impl`：控制节点实现，支持 `auto`、`official`、`estun`
- `controllers_file`：控制器配置文件

### `dual_moveit.launch.py`

- `start_control`：是否在同一入口下同时启动双臂控制层
- `start_rviz`：是否启动 RViz
- `rarm_namespace` / `larm_namespace`：左右臂命名空间
- `rarm_prefix` / `larm_prefix`：左右臂前缀
- `rarm_model` / `larm_model`：左右臂机型；`URDF/Xacro`、运动学参数和关节限位会随机型自动切换
- `rarm_use_fake_hardware` / `larm_use_fake_hardware`：左右臂是否启用虚拟硬件
- `controllers_file`：透传给 `dual_control.launch.py` 的控制器配置文件

## joint_state_merger 说明

`joint_state_merger.py` 是本包的重要辅助节点，作用是把左右臂的关节状态汇总给 MoveIt。

当前行为如下：

1. 固定按“右臂、左臂”的顺序合并消息
2. 若某一侧 `JointState` 消息长度非法，则忽略该侧当前缓存，避免污染全局 `/joint_states`
3. `velocity` 和 `effort` 只有在参与合并的消息都完整时才会保留

## 当前支持范围

- 本包定位为双臂控制与规划的使用案例
- 当前默认按两套独立单臂控制链路组合，不在本包内新增双臂专用硬件接口
- 当前双臂 MoveIt 会按 `rarm_model` / `larm_model` 从 `estun_description/config/<model>/` 自动加载对应运动学参数和关节限位
