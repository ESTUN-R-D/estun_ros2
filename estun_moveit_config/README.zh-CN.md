# estun_moveit_config

语言：中文 | [English](./README.md)

`estun_moveit_config` 是 ESTUN 机械臂的 MoveIt 配置包，用于提供规划参数、控制器映射以及演示启动入口。

## 包含内容

本包提供：

1. MoveIt 规划配置
2. 机型级 `SRDF`（语义机器人描述格式）与规划专属配置
3. MoveIt Servo 配置
4. RViz 规划演示配置
5. 对外演示入口 `demo.launch.py`

## 配套关系

- `estun_description`：提供机器人描述、模型资源与机型限速参数
- `estun_hardware`：提供控制层启动与硬件接口

`demo.launch.py` 会复用 `estun_hardware/launch/estun_control.launch.py` 作为控制层。

## 唯一公共演示入口

本包对外的统一演示入口为：

```text
estun_moveit_config/launch/demo.launch.py
```

## 快速开始

先完成工作区编译并加载环境：

```bash
cd ~/estun_ws
colcon build --packages-select estun_msgs estun_libs estun_description estun_hardware estun_moveit_config
source install/setup.bash
```

### 规划执行模式

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=plan
```

### 关节空间实时流模式

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=apos
```

### 笛卡尔空间实时流模式

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=cpos
```

## 关键启动参数

- `control_mode`：控制模式，支持 `plan`、`apos`、`cpos`
- `start_moveit`：是否启动 `move_group`，支持 `auto|true|false`
- `start_rviz`：是否启动 RViz，支持 `auto|true|false`
- `start_servo`：是否启动 MoveIt Servo，支持 `auto|true|false`
- `planning_pipelines`：规划管线列表，默认 `ompl,pilz`
- `default_planning_pipeline`：默认规划管线，支持 `ompl|pilz`
- `controllers_file`：控制器参数文件，会透传给 `estun_hardware`
- `model`、`robot_ip`、`use_fake_hardware`、`prefix`：与控制层启动参数保持一致；其中 `robot_ip` 在真机模式必填，虚拟硬件可留空

## 当前行为说明

1. `start_moveit:=auto` 时，`plan` 模式默认开启，`apos/cpos` 模式默认关闭。
2. `start_rviz:=auto` 时，`plan` 模式默认开启，`apos/cpos` 模式默认关闭。
3. `start_servo` 仅支持 `control_mode:=apos`。
4. 如果启用 Servo（实时伺服）但未启用 MoveIt，启动逻辑会自动拉起 `move_group`。

## 规划管线

当前支持两类规划管线：

1. `ompl`
2. `pilz`

补充说明：

- `default_planning_pipeline` 必须包含在 `planning_pipelines` 中。
- 当启用 RViz 且默认规划管线配置为 `pilz` 时，启动逻辑会自动回退到 `ompl`，以适配 RViz 默认规划请求。

## 补充说明

- 用户侧运动示例可参考 `estun_user_motion_cpp`。
- 如果只需要控制层，不需要 MoveIt 与 RViz，可直接使用 `estun_hardware`。
