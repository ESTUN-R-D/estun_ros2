# estun_hardware

语言：中文 | [English](./README.md)

`estun_hardware` 是 ESTUN ROS 2 控制栈的硬件接口包，基于 `ros2_control`（ROS 2 控制框架）对接 ERI 接口，并提供统一的控制层启动入口。

## 包含内容

本包提供：

1. `ros2_control` 硬件接口插件
2. 控制主节点启动入口 `estun_control.launch.py`
3. 机型级控制用 `URDF/Xacro` 入口
4. 控制器参数与硬件配置
5. 运行时 SDK 服务接口 `/estun/*`

## 配套关系

- `estun_description`：提供机器人模型、运动学参数与机型限速参数
- `estun_libs`：提供 ERI SDK 头文件、共享库和运行时配置
- `estun_msgs`：定义本包运行时使用的消息与服务接口
- `estun_moveit_config`：在规划与演示场景中复用本包作为控制层入口

## 唯一公共控制入口

本包对外的统一控制入口为：

```text
estun_hardware/launch/estun_control.launch.py
```

## 入口分层

当前仓库把“模型描述”和“硬件装配”明确拆成两层：

1. `estun_description/robot/<MODEL>.urdf.xacro`
   - 纯模型入口
   - 只负责连杆、关节、网格、运动学参数和机型限速参数
   - 不带 `ros2_control`
2. `estun_hardware/robot/<MODEL>.urdf.xacro`
   - 硬件装配入口
   - 负责把纯模型宏与 `ros2_control` 宏组合成控制链路使用的 `robot_description`

日常启动请优先使用 `estun_control.launch.py` 或 `estun_moveit_config/demo.launch.py`。
如果直接调用 `estun_hardware/robot/<MODEL>.urdf.xacro`，必须显式传入 `control_mode:=plan|apos|cpos`，不会再静默使用默认业务模式。

## 控制模式

| `control_mode` | 含义 | 默认控制器 |
|---|---|---|
| `plan` | 规划执行 | `estun_arm_controller` |
| `apos` | 关节空间实时流 | `forward_position_controller` |
| `cpos` | 笛卡尔空间实时流 | `cartesian_forward_controller` |

说明：

1. `plan` 和 `apos` 都通过 APOS 链路执行。
2. `cpos` 通过 CPOS 链路执行。

## 快速开始

先完成工作区编译并加载环境：

```bash
cd ~/estun_ws
colcon build --packages-select estun_msgs estun_libs estun_description estun_hardware
source install/setup.bash
```

启动真实机器人控制层：

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=plan
```

启动虚拟硬件：

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=true \
  model:=ER20-1780-A6 \
  control_mode:=apos
```

## 关键启动参数

- `model`：机型名称
- `robot_ip`：机器人控制器 IP 地址；真机模式必填，虚拟硬件可留空
- `use_fake_hardware`：是否启用虚拟硬件
- `control_mode`：控制模式，支持 `plan`、`apos`、`cpos`
- `motion_period_ms`：ERI 运动周期，单位为毫秒
- `controllers_file`：控制器参数文件，支持相对 `estun_hardware` share 目录的路径或绝对路径；顶层 key 支持相对节点名、`/**` / `/**/<node>` 通配写法，以及 `/<namespace>/<node>` 绝对写法
- `control_node_impl`：控制节点实现，支持 `auto`、`official`、`estun`
- `prefix`：机器人命名前缀

高级参数：

- `cmd_port`：ERI 命令端口
- `servo_port`：ERI 实时伺服端口
- `status_port`：ERI 状态端口
- `use_logical_control_time`：仅在 `estun_control_node` 下生效
- `enable_control_loop_diag_log`：仅在 `estun_control_node` 下生效

## 运行时服务接口

本包在成功配置后会启动 `/estun/*` 服务命名空间；如果设置了 `prefix`，则服务命名空间会变为 `/<prefix>estun/*`。

当前服务包括：

1. `/estun/get_connection_status`
2. `/estun/get_robot_conn_status`
3. `/estun/get_cur_err_msg`
4. `/estun/get_world_cpos`
5. `/estun/get_joint_value`
6. `/estun/set_do`
7. `/estun/get_do`
8. `/estun/get_tool`
9. `/estun/get_user`

这些服务的接口定义请查看 `estun_msgs`。

## 当前支持范围

- 本包负责控制层启动、硬件接口和 SDK 运行时服务。
- MoveIt 规划、RViz 演示与 Servo 启动请使用 `estun_moveit_config`。
