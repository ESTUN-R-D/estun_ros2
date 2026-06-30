# estun_msgs

语言：中文 | [English](./README.md)

`estun_msgs` 提供 ESTUN ROS 2 控制栈共用的消息与服务接口定义。本包只定义接口类型，不直接实现运行时逻辑。

运行时实现位于 `estun_hardware`。

## 消息接口

### 连接状态

- `msg/ConnectionStatus.msg`
  - 返回时间戳、连接状态、机器人 IP 和状态说明

### 机器人运行状态

- `msg/EstunRobotStatus.msg`
  - 返回连接状态、错误状态、当前指令模式，以及队列和伺服统计信息

## 服务接口

### 连接与错误

- `srv/GetConnectionStatus.srv`
- `srv/GetRobotConnStatus.srv`
- `srv/GetCurErrMsg.srv`

### 运行状态

- `srv/GetWorldCpos.srv`
- `srv/GetJointValue.srv`

### 工具 / 用户坐标系

- `srv/GetTool.srv`
- `srv/GetUser.srv`

### DO（数字输出）

- `srv/SetDo.srv`
- `srv/GetDo.srv`

## 使用关系

- `estun_hardware`：创建并对外提供这些服务
- 上层应用：通过这些消息和服务读取机器人状态或调用 SDK 能力

## 构建说明

```bash
cd ~/estun_ws
colcon build --packages-select estun_msgs
source install/setup.bash
```
