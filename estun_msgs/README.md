# estun_msgs

Language: [Chinese](./README.zh-CN.md) | English

`estun_msgs` provides the shared message and service interface definitions used by the ESTUN ROS 2 control stack. This package defines interface types only and does not implement runtime logic directly.

The runtime implementations live in `estun_hardware`.

## Message Interfaces

### Connection Status

- `msg/ConnectionStatus.msg`
  - returns timestamp, connection state, robot IP, and status description

### Robot Runtime Status

- `msg/EstunRobotStatus.msg`
  - returns connection status, error state, current command mode, plus queue and servo statistics

## Service Interfaces

### Connection and Error

- `srv/GetConnectionStatus.srv`
- `srv/GetRobotConnStatus.srv`
- `srv/GetCurErrMsg.srv`

### Runtime Status

- `srv/GetWorldCpos.srv`
- `srv/GetJointValue.srv`

### Tool / User Coordinate Frames

- `srv/GetTool.srv`
- `srv/GetUser.srv`

### DO (Digital Output)

- `srv/SetDo.srv`
- `srv/GetDo.srv`

## Usage Relationships

- `estun_hardware`: creates and exposes these services at runtime
- upper-layer applications: read robot state or call SDK capabilities through these messages and services

## Build

```bash
cd ~/estun_ws
colcon build --packages-select estun_msgs
source install/setup.bash
```
