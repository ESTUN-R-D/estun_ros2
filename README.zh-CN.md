# estun_ros2

![ESTUN ROS 2](./images/estun.png "ESTUN ROS 2 Control Driver")

`estun_ros2` 是 ESTUN 工业机器人 ROS 2 支持包主仓库，提供基于 ERI 接口的控制、规划配置、消息接口和 SDK 绑定能力。机器人机型描述由配套仓库 [`estun_description`](https://github.com/ESTUN-R-D/estun_description) 提供。

语言：中文 | [English](./README.md)

注：本仓库当前面向 ROS 2 Humble Hawksbill。

## 概述

本仓库提供 ESTUN 机器人 ROS 2 支持包的源码，包括：

- `ros2_control`（ROS 2 控制框架）硬件接口
- MoveIt 配置与演示入口
- ERI SDK 绑定与运行时服务接口
- 关节空间与笛卡尔空间实时流控制能力

机器人描述、网格、运动学参数和机型限速参数由配套仓库 [`estun_description`](https://github.com/ESTUN-R-D/estun_description) 维护；使用时需要和本仓库同级放在工作区 `src/` 下。

## 文档

详细说明请查看各包 README：

- [estun_hardware/README.zh-CN.md](./estun_hardware/README.zh-CN.md)
- [estun_moveit_config/README.zh-CN.md](./estun_moveit_config/README.zh-CN.md)
- [estun_description/README.zh-CN.md](https://github.com/ESTUN-R-D/estun_description/blob/main/README.zh-CN.md)
- [estun_libs/README.zh-CN.md](./estun_libs/README.zh-CN.md)
- [estun_msgs/README.zh-CN.md](./estun_msgs/README.zh-CN.md)

推荐阅读顺序：

1. `estun_hardware`
2. `estun_moveit_config`
3. `estun_description`
4. `estun_libs`
5. `estun_msgs`

## 安装

先安装 Git LFS（Git Large File Storage，大文件存储）并创建工作区，然后同时克隆主仓库和机型描述仓库：

```bash
git lfs install

mkdir -p ~/estun_ws/src
cd ~/estun_ws/src
git clone https://github.com/ESTUN-R-D/estun_ros2.git
git clone https://github.com/ESTUN-R-D/estun_description.git

cd ~/estun_ws
colcon build --packages-select \
  estun_msgs \
  estun_libs \
  estun_description \
  estun_controllers \
  estun_hardware \
  estun_moveit_config
source install/setup.bash
```

如果使用 `vcs`（多仓库导入工具）管理工作区，也可以使用本仓库提供的 [`estun_ros2.repos`](./estun_ros2.repos) 清单。

更具体的启动参数、运行模式和接口说明，请分别查看各包 README。

## 快速开始

真实机器人控制入口：

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=plan
```

虚拟硬件调试入口：

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=true \
  model:=ER20-1780-A6 \
  control_mode:=apos
```

MoveIt 演示入口：

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=true \
  model:=ER20-1780-A6 \
  control_mode:=plan
```

说明：`robot_ip` 在真机模式下必须显式传入；`use_fake_hardware:=true` 时可以留空。

## 仓库结构

本仓库当前主要包含以下组件：

- `estun_hardware`
- `estun_libs`
- `estun_moveit_config`
- `estun_msgs`
- `estun_controllers`

配套机型描述仓库：

- [`estun_description`](https://github.com/ESTUN-R-D/estun_description)

`libEstunMotion` 是 ESTUN 内部维护的独立 CMake 工程，用于生成 stream smoothing 发布态资产。发布目录携带的是 `estun_libs/lib/libEstunMotion.so` 和 `estun_libs/include/estun_motion/EstunMotion.hpp`，不要求同时携带 `libEstunMotion/` 源码目录。

## 许可

本仓库是混合许可仓库，包含开源包装层代码，也包含 `estun_libs` 中承载的 SDK 二进制与相关分发材料。

其中：

1. 大部分 ROS 包装层源码采用 `Apache-2.0`
2. `estun_libs` 中的 SDK 头文件、共享库和相关运行时配置包含专有许可边界
3. 仓库根 `LICENSE` 是混合许可说明入口；`Apache-2.0` 正文请查看 [Apache-2.0 文本](./LICENSES/Apache-2.0.txt)
4. `estun_libs` 的具体文件级映射、第三方组件说明和再分发要求，请查看：
   - [estun_libs 说明](./estun_libs/README.zh-CN.md)
   - [再分发说明](./estun_libs/REDISTRIBUTION.md)
   - [第三方说明](./estun_libs/THIRD_PARTY_NOTICES.md)
