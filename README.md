# estun_ros2

![ESTUN ROS 2](./images/estun.png "ESTUN ROS 2 Control Driver")

`estun_ros2` is ESTUN's main ROS 2 support package repository for industrial robots. It provides ERI-based control, planning configuration, message interfaces, and SDK bindings. Robot model descriptions are provided by the companion [`estun_description`](https://github.com/ESTUN-R-D/estun_description) repository.

Language: [Chinese](./README.zh-CN.md) | English

Note: this repository targets ROS 2 Humble Hawksbill.

## Overview

This repository provides source code for ESTUN robot ROS 2 support packages, including:

- `ros2_control` hardware interfaces
- MoveIt configuration and demo entry points
- ERI SDK bindings and runtime service interfaces
- real-time streaming control for joint-space and Cartesian-space motion

Robot descriptions, meshes, kinematic parameters, and machine limits are maintained in the companion [`estun_description`](https://github.com/ESTUN-R-D/estun_description) repository. Use it as a sibling repository under the workspace `src/` directory.

## Documentation

Detailed package-level documentation is available in each package README:

- [estun_hardware/README.md](./estun_hardware/README.md)
- [estun_moveit_config/README.md](./estun_moveit_config/README.md)
- [estun_description/README.md](https://github.com/ESTUN-R-D/estun_description/blob/main/README.md)
- [estun_libs/README.md](./estun_libs/README.md)
- [estun_msgs/README.md](./estun_msgs/README.md)

Recommended reading order:

1. `estun_hardware`
2. `estun_moveit_config`
3. `estun_description`
4. `estun_libs`
5. `estun_msgs`

## Installation

Install Git LFS (Git Large File Storage), create the workspace, then clone both the main repository and the robot description repository:

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

If you manage the workspace with `vcs` (multi-repository import tool), you can also use the [`estun_ros2.repos`](./estun_ros2.repos) manifest provided by this repository.

See each package README for launch arguments, runtime modes, and interface details.

## Quick Start

Real robot control entry:

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=false \
  model:=ER20-1780-A6 \
  robot_ip:=<ROBOT_IP> \
  control_mode:=plan
```

Fake hardware debugging entry:

```bash
ros2 launch estun_hardware estun_control.launch.py \
  use_fake_hardware:=true \
  model:=ER20-1780-A6 \
  control_mode:=apos
```

MoveIt demo entry:

```bash
ros2 launch estun_moveit_config demo.launch.py \
  use_fake_hardware:=true \
  model:=ER20-1780-A6 \
  control_mode:=plan
```

Note: `robot_ip` must be provided explicitly for real hardware. It can be left empty when `use_fake_hardware:=true`.

## Repository Layout

This repository currently includes:

- `estun_hardware`
- `estun_libs`
- `estun_moveit_config`
- `estun_msgs`
- `estun_controllers`

Companion robot description repository:

- [`estun_description`](https://github.com/ESTUN-R-D/estun_description)

`libEstunMotion` is an ESTUN-maintained standalone CMake project used to generate release-form stream smoothing assets. The published layout carries `estun_libs/lib/libEstunMotion.so` and `estun_libs/include/estun_motion/EstunMotion.hpp`; the `libEstunMotion/` source directory is not required in the distributed tree.

## Licensing

This repository is a mixed-license repository and includes both open source wrapper code and SDK binaries plus related distribution materials under `estun_libs`.

In particular:

1. Most ROS wrapper source code is provided under `Apache-2.0`.
2. The SDK headers, shared libraries, and related runtime configuration in `estun_libs` are subject to proprietary license boundaries.
3. The repository root `LICENSE` is a mixed-license overview entry; see [Apache-2.0](./LICENSES/Apache-2.0.txt) for the Apache-2.0 text.
4. For file-level mappings, third-party notices, and redistribution requirements for `estun_libs`, see:
   - [estun_libs README](./estun_libs/README.md)
   - [Redistribution Guide](./estun_libs/REDISTRIBUTION.md)
   - [Third-Party Notices](./estun_libs/THIRD_PARTY_NOTICES.md)
