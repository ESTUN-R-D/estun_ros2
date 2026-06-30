# estun_libs

Language: [Chinese](./README.zh-CN.md) | English

`estun_libs` is the ESTUN ERI SDK asset distribution package. It provides headers, shared libraries, and runtime configuration files to other ROS 2 packages. This package does not start the robot by itself.

## What It Contains

The current deliverables include:

1. SDK header: `ERIParamManager.h`
2. SDK header: `EstunRobotERI.h`
3. public ROS header: `estun_libs/estun_servo_stream_engine.hpp`
4. release-form motion-library header: `estun_motion/EstunMotion.hpp`
5. SDK shared library: `libEstunRobotERI.so`
6. third-party shared library: `liblog4cpp.so`
7. release-form motion library: `libEstunMotion.so`
8. runtime configuration: `log4cpp.conf`

## Package Relationships

- `estun_hardware`: links against and uses the SDK assets distributed by this package
- other ESTUN ROS 2 packages: reuse the unified SDK distribution boundary provided by this package

## Purpose

This package is designed to:

1. centralize SDK assets in a single distribution location
2. keep a clear boundary between the control layer and SDK binaries
3. provide a stable runtime layout for installation, deployment, and version management

## Build

This package is built and installed with the workspace:

```bash
cd ~/estun_ws
colcon build --packages-select estun_libs
source install/setup.bash
```

## License and Distribution

This package has a mixed-license boundary:

1. ROS wrapper source code, build scripts, and documentation are provided under `Apache-2.0`
2. ESTUN SDK headers, ESTUN shared libraries, and runtime configuration are provided under `Proprietary`
3. the third-party `liblog4cpp.so` is recorded under `LGPL-2.1-only`

The company authorization scope is:

1. `ESTUN AUTOMATION CO., LTD.` grants use and redistribution rights for ESTUN-owned SDK headers, shared libraries, and runtime configuration distributed in this package
2. that authorization covers internal repositories, customer delivery, and public GitHub repository distribution
3. external redistribution must retain the license, redistribution, and third-party notice materials shipped with this package

Review the following files together to understand the package boundary:

1. `LICENSE`
2. `REDISTRIBUTION.md`
3. `THIRD_PARTY_NOTICES.md`
4. `LICENSES/`

## Authorization Source

The proprietary ESTUN assets in this package are authorized by `ESTUN AUTOMATION CO., LTD.`.

That means:

1. `LICENSES/ESTUN-SDK-REDISTRIBUTION.txt` is the formal license text for the proprietary assets in this package
2. `REDISTRIBUTION.md` explains file-level mappings, delivery scope, and redistribution requirements
3. external distribution of this package follows the license texts shipped with the current package contents
