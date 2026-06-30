# estun_libs 再分发说明

本文档说明 `estun_libs` 中各类资产的来源与再分发边界。

## 1. 资产来源

`estun_libs` 用于承载 ESTUN ROS 2 支持包运行所需的 SDK 头文件、共享库和运行时配置。

当前包内资产按来源分为三类：

1. ROS 包装层与工程集成层  
   - 来源：`estun_ros2/estun_libs` 仓库内源码  
   - 代表文件：`CMakeLists.txt`、`package.xml`、`README.md`、`README.zh-CN.md`、`src/estun_servo_stream_engine.cpp`

2. ESTUN ERI SDK 专有资产  
   - 授权主体：`ESTUN AUTOMATION CO., LTD.`  
   - 代表文件：`include/ERIParamManager.h`、`include/EstunRobotERI.h`、`lib/libEstunRobotERI.so`

3. ESTUN stream smoothing 发布态资产  
   - 授权主体：`ESTUN AUTOMATION CO., LTD.`  
   - 生成来源：ESTUN 内部维护的 `libEstunMotion` 独立 CMake 工程；发布包以 `estun_libs` 内的公开头文件和共享库作为交付边界
   - 代表文件：`include/estun_motion/EstunMotion.hpp`、`lib/libEstunMotion.so`

## 2. 文件级许可映射

| 路径 | 许可边界 | 说明 |
|---|---|---|
| `src/` | Apache-2.0 | ROS 包装层源码 |
| `test/` | Apache-2.0 | 单元测试源码 |
| `CMakeLists.txt` / `package.xml` / `README.md` / `README.zh-CN.md` | Apache-2.0 | 工程集成与文档 |
| `include/estun_libs/estun_servo_stream_engine.hpp` | Apache-2.0 | ROS 公开接口 |
| `include/estun_motion/EstunMotion.hpp` | Proprietary | 发布态 SDK/算法头文件边界 |
| `include/ERIParamManager.h` | Proprietary | SDK 头文件 |
| `include/EstunRobotERI.h` | Proprietary | SDK 头文件 |
| `lib/libEstunRobotERI.so` | Proprietary | SDK 共享库 |
| `lib/libEstunMotion.so` | Proprietary | ESTUN 发布态运动库 |
| `lib/liblog4cpp.so` | LGPL-2.1-only | 第三方共享库 |
| `config/log4cpp.conf` | Proprietary | 当前作为 SDK 运行配置随包分发 |

## 3. 再分发原则

1. 对外再分发 `estun_libs` 时，必须同时携带本目录下的：
   - `LICENSE`
   - `LICENSES/Apache-2.0.txt`
   - `LICENSES/ESTUN-SDK-REDISTRIBUTION.txt`
   - `LICENSES/LGPL-2.1.txt`
   - `THIRD_PARTY_NOTICES.md`
   - 本文件

2. `ESTUN AUTOMATION CO., LTD.` 授权本包中的 ESTUN 自有专有资产可用于：
   - 内部仓库分发
   - 客户交付
   - 公开 GitHub 仓库分发

3. 对外再分发时，应保留本包附带的许可证文本、再分发说明和第三方说明文件。

4. 若后续公司授权范围发生变化，应同步刷新：
   - `package.xml`
   - `LICENSE`
   - `LICENSES/ESTUN-SDK-REDISTRIBUTION.txt`
   - `README.md`
   - `README.zh-CN.md`
   - 本文件
