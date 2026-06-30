# estun_libs

语言：中文 | [English](./README.md)

`estun_libs` 是 ESTUN ERI SDK 资产分发包，用于向其他 ROS 2 包提供头文件、共享库和运行时配置。本包本身不直接启动机器人。

## 包含内容

当前交付内容包括：

1. SDK 头文件：`ERIParamManager.h`
2. SDK 头文件：`EstunRobotERI.h`
3. ROS 公开头文件：`estun_libs/estun_servo_stream_engine.hpp`
4. 发布态运动库头文件：`estun_motion/EstunMotion.hpp`
5. SDK 共享库：`libEstunRobotERI.so`
6. 第三方共享库：`liblog4cpp.so`
7. 发布态运动库：`libEstunMotion.so`
8. 运行时配置：`log4cpp.conf`

## 配套关系

- `estun_hardware`：链接并使用本包提供的 SDK 资产
- 其他 ESTUN ROS 2 包：通过本包复用统一的 SDK 发布边界

## 作用说明

本包的设计目标是：

1. 将 SDK 资产集中在单一分发位置
2. 让控制层与 SDK 二进制边界保持清晰
3. 为安装、部署和版本管理提供稳定的运行时目录

## 构建说明

本包随工作区一起编译和安装：

```bash
cd ~/estun_ws
colcon build --packages-select estun_libs
source install/setup.bash
```

## 许可与分发说明

本包是混合许可边界：

1. ROS 包装层源码、构建脚本和说明文档采用 `Apache-2.0`
2. ESTUN SDK 头文件、ESTUN 共享库和运行时配置采用 `Proprietary`
3. 第三方 `liblog4cpp.so` 按 `LGPL-2.1-only` 口径记录

公司授权口径如下：

1. `ESTUN AUTOMATION CO., LTD.` 对本包内 ESTUN 自有 SDK 头文件、共享库和运行时配置授予使用与再分发授权
2. 上述授权覆盖内部仓库、客户交付以及公开 GitHub 仓库分发
3. 对外再分发时需要保留本包随附的许可、再分发说明和第三方说明材料

请结合以下文件一起理解本包边界：

1. `LICENSE`
2. `REDISTRIBUTION.md`
3. `THIRD_PARTY_NOTICES.md`
4. `LICENSES/`

## 授权来源

本包中 ESTUN 自有专有资产的授权来源是 `ESTUN AUTOMATION CO., LTD.`。

这意味着：

1. `LICENSES/ESTUN-SDK-REDISTRIBUTION.txt` 是本包专有资产的正式授权文本
2. `REDISTRIBUTION.md` 负责说明文件级映射、交付范围和再分发要求
3. 本包对外分发时以当前随包授权文本为准执行
