cmake_minimum_required(VERSION 3.8)
project(estun_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/ConnectionStatus.msg"
  "msg/EstunRobotStatus.msg"
)

set(srv_files
  "srv/GetConnectionStatus.srv"
  "srv/GetCurErrMsg.srv"
  "srv/GetDo.srv"
  "srv/GetJointValue.srv"
  "srv/GetRobotConnStatus.srv"
  "srv/GetTool.srv"
  "srv/GetUser.srv"
  "srv/GetWorldCpos.srv"
  "srv/SetDo.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  # xmllint 依赖远程 schema，离线/受限网络环境下会出现非代码问题导致误失败。
  set(AMENT_LINT_AUTO_EXCLUDE ament_xmllint ament_cmake_xmllint)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
