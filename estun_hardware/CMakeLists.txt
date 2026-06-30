cmake_minimum_required(VERSION 3.8)
project(estun_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# 1. 查找依赖
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(estun_libs REQUIRED)
find_package(estun_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
include(GNUInstallDirs)
include(${CMAKE_CURRENT_SOURCE_DIR}/../tools/cmake/EstunTestEnv.cmake)

# ---------------------------------------------------------
# 2. 构建硬件接口库
# ---------------------------------------------------------
add_library(estun_hardware_interface SHARED
  src/estun_hardware_config.cpp
  src/estun_hardware_interface.cpp
  src/estun_service_bridge.cpp
  src/estun_servo_trace.cpp
  src/estun_status_cache.cpp
)

target_include_directories(estun_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(estun_hardware_interface
  estun_libs::estun_libs_sdk
  estun_libs::estun_servo_stream_engine
  yaml-cpp
  ${CMAKE_DL_LIBS}
  estun_msgs::estun_msgs__rosidl_typesupport_cpp
  estun_msgs::estun_msgs__rosidl_typesupport_fastrtps_cpp
)

# 保证安装后的插件可跨包查找 estun_libs 与 estun_msgs 的运行时库。
set_target_properties(estun_hardware_interface PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../estun_libs/lib"
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../../estun_libs/lib;$ORIGIN/../../estun_msgs/lib"
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

ament_target_dependencies(estun_hardware_interface
  ament_index_cpp
  estun_libs
  estun_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml_cpp_vendor
)

# ---------------------------------------------------------
# 3. 构建 estun_control_node（自定义控制主循环）
# ---------------------------------------------------------
add_executable(estun_control_node
  src/estun_control_node.cpp
)

target_include_directories(estun_control_node PRIVATE
  include
)

ament_target_dependencies(estun_control_node
  controller_manager
  estun_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  yaml_cpp_vendor
)

target_link_libraries(estun_control_node
  estun_hardware_interface
)

# ---------------------------------------------------------
# 4. 插件导出 (ros2_control 必须)
# ---------------------------------------------------------
pluginlib_export_plugin_description_file(hardware_interface estun_hardware.xml)

# ---------------------------------------------------------
# 5. 安装规则
# ---------------------------------------------------------

ament_python_install_package(estun_hardware_launch)

# A. 安装 C++ 编译产物
install(TARGETS estun_hardware_interface
  EXPORT export_estun_hardware
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION include
)

install(TARGETS estun_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller_state_converger.py
  scripts/setup_realtime_caps.sh
  DESTINATION lib/${PROJECT_NAME}
)

# B. 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# C. 安装资源目录
install(DIRECTORY
  config
  launch
  robot
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
)

# D. 安装插件描述 XML 到 share 下
install(FILES
  estun_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # xmllint 依赖远程 schema；旧 SDK 适配层仍有历史版权头与 C/C++ 风格债，先保留其余门禁。
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_xmllint
    ament_cmake_xmllint
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
  estun_get_ros_test_environment(
    test_estun_hardware_runtime
    test_estun_hardware_runtime_env
  )

  ament_add_gtest(
    test_estun_hardware_runtime
    test/test_estun_hardware_runtime.cpp
    ENV ${test_estun_hardware_runtime_env}
  )
  if(TARGET test_estun_hardware_runtime)
    target_include_directories(test_estun_hardware_runtime PRIVATE
      include
    )
    target_link_libraries(test_estun_hardware_runtime
      estun_hardware_interface
    )
    ament_target_dependencies(test_estun_hardware_runtime
      ament_index_cpp
      estun_libs
      estun_msgs
      hardware_interface
      pluginlib
      rclcpp
      rclcpp_lifecycle
      yaml_cpp_vendor
    )
    set_tests_properties(test_estun_hardware_runtime PROPERTIES
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../estun_libs/config
    )
  endif()

  estun_get_ros_test_environment(
    test_estun_hardware_config
    test_estun_hardware_config_env
  )

  ament_add_gtest(
    test_estun_hardware_config
    test/test_estun_hardware_config.cpp
    src/estun_hardware_config.cpp
    ENV ${test_estun_hardware_config_env}
  )
  if(TARGET test_estun_hardware_config)
    target_include_directories(test_estun_hardware_config PRIVATE
      include
    )
  endif()

  estun_get_ros_test_environment(
    test_estun_status_cache
    test_estun_status_cache_env
  )

  ament_add_gtest(
    test_estun_status_cache
    test/test_estun_status_cache.cpp
    src/estun_status_cache.cpp
    ENV ${test_estun_status_cache_env}
  )
  if(TARGET test_estun_status_cache)
    target_include_directories(test_estun_status_cache PRIVATE
      include
    )
    ament_target_dependencies(test_estun_status_cache
      estun_libs
    )
  endif()
endif()

# ---------------------------------------------------------
# 6. Ament 导出
# ---------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(estun_hardware_interface)
ament_export_targets(export_estun_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  estun_libs
  estun_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml_cpp_vendor
)

ament_package()
