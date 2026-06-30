cmake_minimum_required(VERSION 3.8)
project(estun_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(estun_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/../tools/cmake/EstunTestEnv.cmake)

add_library(estun_controllers SHARED
  src/estun_do_controller.cpp
  src/estun_state_broadcaster.cpp
)

target_include_directories(estun_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(estun_controllers
  controller_interface
  estun_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(controller_interface estun_controllers.xml)

install(TARGETS estun_controllers
  EXPORT export_estun_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES estun_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_estun_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  estun_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # xmllint 依赖远程 schema；copyright 头格式尚未在仓内统一，先保留其余门禁。
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_xmllint
    ament_cmake_xmllint
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_estun_controllers_runtime
    test/test_estun_controllers_runtime.cpp
  )
  if(TARGET test_estun_controllers_runtime)
    target_link_libraries(test_estun_controllers_runtime estun_controllers)
    ament_target_dependencies(test_estun_controllers_runtime
      controller_interface
      estun_msgs
      hardware_interface
      pluginlib
      rclcpp
      rclcpp_lifecycle
    )
    estun_configure_ros_test_environment(test_estun_controllers_runtime)
  endif()
endif()

ament_package()
