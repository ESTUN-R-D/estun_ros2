cmake_minimum_required(VERSION 3.8)
project(estun_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest QUIET)
include(${CMAKE_CURRENT_SOURCE_DIR}/../tools/cmake/EstunTestEnv.cmake)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  # xmllint 依赖远程 schema；Python/launch 文件版权头尚未统一，先保留其余门禁。
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_xmllint
    ament_cmake_xmllint
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()

  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(
      test_controller_mode_matrix
      test/test_controller_mode_matrix.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 360
    )
    estun_configure_ros_test_environment(test_controller_mode_matrix)
    ament_add_pytest_test(
      test_launch_logic
      test/test_launch_logic.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 120
    )
    estun_configure_ros_test_environment(test_launch_logic)
    ament_add_pytest_test(
      test_control_mode_contract
      test/test_control_mode_contract.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 120
    )
    estun_configure_ros_test_environment(test_control_mode_contract)
    ament_add_pytest_test(
      test_config_contracts
      test/test_config_contracts.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 120
    )
    estun_configure_ros_test_environment(test_config_contracts)
    ament_add_pytest_test(
      test_message_interface_contracts
      test/test_message_interface_contracts.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 120
    )
    estun_configure_ros_test_environment(test_message_interface_contracts)
    ament_add_pytest_test(
      test_controller_state_converger_logging
      test/test_controller_state_converger_logging.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 120
    )
    estun_configure_ros_test_environment(test_controller_state_converger_logging)
  endif()
endif()

ament_package()
