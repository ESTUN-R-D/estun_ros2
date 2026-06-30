cmake_minimum_required(VERSION 3.8)
project(estun_dual_arm_example)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest QUIET)
include(${CMAKE_CURRENT_SOURCE_DIR}/../tools/cmake/EstunTestEnv.cmake)

install(DIRECTORY config launch rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_merger.py
  DESTINATION lib/${PROJECT_NAME}
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
      test_dual_control_launch
      test/test_dual_control_launch.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 120
    )
    estun_configure_ros_test_environment(test_dual_control_launch)

    ament_add_pytest_test(
      test_joint_state_merger
      test/test_joint_state_merger.py
      ENV PYTHONDONTWRITEBYTECODE=1
      TIMEOUT 120
    )
    estun_configure_ros_test_environment(test_joint_state_merger)
  endif()
endif()

ament_package()
