if(DEFINED ESTUN_TEST_ENV_CMAKE_INCLUDED)
  return()
endif()
set(ESTUN_TEST_ENV_CMAKE_INCLUDED TRUE)

function(estun_get_ros_test_environment test_name out_var)
  if(NOT test_name)
    message(FATAL_ERROR "estun_get_ros_test_environment 需要传入测试名")
  endif()
  if(NOT out_var)
    message(FATAL_ERROR "estun_get_ros_test_environment 需要传出变量名")
  endif()

  # 统一为 ROS 运行时测试提供包内可写目录，避免默认回落到 ~/.ros/log。
  set(test_ros_home "${CMAKE_CURRENT_BINARY_DIR}/test_ros_home/${test_name}")
  set(test_ros_log_dir "${test_ros_home}/log")
  file(MAKE_DIRECTORY "${test_ros_log_dir}")

  set(${out_var}
    "ROS_HOME=${test_ros_home}"
    "ROS_LOG_DIR=${test_ros_log_dir}"
    PARENT_SCOPE
  )
endfunction()

function(estun_configure_ros_test_environment test_name)
  if(NOT test_name)
    message(FATAL_ERROR "estun_configure_ros_test_environment 需要传入测试名")
  endif()

  estun_get_ros_test_environment("${test_name}" test_env)

  set_property(TEST "${test_name}" APPEND PROPERTY ENVIRONMENT
    ${test_env}
  )
endfunction()
