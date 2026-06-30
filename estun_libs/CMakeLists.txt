cmake_minimum_required(VERSION 3.8)
project(estun_libs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include(GNUInstallDirs)

set(ESTUN_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Estun SDK root path")
set(SDK_INCLUDE_DIR "${ESTUN_SDK_ROOT}/include")
set(SDK_LIB_DIR "${ESTUN_SDK_ROOT}/lib")
set(SDK_CONFIG_DIR "${ESTUN_SDK_ROOT}/config")

set(ERI_LIBRARY "${SDK_LIB_DIR}/libEstunRobotERI.so")
set(LOG4CPP_LIB "${SDK_LIB_DIR}/liblog4cpp.so")
set(LOG4CPP_CONFIG_FILE "${SDK_CONFIG_DIR}/log4cpp.conf")
set(ESTUN_MOTION_LIBRARY "${SDK_LIB_DIR}/libEstunMotion.so")

foreach(required_file
    "${ERI_LIBRARY}"
    "${LOG4CPP_LIB}"
    "${ESTUN_MOTION_LIBRARY}"
    "${LOG4CPP_CONFIG_FILE}")
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR "未找到 Estun SDK 资产: ${required_file}")
  endif()
endforeach()

add_library(estun_libs_sdk INTERFACE)
target_include_directories(estun_libs_sdk INTERFACE
  $<BUILD_INTERFACE:${SDK_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(estun_libs_sdk INTERFACE
  $<BUILD_INTERFACE:${ERI_LIBRARY}>
  $<BUILD_INTERFACE:${LOG4CPP_LIB}>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/libEstunRobotERI.so>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/liblog4cpp.so>
)

# RELEASE_BOUNDARY:
# estun_servo_stream_engine links the release-layout libEstunMotion binary from
# estun_libs/lib. Refresh that binary from the standalone project when needed:
#   cmake -S libEstunMotion -B libEstunMotion/build -DCMAKE_BUILD_TYPE=Release
#   cmake --build libEstunMotion/build
#   cp libEstunMotion/build/libEstunMotion.so estun_libs/lib/libEstunMotion.so
add_library(estun_motion_shared SHARED IMPORTED GLOBAL)
set_target_properties(estun_motion_shared PROPERTIES
  IMPORTED_LOCATION "${ESTUN_MOTION_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(estun_servo_stream_engine SHARED
  src/estun_servo_stream_engine.cpp
)
target_compile_features(estun_servo_stream_engine PUBLIC cxx_std_17)
target_include_directories(estun_servo_stream_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(estun_servo_stream_engine PRIVATE
  estun_motion_shared
)

install(TARGETS estun_libs_sdk
  estun_servo_stream_engine
  EXPORT export_estun_libs
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  "${ERI_LIBRARY}"
  "${LOG4CPP_LIB}"
  "${ESTUN_MOTION_LIBRARY}"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES
  "${LOG4CPP_CONFIG_FILE}"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

install(FILES
  "${LOG4CPP_CONFIG_FILE}"
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  LICENSE
  REDISTRIBUTION.md
  THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY LICENSES/
  DESTINATION share/${PROJECT_NAME}/LICENSES
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # xmllint 依赖远程 schema；SDK 头文件仍保留历史版权头与旧风格，先保留其余门禁。
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_xmllint
    ament_cmake_xmllint
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(
    test_estun_servo_stream_engine
    test/test_estun_servo_stream_engine.cpp
  )
  if(TARGET test_estun_servo_stream_engine)
    target_link_libraries(test_estun_servo_stream_engine estun_servo_stream_engine)
  endif()
  ament_add_gtest(
    test_estun_servo_stream_engine_contracts
    test/test_estun_servo_stream_engine_contracts.cpp
  )
  if(TARGET test_estun_servo_stream_engine_contracts)
    target_link_libraries(test_estun_servo_stream_engine_contracts estun_servo_stream_engine)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_estun_libs HAS_LIBRARY_TARGET)

ament_package()
