#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${script_dir}/../../.." && pwd)"
ros_home="${ESTUN_TEST_ROS_HOME:-${workspace_root}/log/test_ros_home}"
ros_log_dir="${ESTUN_TEST_ROS_LOG_DIR:-${ros_home}/log}"
ros_setup="/opt/ros/humble/setup.bash"
workspace_setup="${workspace_root}/install/setup.bash"

# 统一测试入口显式设置 ROS 运行目录，避免测试默认写 ~/.ros/log 依赖外部 HOME 可写。
mkdir -p "${ros_log_dir}"
export ROS_HOME="${ros_home}"
export ROS_LOG_DIR="${ros_log_dir}"

if [[ -f "${ros_setup}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  set -u
fi

if [[ -f "${workspace_setup}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${workspace_setup}"
  set -u
fi

cd "${workspace_root}"
colcon test "$@"
