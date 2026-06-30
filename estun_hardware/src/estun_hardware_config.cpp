// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include "estun_hardware/estun_hardware_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace estun_hardware
{
namespace
{
static inline std::string trim_copy(std::string v)
{
  auto not_space = [](unsigned char c) {return !std::isspace(c);};
  v.erase(v.begin(), std::find_if(v.begin(), v.end(), not_space));
  v.erase(std::find_if(v.rbegin(), v.rend(), not_space).base(), v.end());
  return v;
}

static inline std::string to_lower_copy(std::string v)
{
  std::transform(
    v.begin(), v.end(), v.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return v;
}

static inline bool env_flag_enabled(const char * name)
{
  const char * raw = std::getenv(name);
  if (!raw) {
    return false;
  }
  std::string v = to_lower_copy(trim_copy(std::string(raw)));
  return v == "1" || v == "true" || v == "yes" || v == "on";
}

static inline bool parse_bool_param(
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  const char * name,
  bool default_value)
{
  auto it = hardware_parameters.find(name);
  if (it == hardware_parameters.end() || trim_copy(it->second).empty()) {
    return default_value;
  }

  const std::string value = to_lower_copy(trim_copy(it->second));
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::invalid_argument(std::string(name) + " 非法，必须为 true/false");
}

static inline double parse_double_hardware_param(
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  const char * name,
  double default_value)
{
  auto it = hardware_parameters.find(name);
  if (it == hardware_parameters.end() || trim_copy(it->second).empty()) {
    return default_value;
  }
  const double value = std::stod(trim_copy(it->second));
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " 必须为有限数值");
  }
  return value;
}

static inline uint16_t parse_required_port(
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  const char * name)
{
  const int raw_port = std::stoi(trim_copy(hardware_parameters.at(name)));
  if (raw_port <= 0 ||
    raw_port > static_cast<int>(std::numeric_limits<uint16_t>::max()))
  {
    throw std::out_of_range(std::string(name) + " 超出有效范围 (1..65535)");
  }
  return static_cast<uint16_t>(raw_port);
}

static inline EstunStreamPolicy parse_stream_policy(
  const std::unordered_map<std::string, std::string> & hardware_parameters)
{
  auto it = hardware_parameters.find("stream_policy");
  const std::string stream_policy_raw =
    (it != hardware_parameters.end()) ? to_lower_copy(trim_copy(it->second)) : "fifo";
  if (stream_policy_raw.empty() || stream_policy_raw == "fifo") {
    return EstunStreamPolicy::FIFO;
  }
  if (stream_policy_raw == "latest_overwrite" ||
    stream_policy_raw == "latest-overwrite" ||
    stream_policy_raw == "latest")
  {
    return EstunStreamPolicy::LATEST_OVERWRITE;
  }
  throw std::invalid_argument(
          "stream_policy 非法，仅支持 fifo/latest_overwrite");
}

static inline std::string parse_servo_mode(
  const std::unordered_map<std::string, std::string> & hardware_parameters)
{
  auto it = hardware_parameters.find("servo_mode");
  if (it == hardware_parameters.end() || trim_copy(it->second).empty()) {
    return "apos";
  }

  // 硬件层运行时只支持 APOS/CPOS 两种真实模式；
  // 让 launch 之外的调用入口也能尽早暴露非法配置。
  const std::string value = to_lower_copy(trim_copy(it->second));
  if (value == "apos") {
    return "apos";
  }
  if (value == "cpos") {
    return "cpos";
  }
  throw std::invalid_argument(
          "servo_mode 非法，仅支持 apos/cpos");
}

static inline uint16_t parse_motion_period_ms(
  const std::unordered_map<std::string, std::string> & hardware_parameters)
{
  auto it = hardware_parameters.find("motion_period_ms");
  if (it == hardware_parameters.end() || trim_copy(it->second).empty()) {
    return 4;
  }
  const int raw_motion_period_ms = std::stoi(it->second);
  if (raw_motion_period_ms <= 0 ||
    raw_motion_period_ms > static_cast<int>(std::numeric_limits<uint16_t>::max()))
  {
    throw std::out_of_range("motion_period_ms 超出有效范围 (1..65535)");
  }
  return static_cast<uint16_t>(raw_motion_period_ms);
}

static inline size_t parse_positive_size_param(
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  const char * name,
  size_t default_value)
{
  auto it = hardware_parameters.find(name);
  if (it == hardware_parameters.end() || trim_copy(it->second).empty()) {
    return default_value;
  }
  const int value = std::stoi(it->second);
  if (value <= 0) {
    throw std::out_of_range(std::string(name) + " 必须大于 0");
  }
  return static_cast<size_t>(value);
}
}  // namespace

EstunHardwareConfig parse_estun_hardware_config(
  const std::unordered_map<std::string, std::string> & hardware_parameters)
{
  EstunHardwareConfig config;

  auto prefix_it = hardware_parameters.find("prefix");
  config.prefix = (prefix_it != hardware_parameters.end()) ? prefix_it->second : "";

  config.servo_mode = parse_servo_mode(hardware_parameters);

  config.stream_policy = parse_stream_policy(hardware_parameters);
  config.robot_ip = hardware_parameters.at("robot_ip");
  config.cmd_port = parse_required_port(hardware_parameters, "cmd_port");
  config.servo_port = parse_required_port(hardware_parameters, "servo_port");
  config.status_port = parse_required_port(hardware_parameters, "status_port");
  config.motion_period_ms = parse_motion_period_ms(hardware_parameters);

  config.stream_target_depth =
    parse_positive_size_param(hardware_parameters, "stream_target_depth", 5);
  if (config.stream_target_depth > kEstunStreamMaxQueueDepth) {
    throw std::out_of_range("stream_target_depth 不能大于内部最大队列深度 100");
  }

  config.servo_tracker_lookahead_time =
    parse_double_hardware_param(hardware_parameters, "servo_tracker_lookahead_time", 0.03);
  if (config.servo_tracker_lookahead_time < 0.0) {
    throw std::out_of_range("servo_tracker_lookahead_time 不能为负数");
  }
  config.servo_tracker_gain =
    parse_double_hardware_param(hardware_parameters, "servo_tracker_gain", 2000.0);
  if (config.servo_tracker_gain < 0.0) {
    throw std::out_of_range("servo_tracker_gain 不能为负数");
  }
  config.servo_tracker_max_velocity =
    parse_double_hardware_param(hardware_parameters, "servo_tracker_max_velocity", 180.0);
  if (config.servo_tracker_max_velocity < 0.0) {
    throw std::out_of_range("servo_tracker_max_velocity 不能为负数");
  }
  config.servo_tracker_max_acceleration =
    parse_double_hardware_param(hardware_parameters, "servo_tracker_max_acceleration", 3000.0);
  if (config.servo_tracker_max_acceleration < 0.0) {
    throw std::out_of_range("servo_tracker_max_acceleration 不能为负数");
  }

  auto robot_model_it = hardware_parameters.find("robot_model");
  config.robot_model =
    (robot_model_it != hardware_parameters.end()) ? trim_copy(robot_model_it->second) : "";
  if (config.robot_model.empty()) {
    throw std::invalid_argument("robot_model 不能为空");
  }

  // 默认关闭 trace 落盘，避免每次运行结束自动生成文件。
  // 需要时可通过 ros2_control 硬件参数显式开启。
  config.servo_trace_enable =
    parse_bool_param(hardware_parameters, "servo_trace_enable", false);
  config.servo_trace_capacity =
    parse_positive_size_param(hardware_parameters, "servo_trace_capacity", 900000);

  auto servo_trace_file_it = hardware_parameters.find("servo_trace_file");
  if (servo_trace_file_it != hardware_parameters.end()) {
    config.servo_trace_file = trim_copy(servo_trace_file_it->second);
  } else {
    config.servo_trace_file.clear();
  }

  config.test_stub_sdk = env_flag_enabled("ESTUN_TEST_STUB_SDK");
  config.test_force_disconnect = env_flag_enabled("ESTUN_TEST_FORCE_DISCONNECT");
  config.test_force_alarm = env_flag_enabled("ESTUN_TEST_FORCE_ALARM");
  config.test_force_service_bridge_fail =
    env_flag_enabled("ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL");

  return config;
}

}  // namespace estun_hardware
