// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#ifndef ESTUN_HARDWARE__ESTUN_HARDWARE_CONFIG_HPP_
#define ESTUN_HARDWARE__ESTUN_HARDWARE_CONFIG_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace estun_hardware
{

inline constexpr size_t kEstunStreamMaxQueueDepth = 100;

enum class EstunStreamPolicy
{
  FIFO = 0,
  LATEST_OVERWRITE = 1,
};

struct EstunHardwareConfig
{
  std::string prefix;
  std::string robot_ip;
  uint16_t cmd_port;
  uint16_t servo_port;
  uint16_t status_port;
  std::string servo_mode;
  EstunStreamPolicy stream_policy{EstunStreamPolicy::FIFO};
  uint16_t motion_period_ms{4};
  size_t stream_target_depth{5};
  double servo_tracker_lookahead_time{0.03};
  double servo_tracker_gain{2000.0};
  double servo_tracker_max_velocity{180.0};
  double servo_tracker_max_acceleration{3000.0};
  std::array<double, 6> servo_tracker_axis_max_velocity{{
    180.0, 180.0, 180.0, 180.0, 180.0, 180.0
  }};
  std::array<double, 6> servo_tracker_axis_max_acceleration{{
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0
  }};
  double cartesian_max_trans_velocity{1000.0};      // mm/s
  double cartesian_max_trans_acceleration{2250.0};  // mm/s^2
  double cartesian_max_rot_velocity{1.57};          // rad/s
  double cartesian_max_rot_acceleration{52.35987755982988};  // rad/s^2
  std::string robot_model;
  bool servo_trace_enable{false};
  size_t servo_trace_capacity{900000};
  std::string servo_trace_file{};
  // Test-only fault injection toggles (disabled by default).
  bool test_stub_sdk{false};
  bool test_force_disconnect{false};
  bool test_force_alarm{false};
  bool test_force_service_bridge_fail{false};
};

EstunHardwareConfig parse_estun_hardware_config(
  const std::unordered_map<std::string, std::string> & hardware_parameters);

}  // namespace estun_hardware

#endif  // ESTUN_HARDWARE__ESTUN_HARDWARE_CONFIG_HPP_
