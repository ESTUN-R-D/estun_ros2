// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#ifndef ESTUN_HARDWARE__ESTUN_SERVO_TRACE_HPP_
#define ESTUN_HARDWARE__ESTUN_SERVO_TRACE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "estun_libs/estun_servo_stream_engine.hpp"
#include "rclcpp/logger.hpp"

namespace estun_hardware
{

class EstunServoTrace
{
public:
  void reset(bool enabled, size_t capacity, const std::string & file);

  void record(
    const std::array<double, 6> & packet_values,
    const std::array<double, 6> & sdk_values,
    uint64_t block_duration_ns,
    uint64_t status_packet_count,
    uint64_t sdk_timestamp,
    const estun_libs::EstunServoSendMeta * send_meta = nullptr);

  void dump_to_file(const rclcpp::Logger & logger);

  static std::string default_trace_dir();

private:
  struct Entry
  {
    uint64_t seq{0};
    int64_t steady_time_ns{0};
    uint64_t status_packet_count{0};
    uint64_t sdk_timestamp{0};
    uint8_t send_source{0};  // 0=stream 1=hold
    bool prefill_wait{false};
    bool low_water_wait{false};
    bool allow_pop{false};
    uint8_t pop_count_this_cycle{0};
    uint32_t queue_depth{0};
    bool repeated_send{false};
    // 仅记录 J1（索引 0），减小 trace 体积。
    double packet_j1{0.0};  // dispatch 前包值
    double sdk_j1{0.0};     // 实际 SDK 入参值
    uint64_t block_duration_ns{0};
  };

  bool enabled_{false};
  size_t capacity_{900000};
  std::string configured_file_;
  std::vector<Entry> entries_;
  uint64_t sequence_{0};
  uint64_t overflow_count_{0};
  std::string runtime_file_;
};

}  // namespace estun_hardware

#endif  // ESTUN_HARDWARE__ESTUN_SERVO_TRACE_HPP_
