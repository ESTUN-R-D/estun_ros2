// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include "estun_hardware/estun_servo_trace.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

#include "rclcpp/logging.hpp"

namespace estun_hardware
{
namespace
{
constexpr int kTraceFloatPrecision = 12;

static inline std::string trim_copy(std::string v)
{
  auto not_space = [](unsigned char c) {return !std::isspace(c);};
  v.erase(v.begin(), std::find_if(v.begin(), v.end(), not_space));
  v.erase(std::find_if(v.rbegin(), v.rend(), not_space).base(), v.end());
  return v;
}
}  // namespace

std::string EstunServoTrace::default_trace_dir()
{
  const char * env_dir = std::getenv("ESTUN_SERVO_TRACE_DIR");
  if (env_dir != nullptr) {
    std::string from_env = trim_copy(std::string(env_dir));
    if (!from_env.empty()) {
      return from_env;
    }
  }

  try {
    const std::string cwd = std::filesystem::current_path().string();
    const std::string install_marker = "/install/";
    const auto pos = cwd.find(install_marker);
    if (pos != std::string::npos && pos > 0) {
      return cwd.substr(0, pos);  // 优先回落到工作空间根目录
    }
    if (!cwd.empty() && cwd != "/") {
      return cwd;
    }
  } catch (...) {
    // 回落到 HOME / 当前目录
  }

  const char * home_dir = std::getenv("HOME");
  if (home_dir != nullptr) {
    std::string home = trim_copy(std::string(home_dir));
    if (!home.empty()) {
      return home;
    }
  }
  return ".";
}

void EstunServoTrace::reset(bool enabled, size_t capacity, const std::string & file)
{
  enabled_ = enabled;
  capacity_ = capacity;
  configured_file_ = file;

  if (!enabled_) {
    return;
  }

  entries_.clear();
  entries_.reserve(capacity_);
  sequence_ = 0;
  overflow_count_ = 0;

  if (!configured_file_.empty()) {
    runtime_file_ = configured_file_;
    return;
  }

  const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const std::filesystem::path base_dir(default_trace_dir());
  const std::filesystem::path trace_file =
    base_dir / ("estun_servo_trace_" + std::to_string(now_ms) + ".txt");
  runtime_file_ = trace_file.string();
}

void EstunServoTrace::record(
  const std::array<double, 6> & packet_values,
  const std::array<double, 6> & sdk_values,
  uint64_t block_duration_ns,
  uint64_t status_packet_count,
  uint64_t sdk_timestamp,
  const estun_libs::EstunServoSendMeta * send_meta)
{
  if (!enabled_) {
    return;
  }
  if (entries_.size() >= capacity_) {
    ++overflow_count_;
    return;
  }

  Entry entry;
  entry.seq = ++sequence_;
  entry.steady_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  entry.status_packet_count = status_packet_count;
  entry.sdk_timestamp = sdk_timestamp;
  if (send_meta != nullptr) {
    entry.send_source = static_cast<uint8_t>(send_meta->source);
    entry.prefill_wait = send_meta->prefill_wait;
    entry.low_water_wait = send_meta->low_water_wait;
    entry.allow_pop = send_meta->allow_pop;
    entry.pop_count_this_cycle =
      static_cast<uint8_t>(std::min<size_t>(255, send_meta->pop_count_this_cycle));
    entry.queue_depth = static_cast<uint32_t>(
      std::min<size_t>(std::numeric_limits<uint32_t>::max(), send_meta->queue_depth));
    entry.repeated_send = send_meta->repeated_send;
  } else {
    // 结束帧等非流引擎发送场景。
    entry.send_source = 255;
    entry.prefill_wait = false;
    entry.low_water_wait = false;
    entry.allow_pop = false;
    entry.pop_count_this_cycle = 0;
    entry.queue_depth = 0;
    entry.repeated_send = false;
  }
  entry.packet_j1 = packet_values[0];
  entry.sdk_j1 = sdk_values[0];
  entry.block_duration_ns = block_duration_ns;
  entries_.push_back(entry);
}

void EstunServoTrace::dump_to_file(const rclcpp::Logger & logger)
{
  if (!enabled_ || entries_.empty()) {
    return;
  }

  if (runtime_file_.empty()) {
    reset(enabled_, capacity_, configured_file_);
  }

  std::ofstream out(runtime_file_, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    RCLCPP_WARN(
      logger,
      "servo trace 落盘失败，无法打开文件: %s",
      runtime_file_.c_str());
    return;
  }

  out << std::fixed << std::setprecision(kTraceFloatPrecision);
  out << "seq,steady_time_ns,status_packet_count,sdk_timestamp,send_source,prefill_wait,"
    "low_water_wait,allow_pop,pop_count,queue_depth,repeated_send,"
    "packet_j1,sdk_j1,block_duration_ns\n";
  for (const auto & e : entries_) {
    out << e.seq << ','
        << e.steady_time_ns << ','
        << e.status_packet_count << ','
        << e.sdk_timestamp << ','
        << static_cast<unsigned int>(e.send_source) << ','
        << (e.prefill_wait ? 1 : 0) << ','
        << (e.low_water_wait ? 1 : 0) << ','
        << (e.allow_pop ? 1 : 0) << ','
        << static_cast<unsigned int>(e.pop_count_this_cycle) << ','
        << e.queue_depth << ','
        << (e.repeated_send ? 1 : 0) << ','
        << e.packet_j1 << ','
        << e.sdk_j1 << ','
        << e.block_duration_ns << '\n';
  }
  out.close();

  const std::string overflow_count = std::to_string(overflow_count_);
  RCLCPP_INFO(
    logger,
    "servo trace 已落盘: %s (samples=%zu overflow=%s)",
    runtime_file_.c_str(),
    entries_.size(),
    overflow_count.c_str());
}

}  // namespace estun_hardware
