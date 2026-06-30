// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#ifndef ESTUN_LIBS__ESTUN_SERVO_STREAM_ENGINE_HPP_
#define ESTUN_LIBS__ESTUN_SERVO_STREAM_ENGINE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace estun_motion
{
class LatestServoTracker;
class StreamSmoothingQueue;
}  // namespace estun_motion

namespace estun_libs
{
// RELEASE_BOUNDARY:
// The stream smoothing implementation is called through libEstunMotion, so
// algorithm state stays out of this public engine header.

enum class EstunServoStreamMode
{
  APOS = 0,
  CPOS = 1,
};

enum class EstunServoStreamPolicy
{
  FIFO = 0,
  LATEST_OVERWRITE = 1,
};

struct EstunServoStreamConfig
{
  std::chrono::milliseconds period{4};
  size_t target_depth{5};
  EstunServoStreamMode command_mode{EstunServoStreamMode::APOS};
  EstunServoStreamPolicy stream_policy{EstunServoStreamPolicy::FIFO};
  bool callback_paced{false};
  double servo_tracker_lookahead_time{0.03};
  double servo_tracker_gain{2000.0};
  double servo_tracker_max_velocity{180.0};
  double servo_tracker_max_acceleration{3000.0};
  std::array<double, 6> servo_tracker_axis_max_velocity{{180.0, 180.0, 180.0, 180.0, 180.0, 180.0}};
  std::array<double, 6> servo_tracker_axis_max_acceleration{{
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0
  }};
  double cartesian_max_trans_velocity{1000.0};
  double cartesian_max_trans_acceleration{2250.0};
  double cartesian_max_rot_velocity{1.57};
  double cartesian_max_rot_acceleration{52.35987755982988};
};

struct EstunServoStreamStats
{
  uint64_t write_total{0};
  uint64_t take_total{0};
  uint64_t request_total{0};
  int64_t diff_total{0};
  uint64_t zero_pop_cycles{0};
  uint64_t multi_pop_cycles{0};

  size_t queue_depth{0};
  size_t queue_max{0};
  size_t target_depth{0};

  uint64_t interp_send{0};
  uint64_t hold_send{0};
  uint64_t underflow_count{0};
  uint64_t stale_drop_count{0};
  uint64_t timing_overrun_count{0};
  uint64_t repeated_send_count{0};

  double smoothing_phase{0.0};
  double smoothing_phase_step{1.0};
  double smoothing_filtered_depth{0.0};

  double pop_hit{0.0};
  double avg_pop_depth{0.0};
};

enum class EstunServoSendSource : uint8_t
{
  STREAM = 0,  // 正常流输出（含低水位复用当前样本）
  HOLD = 1,    // 空队列保持输出
};

struct EstunServoSendMeta
{
  EstunServoSendSource source{EstunServoSendSource::STREAM};
  bool prefill_wait{false};
  bool low_water_wait{false};
  bool allow_pop{false};
  size_t pop_count_this_cycle{0};
  size_t queue_depth{0};
  size_t target_depth{0};
  bool repeated_send{false};
};

class EstunServoStreamEngine
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using SendCallback = std::function<void (
        const std::array<double, 6> & values,
        const EstunServoSendMeta & meta)>;

  explicit EstunServoStreamEngine(
    const EstunServoStreamConfig & config,
    SendCallback send_callback);
  ~EstunServoStreamEngine();

  EstunServoStreamEngine(const EstunServoStreamEngine &) = delete;
  EstunServoStreamEngine & operator=(const EstunServoStreamEngine &) = delete;

  void start(const std::array<double, 6> & hold_values);
  void stop();

  // 将点位与 steady_clock 时间戳入队。返回 false 表示队列已满或未运行。
  bool enqueue(const std::array<double, 6> & values, TimePoint steady_timestamp);

  // 覆盖最新点位。latest-overwrite 模式下返回 false 表示未运行。
  bool updateLatestCommand(const std::array<double, 6> & values, TimePoint steady_timestamp);

  // 清空队列并重置内部状态。
  void flush(const std::array<double, 6> & hold_values);

  // 切换 APOS/CPOS 模式标记，并同步清空状态。
  void set_mode(EstunServoStreamMode mode, const std::array<double, 6> & hold_values);

  EstunServoStreamStats snapshot() const;
  std::array<double, 6> current_hold_values() const;

  bool running() const;

private:
  void reset_state_locked(const std::array<double, 6> & hold_values);
  void worker_loop();
  void run_fifo_cycle_locked(
    std::array<double, 6> & values_to_send,
    bool & should_send,
    EstunServoSendMeta & send_meta);
  void run_latest_overwrite_cycle_locked(
    std::array<double, 6> & values_to_send,
    bool & should_send,
    EstunServoSendMeta & send_meta);

  mutable std::mutex mutex_;
  EstunServoStreamConfig config_;
  SendCallback send_callback_;

  std::unique_ptr<estun_motion::StreamSmoothingQueue> stream_smoothing_queue_;
  std::unique_ptr<estun_motion::LatestServoTracker> latest_servo_tracker_;

  bool initial_prefill_pending_{true};
  bool low_water_recovery_{false};

  std::array<double, 6> hold_values_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_sent_values_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> latest_command_values_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  TimePoint latest_command_timestamp_{};
  bool latest_command_valid_{false};

  std::atomic<bool> running_{false};
  std::thread worker_thread_;

  std::atomic<uint64_t> stat_write_total_{0};
  std::atomic<uint64_t> stat_take_total_{0};
  std::atomic<uint64_t> stat_request_total_{0};
  std::atomic<size_t> stat_queue_max_{0};
  std::atomic<uint64_t> stat_interp_send_{0};
  std::atomic<uint64_t> stat_hold_send_{0};
  std::atomic<uint64_t> stat_underflow_count_{0};
  std::atomic<uint64_t> stat_stale_drop_count_{0};
  std::atomic<uint64_t> stat_timing_overrun_count_{0};
  std::atomic<uint64_t> stat_pop_hit_count_{0};
  std::atomic<uint64_t> stat_pop_depth_sum_{0};
  std::atomic<uint64_t> stat_zero_pop_cycles_{0};
  std::atomic<uint64_t> stat_multi_pop_cycles_{0};
  std::atomic<uint64_t> stat_repeated_send_count_{0};
};

}  // namespace estun_libs

#endif  // ESTUN_LIBS__ESTUN_SERVO_STREAM_ENGINE_HPP_
