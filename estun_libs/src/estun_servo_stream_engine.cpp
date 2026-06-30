// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include "estun_libs/estun_servo_stream_engine.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

// RELEASE_BOUNDARY:
// EstunServoStreamEngine calls stream smoothing through libEstunMotion. Keep
// product-facing code on this engine interface, and keep algorithm mechanics
// inside the external library artifact for release packaging.
#include "estun_motion/EstunMotion.hpp"

namespace estun_libs
{
namespace
{
constexpr double kRepeatCommandEpsilon = 1e-9;
constexpr size_t kInternalStreamLowWatermark = 3;
constexpr size_t kInternalStreamMaxQueueDepth = 100;
constexpr double kInternalDepthFilterAlpha = 0.02;
constexpr double kInternalDepthGain = 0.01;
constexpr double kInternalMinPhaseStep = 0.95;
constexpr double kInternalMaxPhaseStep = 1.05;

bool approx_equal(
  const std::array<double, 6> & lhs,
  const std::array<double, 6> & rhs,
  double epsilon)
{
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (std::fabs(lhs[i] - rhs[i]) > epsilon) {
      return false;
    }
  }
  return true;
}

estun_motion::StreamSmoothingConfig make_stream_smoothing_config(
  const EstunServoStreamConfig & config)
{
  estun_motion::StreamSmoothingConfig smoothing_config;
  smoothing_config.target_depth = config.target_depth;
  if (config.command_mode == EstunServoStreamMode::CPOS) {
    smoothing_config.angular_wrap_axes[3] = true;
    smoothing_config.angular_wrap_axes[4] = true;
    smoothing_config.angular_wrap_axes[5] = true;
  }
  return smoothing_config;
}

estun_motion::LatestServoTrackerConfig make_latest_servo_tracker_config(
  const EstunServoStreamConfig & config)
{
  estun_motion::LatestServoTrackerConfig tracker_config;
  tracker_config.mode = config.command_mode == EstunServoStreamMode::CPOS ?
    estun_motion::LatestServoTrackerMode::CPOS_ZYX :
    estun_motion::LatestServoTrackerMode::APOS;
  tracker_config.sample_period =
    std::chrono::duration<double>(config.period).count();
  tracker_config.lookahead_time = config.servo_tracker_lookahead_time;
  tracker_config.gain = config.servo_tracker_gain;
  tracker_config.max_velocity = config.servo_tracker_max_velocity;
  tracker_config.max_acceleration = config.servo_tracker_max_acceleration;
  tracker_config.axis_max_velocity = config.servo_tracker_axis_max_velocity;
  tracker_config.axis_max_acceleration = config.servo_tracker_axis_max_acceleration;
  tracker_config.cartesian_max_trans_velocity = config.cartesian_max_trans_velocity;
  tracker_config.cartesian_max_trans_acceleration = config.cartesian_max_trans_acceleration;
  tracker_config.cartesian_max_rot_velocity = config.cartesian_max_rot_velocity;
  tracker_config.cartesian_max_rot_acceleration = config.cartesian_max_rot_acceleration;
  return tracker_config;
}
}  // namespace

EstunServoStreamEngine::EstunServoStreamEngine(
  const EstunServoStreamConfig & config,
  SendCallback send_callback)
: config_(config),
  send_callback_(std::move(send_callback)),
  stream_smoothing_queue_(
    std::make_unique<estun_motion::StreamSmoothingQueue>(
      make_stream_smoothing_config(config_))),
  latest_servo_tracker_(
    std::make_unique<estun_motion::LatestServoTracker>(
      make_latest_servo_tracker_config(config_)))
{
}

EstunServoStreamEngine::~EstunServoStreamEngine()
{
  stop();
}

void EstunServoStreamEngine::start(const std::array<double, 6> & hold_values)
{
  stop();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    reset_state_locked(hold_values);

    stat_write_total_.store(0, std::memory_order_relaxed);
    stat_take_total_.store(0, std::memory_order_relaxed);
    stat_request_total_.store(0, std::memory_order_relaxed);
    stat_queue_max_.store(0, std::memory_order_relaxed);
    stat_interp_send_.store(0, std::memory_order_relaxed);
    stat_hold_send_.store(0, std::memory_order_relaxed);
    stat_underflow_count_.store(0, std::memory_order_relaxed);
    stat_stale_drop_count_.store(0, std::memory_order_relaxed);
    stat_timing_overrun_count_.store(0, std::memory_order_relaxed);
    stat_pop_hit_count_.store(0, std::memory_order_relaxed);
    stat_pop_depth_sum_.store(0, std::memory_order_relaxed);
    stat_zero_pop_cycles_.store(0, std::memory_order_relaxed);
    stat_multi_pop_cycles_.store(0, std::memory_order_relaxed);
    stat_repeated_send_count_.store(0, std::memory_order_relaxed);
  }

  running_.store(true, std::memory_order_release);
  worker_thread_ = std::thread([this]() {worker_loop();});
}

void EstunServoStreamEngine::stop()
{
  const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
  if (!was_running) {
    return;
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool EstunServoStreamEngine::enqueue(
  const std::array<double, 6> & values,
  TimePoint steady_timestamp)
{
  if (!running_.load(std::memory_order_acquire)) {
    return false;
  }

  if (config_.stream_policy == EstunServoStreamPolicy::LATEST_OVERWRITE) {
    return updateLatestCommand(values, steady_timestamp);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_smoothing_queue_->push(values)) {
    return false;
  }

  stat_write_total_.fetch_add(1, std::memory_order_relaxed);

  const size_t queue_size = stream_smoothing_queue_->depth();
  size_t observed_max = stat_queue_max_.load(std::memory_order_relaxed);
  while (queue_size > observed_max &&
    !stat_queue_max_.compare_exchange_weak(
      observed_max,
      queue_size,
      std::memory_order_relaxed,
      std::memory_order_relaxed))
  {
  }

  return true;
}

bool EstunServoStreamEngine::updateLatestCommand(
  const std::array<double, 6> & values,
  TimePoint steady_timestamp)
{
  if (!running_.load(std::memory_order_acquire)) {
    return false;
  }
  if (config_.stream_policy != EstunServoStreamPolicy::LATEST_OVERWRITE) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_command_values_ = values;
  latest_command_timestamp_ = steady_timestamp;
  latest_command_valid_ = true;
  latest_servo_tracker_->updateTarget(
    values,
    std::chrono::duration<double>(steady_timestamp.time_since_epoch()).count());

  stat_write_total_.fetch_add(1, std::memory_order_relaxed);
  size_t observed_max = stat_queue_max_.load(std::memory_order_relaxed);
  while (observed_max < 1 &&
    !stat_queue_max_.compare_exchange_weak(
      observed_max,
      1,
      std::memory_order_relaxed,
      std::memory_order_relaxed))
  {
  }

  return true;
}

void EstunServoStreamEngine::flush(const std::array<double, 6> & hold_values)
{
  std::lock_guard<std::mutex> lock(mutex_);
  reset_state_locked(hold_values);
}

void EstunServoStreamEngine::set_mode(
  EstunServoStreamMode mode,
  const std::array<double, 6> & hold_values)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_.command_mode = mode;
  reset_state_locked(hold_values);
}

EstunServoStreamStats EstunServoStreamEngine::snapshot() const
{
  EstunServoStreamStats stats;
  size_t queue_depth = 0;
  size_t target_depth = 0;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_.stream_policy == EstunServoStreamPolicy::LATEST_OVERWRITE) {
      queue_depth = latest_command_valid_ ? 1u : 0u;
    } else {
      const auto smoothing_snapshot = stream_smoothing_queue_->snapshot();
      queue_depth = smoothing_snapshot.queue_depth;
      stats.smoothing_phase = smoothing_snapshot.phase;
      stats.smoothing_phase_step = smoothing_snapshot.phase_step;
      stats.smoothing_filtered_depth = smoothing_snapshot.filtered_depth;
    }
    target_depth = config_.target_depth;
  }

  stats.write_total = stat_write_total_.load(std::memory_order_relaxed);
  stats.take_total = stat_take_total_.load(std::memory_order_relaxed);
  stats.request_total = stat_request_total_.load(std::memory_order_relaxed);
  stats.diff_total =
    static_cast<int64_t>(stats.write_total) - static_cast<int64_t>(stats.take_total);
  stats.zero_pop_cycles = stat_zero_pop_cycles_.load(std::memory_order_relaxed);
  stats.multi_pop_cycles = stat_multi_pop_cycles_.load(std::memory_order_relaxed);

  stats.queue_depth = queue_depth;
  stats.queue_max = stat_queue_max_.load(std::memory_order_relaxed);
  stats.target_depth = target_depth;

  stats.interp_send = stat_interp_send_.load(std::memory_order_relaxed);
  stats.hold_send = stat_hold_send_.load(std::memory_order_relaxed);
  stats.underflow_count = stat_underflow_count_.load(std::memory_order_relaxed);
  stats.stale_drop_count = stat_stale_drop_count_.load(std::memory_order_relaxed);
  stats.timing_overrun_count = stat_timing_overrun_count_.load(std::memory_order_relaxed);
  stats.repeated_send_count = stat_repeated_send_count_.load(std::memory_order_relaxed);

  const uint64_t pop_hit_count = stat_pop_hit_count_.load(std::memory_order_relaxed);
  const uint64_t pop_depth_sum = stat_pop_depth_sum_.load(std::memory_order_relaxed);

  stats.pop_hit =
    stats.request_total > 0 ?
    static_cast<double>(pop_hit_count) / static_cast<double>(stats.request_total) :
    0.0;
  stats.avg_pop_depth =
    pop_hit_count > 0 ?
    static_cast<double>(pop_depth_sum) / static_cast<double>(pop_hit_count) :
    0.0;

  return stats;
}

std::array<double, 6> EstunServoStreamEngine::current_hold_values() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return hold_values_;
}

bool EstunServoStreamEngine::running() const
{
  return running_.load(std::memory_order_acquire);
}

void EstunServoStreamEngine::reset_state_locked(const std::array<double, 6> & hold_values)
{
  hold_values_ = hold_values;
  last_sent_values_ = hold_values;
  latest_command_values_ = hold_values;
  latest_command_timestamp_ = TimePoint{};
  latest_command_valid_ = false;

  initial_prefill_pending_ = true;
  low_water_recovery_ = false;
  stream_smoothing_queue_->configure(make_stream_smoothing_config(config_));
  stream_smoothing_queue_->reset(hold_values);
  latest_servo_tracker_->configure(make_latest_servo_tracker_config(config_));
  latest_servo_tracker_->reset(
    hold_values,
    std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

void EstunServoStreamEngine::run_fifo_cycle_locked(
  std::array<double, 6> & values_to_send,
  bool & should_send,
  EstunServoSendMeta & send_meta)
{
  const size_t depth = stream_smoothing_queue_->depth();

  if (initial_prefill_pending_) {
    if (depth >= config_.target_depth) {
      initial_prefill_pending_ = false;
    }
  } else if (kInternalStreamLowWatermark > 0) {
    if (!low_water_recovery_ && depth < kInternalStreamLowWatermark) {
      low_water_recovery_ = true;
    }
    if (low_water_recovery_ && depth >= kInternalStreamLowWatermark) {
      low_water_recovery_ = false;
    }
  } else {
    low_water_recovery_ = false;
  }

  const bool prefill_wait = initial_prefill_pending_;
  const bool low_water_wait = low_water_recovery_;
  // 仅保留 prefill 保护：只要队列非空就允许出队，避免运动段因低水位等待产生重复发送。
  const bool allow_pop = !prefill_wait && stream_smoothing_queue_->depth() > 0;
  send_meta.prefill_wait = prefill_wait;
  send_meta.low_water_wait = low_water_wait;
  send_meta.allow_pop = allow_pop;
  send_meta.target_depth = config_.target_depth;

  const auto read_result = stream_smoothing_queue_->read(allow_pop, !prefill_wait);
  const size_t pop_count_this_cycle = read_result.pop_count_this_cycle;
  bool has_stream_output = read_result.has_output;
  if (read_result.has_output) {
    values_to_send = read_result.values;
  }

  if (pop_count_this_cycle > 0) {
    stat_take_total_.fetch_add(pop_count_this_cycle, std::memory_order_relaxed);
    stat_pop_hit_count_.fetch_add(1, std::memory_order_relaxed);
    stat_pop_depth_sum_.fetch_add(read_result.first_pop_depth, std::memory_order_relaxed);
  }

  if (!prefill_wait && has_stream_output) {
    if (pop_count_this_cycle == 0) {
      stat_zero_pop_cycles_.fetch_add(1, std::memory_order_relaxed);
    } else if (pop_count_this_cycle > 1) {
      stat_multi_pop_cycles_.fetch_add(1, std::memory_order_relaxed);
    }
    const bool repeated_send =
      approx_equal(values_to_send, last_sent_values_, kRepeatCommandEpsilon);
    if (repeated_send) {
      stat_repeated_send_count_.fetch_add(1, std::memory_order_relaxed);
    }
    last_sent_values_ = values_to_send;
    hold_values_ = values_to_send;
    should_send = true;
    send_meta.source = EstunServoSendSource::STREAM;
    send_meta.repeated_send = repeated_send;

    stat_interp_send_.fetch_add(1, std::memory_order_relaxed);
  }

  if (!should_send && prefill_wait) {
    // 预填充阶段固定发送保持值，避免“等蓄满期间不发/低频发”导致链路不连续。
    values_to_send = hold_values_;
    const bool repeated_send =
      approx_equal(values_to_send, last_sent_values_, kRepeatCommandEpsilon);
    if (repeated_send) {
      stat_repeated_send_count_.fetch_add(1, std::memory_order_relaxed);
    }
    should_send = true;
    send_meta.source = EstunServoSendSource::HOLD;
    send_meta.repeated_send = repeated_send;
    stat_hold_send_.fetch_add(1, std::memory_order_relaxed);
  }

  if (!should_send) {
    stat_underflow_count_.fetch_add(1, std::memory_order_relaxed);
    values_to_send = hold_values_;
    const bool repeated_send =
      approx_equal(values_to_send, last_sent_values_, kRepeatCommandEpsilon);
    if (repeated_send) {
      stat_repeated_send_count_.fetch_add(1, std::memory_order_relaxed);
    }
    should_send = true;
    send_meta.source = EstunServoSendSource::HOLD;
    send_meta.repeated_send = repeated_send;
    stat_hold_send_.fetch_add(1, std::memory_order_relaxed);
  }

  send_meta.pop_count_this_cycle = pop_count_this_cycle;
  send_meta.queue_depth = stream_smoothing_queue_->depth();

  if (should_send) {
    stat_request_total_.fetch_add(1, std::memory_order_relaxed);
  }
}

void EstunServoStreamEngine::run_latest_overwrite_cycle_locked(
  std::array<double, 6> & values_to_send,
  bool & should_send,
  EstunServoSendMeta & send_meta)
{
  const auto now = std::chrono::steady_clock::now();
  send_meta.prefill_wait = false;
  send_meta.low_water_wait = false;
  send_meta.allow_pop = latest_command_valid_;
  send_meta.target_depth = config_.target_depth;
  send_meta.queue_depth = latest_command_valid_ ? 1u : 0u;

  if (latest_command_valid_) {
    values_to_send = latest_servo_tracker_->sample(
      std::chrono::duration<double>(now.time_since_epoch()).count());
    const bool repeated_send =
      approx_equal(values_to_send, last_sent_values_, kRepeatCommandEpsilon);
    if (repeated_send) {
      stat_repeated_send_count_.fetch_add(1, std::memory_order_relaxed);
    }

    last_sent_values_ = values_to_send;
    hold_values_ = values_to_send;
    should_send = true;
    send_meta.source = EstunServoSendSource::STREAM;
    send_meta.pop_count_this_cycle = 1;
    send_meta.repeated_send = repeated_send;

    stat_take_total_.fetch_add(1, std::memory_order_relaxed);
    stat_pop_hit_count_.fetch_add(1, std::memory_order_relaxed);
    stat_pop_depth_sum_.fetch_add(1, std::memory_order_relaxed);
    stat_interp_send_.fetch_add(1, std::memory_order_relaxed);
    stat_request_total_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  stat_underflow_count_.fetch_add(1, std::memory_order_relaxed);
  values_to_send = hold_values_;
  const bool repeated_send =
    approx_equal(values_to_send, last_sent_values_, kRepeatCommandEpsilon);
  if (repeated_send) {
    stat_repeated_send_count_.fetch_add(1, std::memory_order_relaxed);
  }

  should_send = true;
  send_meta.source = EstunServoSendSource::HOLD;
  send_meta.repeated_send = repeated_send;
  stat_hold_send_.fetch_add(1, std::memory_order_relaxed);
  stat_request_total_.fetch_add(1, std::memory_order_relaxed);
}

void EstunServoStreamEngine::worker_loop()
{
  const auto period = config_.period;
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  auto next_cycle_time = std::chrono::steady_clock::now();

  while (running_.load(std::memory_order_acquire)) {
    if (!config_.callback_paced) {
      // 非 callback_paced 模式下，严格由本地绝对时钟驱动发送节拍。
      std::this_thread::sleep_until(next_cycle_time);
    }

    std::array<double, 6> values_to_send{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    bool should_send = false;
    EstunServoSendMeta send_meta;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (config_.stream_policy == EstunServoStreamPolicy::LATEST_OVERWRITE) {
        run_latest_overwrite_cycle_locked(values_to_send, should_send, send_meta);
      } else {
        run_fifo_cycle_locked(values_to_send, should_send, send_meta);
      }
    }

    if (should_send) {
      const auto send_begin = std::chrono::steady_clock::now();
      try {
        send_callback_(values_to_send, send_meta);
      } catch (...) {
        // 保持线程存活：发送回调异常仅丢弃本次发送。
      }
      const auto send_elapsed = std::chrono::steady_clock::now() - send_begin;

      if (config_.callback_paced) {
        // 由回调阻塞时长决定节拍；秒返时小睡避免空转。
        if (send_elapsed < std::chrono::milliseconds(1)) {
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        next_cycle_time = std::chrono::steady_clock::now();
        continue;
      }
    }

    if (config_.callback_paced) {
      continue;
    }

    // 绝对时钟推进：只在“确实错过完整周期”时做追赶，并可丢弃陈旧队列点。
    next_cycle_time += period;
    const auto now_after_cycle = std::chrono::steady_clock::now();
    if (period_ns.count() > 0 && now_after_cycle > next_cycle_time) {
      const auto lateness_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_after_cycle - next_cycle_time).count();
      if (lateness_ns > 0) {
        const size_t missed_full_cycles = static_cast<size_t>(
          lateness_ns / period_ns.count());
        if (missed_full_cycles > 0) {
          stat_timing_overrun_count_.fetch_add(missed_full_cycles, std::memory_order_relaxed);
          next_cycle_time += period * static_cast<int64_t>(missed_full_cycles);

          if (config_.stream_policy == EstunServoStreamPolicy::FIFO) {
            size_t dropped = 0;
            {
              std::lock_guard<std::mutex> lock(mutex_);
              dropped = stream_smoothing_queue_->drop_oldest(
                missed_full_cycles, config_.target_depth);
            }
            if (dropped > 0) {
              stat_stale_drop_count_.fetch_add(dropped, std::memory_order_relaxed);
            }
          }
        }
      }
    }
  }
}

}  // namespace estun_libs
