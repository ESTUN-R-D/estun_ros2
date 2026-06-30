// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#ifndef ESTUN_MOTION__ESTUNMOTION_HPP_
#define ESTUN_MOTION__ESTUNMOTION_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace estun_motion
{

using MotionSample = std::array<double, 6>;

struct StreamSmoothingConfig
{
  size_t target_depth{5};
  std::array<bool, 6> angular_wrap_axes{{false, false, false, false, false, false}};
};

struct StreamSmoothingReadResult
{
  bool has_output{false};
  size_t first_pop_depth{0};
  size_t pop_count_this_cycle{0};
  MotionSample values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

struct StreamSmoothingSnapshot
{
  size_t queue_depth{0};
  bool has_current_sample{false};
  bool anchor_valid{false};
  double phase{0.0};
  double phase_step{1.0};
  double filtered_depth{0.0};
};

enum class LatestServoTrackerMode
{
  APOS = 0,
  CPOS_ZYX = 1,
};

struct LatestServoTrackerConfig
{
  LatestServoTrackerMode mode{LatestServoTrackerMode::APOS};
  double sample_period{0.004};
  double lookahead_time{0.03};
  double gain{2000.0};
  double max_velocity{180.0};
  double max_acceleration{3000.0};
  MotionSample axis_max_velocity{{180.0, 180.0, 180.0, 180.0, 180.0, 180.0}};
  MotionSample axis_max_acceleration{{3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0}};
  // CPOS 平移轴使用 mm/s 与 mm/s^2。
  double cartesian_max_trans_velocity{1000.0};
  double cartesian_max_trans_acceleration{2250.0};
  // CPOS 姿态轴使用 rad/s 与 rad/s^2。
  double cartesian_max_rot_velocity{1.57};
  double cartesian_max_rot_acceleration{52.35987755982988};
};

class StreamSmoothingQueue
{
public:
  explicit StreamSmoothingQueue(const StreamSmoothingConfig & config);

  void configure(const StreamSmoothingConfig & config);
  void reset(const MotionSample & hold_values);

  bool push(const MotionSample & values);
  StreamSmoothingReadResult read(bool allow_pop, bool allow_output);
  size_t drop_oldest(size_t max_drop, size_t keep_depth);

  size_t depth() const;
  StreamSmoothingSnapshot snapshot() const;

private:
  struct Command
  {
    MotionSample values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  };

  void pop_front_to_current(StreamSmoothingReadResult & result);
  void read_smoothed(
    bool allow_pop,
    bool allow_output,
    StreamSmoothingReadResult & result);
  size_t future_target_depth() const;

  StreamSmoothingConfig config_;
  std::deque<Command> queue_;

  bool has_current_sample_{false};
  bool anchor_valid_{false};
  double phase_{0.0};
  double phase_step_{1.0};
  double filtered_depth_{0.0};

  Command current_;
};

class LatestServoTracker
{
public:
  explicit LatestServoTracker(const LatestServoTrackerConfig & config);

  void configure(const LatestServoTrackerConfig & config);
  void reset(const MotionSample & hold_values, double time_sec);
  void updateTarget(const MotionSample & values, double time_sec);
  MotionSample sample(double time_sec);

private:
  double effective_sample_period(double time_sec);
  MotionSample sample_apos_servo(double time_sec);
  MotionSample sample_cpos_servo(double time_sec);

  LatestServoTrackerConfig config_;
  MotionSample target_values_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  MotionSample output_values_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  MotionSample predicted_values_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  MotionSample previous_step_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  MotionSample last_step_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  MotionSample velocity_values_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> target_orientation_quat_{{1.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> output_orientation_quat_{{1.0, 0.0, 0.0, 0.0}};
  bool target_valid_{false};
};

}  // namespace estun_motion

#endif  // ESTUN_MOTION__ESTUNMOTION_HPP_
