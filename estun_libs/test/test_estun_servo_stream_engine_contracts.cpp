// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "estun_libs/estun_servo_stream_engine.hpp"

namespace
{
using estun_libs::EstunServoStreamConfig;
using estun_libs::EstunServoStreamEngine;
using estun_libs::EstunServoStreamMode;
using estun_libs::EstunServoStreamPolicy;

TEST(EstunServoStreamEngineContractTest, RejectsEnqueueWhenStopped)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 2;
  config.command_mode = EstunServoStreamMode::APOS;

  EstunServoStreamEngine engine(
    config,
    [](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {});

  const std::array<double, 6> values{{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  EXPECT_FALSE(engine.enqueue(values, std::chrono::steady_clock::now()));

  engine.start(values);
  engine.stop();

  EXPECT_FALSE(engine.enqueue(values, std::chrono::steady_clock::now()));
}

TEST(EstunServoStreamEngineContractTest, EnforcesMaxQueueDepthDuringPrefill)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 100;
  config.command_mode = EstunServoStreamMode::APOS;
  config.callback_paced = true;

  EstunServoStreamEngine engine(
    config,
    [](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto now = std::chrono::steady_clock::now();
  for (size_t i = 0; i < 100; ++i) {
    EXPECT_TRUE(engine.enqueue({{static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0}}, now));
  }
  EXPECT_FALSE(engine.enqueue({{100.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, now));

  const auto stats = engine.snapshot();
  engine.stop();

  EXPECT_EQ(stats.write_total, 100u);
  EXPECT_EQ(stats.queue_depth, 100u);
  EXPECT_EQ(stats.queue_max, 100u);
  EXPECT_EQ(stats.take_total, 0u);
}

TEST(EstunServoStreamEngineContractTest, SetModeFlushesQueuedCommandsAndUpdatesMode)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 100;
  config.command_mode = EstunServoStreamMode::APOS;
  config.callback_paced = true;

  EstunServoStreamEngine engine(
    config,
    [](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto now = std::chrono::steady_clock::now();
  EXPECT_TRUE(engine.enqueue({{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, now));
  EXPECT_TRUE(engine.enqueue({{2.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, now));
  EXPECT_EQ(engine.snapshot().queue_depth, 2u);

  const std::array<double, 6> cpos_hold{{10.0, 20.0, 30.0, 0.1, 0.2, 0.3}};
  engine.set_mode(EstunServoStreamMode::CPOS, cpos_hold);

  const auto stats = engine.snapshot();
  engine.stop();

  EXPECT_EQ(stats.queue_depth, 0u);
  EXPECT_EQ(stats.take_total, 0u);
  EXPECT_EQ(stats.target_depth, config.target_depth);
}

TEST(EstunServoStreamEngineContractTest, LatestOverwriteTracksNewestCommandOnly)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 100;
  config.command_mode = EstunServoStreamMode::APOS;
  config.stream_policy = EstunServoStreamPolicy::LATEST_OVERWRITE;
  config.callback_paced = true;
  config.servo_tracker_max_velocity = 100000.0;
  config.servo_tracker_max_acceleration = 100000.0;

  std::atomic<bool> capture_samples{false};
  std::mutex samples_mutex;
  std::vector<double> samples;
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      if (!capture_samples.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return;
      }
      std::lock_guard<std::mutex> lock(samples_mutex);
      samples.push_back(values[0]);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto now = std::chrono::steady_clock::now();
  for (int i = 1; i <= 20; ++i) {
    EXPECT_TRUE(
      engine.updateLatestCommand(
        {{static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0}},
        now + std::chrono::milliseconds(i)));
  }

  capture_samples.store(true, std::memory_order_relaxed);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const auto stats = engine.snapshot();
  engine.stop();

  EXPECT_EQ(stats.write_total, 20u);
  EXPECT_EQ(stats.queue_depth, 1u);
  EXPECT_EQ(stats.queue_max, 1u);
  EXPECT_GT(stats.take_total, 0u);

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = samples;
  }
  ASSERT_FALSE(local_samples.empty());
  EXPECT_GT(local_samples.back(), 0.0);
  EXPECT_EQ(std::find(local_samples.begin(), local_samples.end(), 1.0), local_samples.end());
}

TEST(EstunServoStreamEngineContractTest, LatestOverwriteTrackerSmoothsTowardNewestCommand)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
  config.command_mode = EstunServoStreamMode::APOS;
  config.stream_policy = EstunServoStreamPolicy::LATEST_OVERWRITE;
  config.callback_paced = true;
  config.servo_tracker_gain = 2000.0;
  config.servo_tracker_max_velocity = 180.0;
  config.servo_tracker_max_acceleration = 3000.0;

  std::mutex samples_mutex;
  std::vector<double> samples;
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      samples.push_back(values[0]);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);
  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const auto stats = engine.snapshot();
  engine.stop();

  EXPECT_EQ(stats.queue_depth, 1u);
  EXPECT_GT(stats.take_total, 0u);

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = samples;
  }
  ASSERT_GT(local_samples.size(), 1u);
  EXPECT_GT(local_samples.back(), 0.0);
  EXPECT_LT(local_samples.front(), 10.0);
}

TEST(EstunServoStreamEngineContractTest, LatestOverwriteAposTrackerFollowsLatestTarget)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
  config.command_mode = EstunServoStreamMode::APOS;
  config.stream_policy = EstunServoStreamPolicy::LATEST_OVERWRITE;
  config.callback_paced = true;
  config.servo_tracker_gain = 2000.0;
  config.servo_tracker_max_velocity = 180.0;
  config.servo_tracker_max_acceleration = 3000.0;

  std::atomic<bool> capture_samples{false};
  std::mutex samples_mutex;
  std::vector<double> samples;
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      if (capture_samples.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(samples_mutex);
        samples.push_back(values[0]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);
  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));
  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{20.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));

  capture_samples.store(true, std::memory_order_relaxed);
  std::this_thread::sleep_for(std::chrono::milliseconds(16));
  engine.stop();

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = samples;
  }
  ASSERT_GE(local_samples.size(), 2u);
  EXPECT_GT(local_samples[0], 0.0);
  EXPECT_LT(local_samples[0], 20.0);
  EXPECT_GT(local_samples[1], local_samples[0]);
}

TEST(EstunServoStreamEngineContractTest, LatestOverwriteAposTrackerUsesPerAxisVelocityLimits)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
  config.command_mode = EstunServoStreamMode::APOS;
  config.stream_policy = EstunServoStreamPolicy::LATEST_OVERWRITE;
  config.callback_paced = true;
  config.servo_tracker_gain = 2000.0;
  config.servo_tracker_max_velocity = 180.0;
  config.servo_tracker_max_acceleration = 1000000.0;
  config.servo_tracker_axis_max_velocity = {{0.5, 180.0, 180.0, 180.0, 180.0, 180.0}};
  config.servo_tracker_axis_max_acceleration = {{
    1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0
  }};

  std::mutex samples_mutex;
  std::vector<double> samples;
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      samples.push_back(values[0]);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);
  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{100.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));

  std::this_thread::sleep_for(std::chrono::milliseconds(8));
  engine.stop();

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = samples;
  }
  ASSERT_FALSE(local_samples.empty());
  EXPECT_NEAR(local_samples.front(), 0.002, 1e-9);
}

TEST(EstunServoStreamEngineContractTest, LatestOverwriteCposTrackerUsesCartesianLimits)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
  config.command_mode = EstunServoStreamMode::CPOS;
  config.stream_policy = EstunServoStreamPolicy::LATEST_OVERWRITE;
  config.callback_paced = true;
  config.servo_tracker_gain = 2000.0;
  config.cartesian_max_trans_velocity = 0.5;
  config.cartesian_max_trans_acceleration = 1000000.0;
  config.cartesian_max_rot_velocity = 0.5;
  config.cartesian_max_rot_acceleration = 1000000.0;

  std::mutex samples_mutex;
  std::vector<std::array<double, 6>> samples;
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      samples.push_back(values);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);
  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{100.0, 0.0, 0.0, 1.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));

  std::this_thread::sleep_for(std::chrono::milliseconds(8));
  engine.stop();

  std::vector<std::array<double, 6>> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = samples;
  }
  ASSERT_FALSE(local_samples.empty());
  EXPECT_NEAR(local_samples.front()[0], 0.002, 1e-9);
  EXPECT_NEAR(local_samples.front()[3], 0.002, 1e-9);
}

TEST(EstunServoStreamEngineContractTest, LatestOverwriteHoldDoesNotAccumulateDepth)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
  config.command_mode = EstunServoStreamMode::APOS;
  config.stream_policy = EstunServoStreamPolicy::LATEST_OVERWRITE;
  config.callback_paced = true;

  EstunServoStreamEngine engine(
    config,
    [](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });

  const std::array<double, 6> hold{{7.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  auto stats = engine.snapshot();

  EXPECT_EQ(stats.write_total, 0u);
  EXPECT_EQ(stats.take_total, 0u);
  EXPECT_EQ(stats.queue_depth, 0u);
  EXPECT_GT(stats.underflow_count, 0u);
  EXPECT_GT(stats.hold_send, 0u);

  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{9.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));
  std::this_thread::sleep_for(std::chrono::milliseconds(12));
  stats = engine.snapshot();
  engine.stop();

  EXPECT_EQ(stats.write_total, 1u);
  EXPECT_EQ(stats.queue_depth, 1u);
  EXPECT_GT(stats.take_total, 0u);
}

TEST(EstunServoStreamEngineContractTest, LatestOverwriteFlushAndSetModeResetLatestState)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
  config.command_mode = EstunServoStreamMode::APOS;
  config.stream_policy = EstunServoStreamPolicy::LATEST_OVERWRITE;
  config.callback_paced = true;

  EstunServoStreamEngine engine(
    config,
    [](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));
  EXPECT_EQ(engine.snapshot().queue_depth, 1u);

  engine.flush(hold);
  EXPECT_EQ(engine.snapshot().queue_depth, 0u);

  EXPECT_TRUE(
    engine.updateLatestCommand(
      {{2.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      std::chrono::steady_clock::now()));
  EXPECT_EQ(engine.snapshot().queue_depth, 1u);

  const std::array<double, 6> cpos_hold{{10.0, 20.0, 30.0, 0.1, 0.2, 0.3}};
  engine.set_mode(EstunServoStreamMode::CPOS, cpos_hold);
  const auto stats = engine.snapshot();
  engine.stop();

  EXPECT_EQ(stats.queue_depth, 0u);
  EXPECT_EQ(stats.target_depth, config.target_depth);
}

TEST(EstunServoStreamEngineContractTest, CurrentHoldValuesFollowLastDeliveredStreamSample)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 2;
  config.command_mode = EstunServoStreamMode::APOS;
  config.callback_paced = true;

  std::mutex samples_mutex;
  std::vector<std::array<double, 6>> samples;
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta & meta) {
      if (meta.source != estun_libs::EstunServoSendSource::STREAM) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
      }
      std::lock_guard<std::mutex> lock(samples_mutex);
      if (samples.size() < 32) {
        samples.push_back(values);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  EXPECT_TRUE(engine.enqueue({{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, base));
  EXPECT_TRUE(
    engine.enqueue(
      {{20.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      base + std::chrono::milliseconds(4)));

  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  const auto current_hold = engine.current_hold_values();
  engine.stop();

  std::vector<std::array<double, 6>> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = samples;
  }

  ASSERT_FALSE(local_samples.empty());
  const auto & last_stream_sample = local_samples.back();
  EXPECT_DOUBLE_EQ(current_hold[0], last_stream_sample[0]);
  EXPECT_DOUBLE_EQ(current_hold[1], last_stream_sample[1]);
  EXPECT_DOUBLE_EQ(current_hold[2], last_stream_sample[2]);
  EXPECT_DOUBLE_EQ(current_hold[3], last_stream_sample[3]);
  EXPECT_DOUBLE_EQ(current_hold[4], last_stream_sample[4]);
  EXPECT_DOUBLE_EQ(current_hold[5], last_stream_sample[5]);
}

}  // namespace
