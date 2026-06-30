// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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

TEST(EstunServoStreamEngineTest, PrefillSendsHoldThenStreamsAfterTargetDepth)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
  config.command_mode = EstunServoStreamMode::APOS;

  std::atomic<uint64_t> callback_total{0};

  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {
      callback_total.fetch_add(1, std::memory_order_relaxed);
    });

  std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  std::array<double, 6> cmd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  for (int i = 0; i < 3; ++i) {
    cmd[0] = static_cast<double>(i);
    EXPECT_TRUE(engine.enqueue(cmd, base + std::chrono::milliseconds(i * 4)));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  const auto prefill_stats = engine.snapshot();
  EXPECT_EQ(prefill_stats.take_total, 0u);
  EXPECT_GT(prefill_stats.hold_send, 0u);
  EXPECT_EQ(prefill_stats.underflow_count, 0u);
  EXPECT_GT(callback_total.load(std::memory_order_relaxed), 0u);

  for (int i = 3; i < 13; ++i) {
    cmd[0] = static_cast<double>(i);
    EXPECT_TRUE(engine.enqueue(cmd, base + std::chrono::milliseconds(i * 4)));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  const auto recovered_stats = engine.snapshot();
  EXPECT_GT(recovered_stats.take_total, 0u);
  EXPECT_GT(recovered_stats.interp_send, 0u);
  EXPECT_EQ(
    recovered_stats.diff_total,
    static_cast<int64_t>(recovered_stats.write_total) -
    static_cast<int64_t>(recovered_stats.take_total));

  engine.stop();
}

TEST(EstunServoStreamEngineTest, ConstantPeriodStreamProducesInterpolatedOutput)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 4;
  config.command_mode = EstunServoStreamMode::APOS;

  std::atomic<uint64_t> callback_total{0};
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {
      callback_total.fetch_add(1, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });

  std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  std::array<double, 6> cmd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  constexpr int kNumPoints = 26;
  for (int i = 0; i < kNumPoints; ++i) {
    cmd[0] = static_cast<double>(i);
    EXPECT_TRUE(engine.enqueue(cmd, base + std::chrono::milliseconds(i * 4)));
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  const auto stats = engine.snapshot();

  EXPECT_EQ(stats.write_total, static_cast<uint64_t>(kNumPoints));
  EXPECT_GT(stats.request_total, 0u);
  EXPECT_GT(stats.interp_send, 0u);
  EXPECT_GT(stats.pop_hit, 0.0);
  EXPECT_EQ(stats.target_depth, config.target_depth);
  EXPECT_GT(callback_total.load(std::memory_order_relaxed), 0u);

  engine.stop();
}

TEST(EstunServoStreamEngineTest, CposModeStreamsQueuedAnglesWithoutWrapArtifacts)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 2;
  config.command_mode = EstunServoStreamMode::CPOS;

  std::mutex samples_mutex;
  std::vector<double> angle_samples;

  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      if (angle_samples.size() < 256) {
        angle_samples.push_back(values[3]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });

  std::array<double, 6> hold{{0.0, 0.0, 0.0, 3.10, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  std::array<double, 6> p1{{0.0, 0.0, 0.0, 3.10, 0.0, 0.0}};
  std::array<double, 6> p2{{0.0, 0.0, 0.0, -3.10, 0.0, 0.0}};

  EXPECT_TRUE(engine.enqueue(p1, base + std::chrono::milliseconds(0)));
  EXPECT_TRUE(engine.enqueue(p2, base + std::chrono::milliseconds(40)));

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  engine.stop();

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = angle_samples;
  }

  ASSERT_GT(local_samples.size(), 8u);

  for (const double sample : local_samples) {
    EXPECT_TRUE(std::isfinite(sample));
    EXPECT_LE(std::fabs(sample), 3.2);
  }

  const auto [min_it, max_it] = std::minmax_element(local_samples.begin(), local_samples.end());
  EXPECT_LT(*min_it, -3.0);
  EXPECT_GT(*max_it, 3.0);
}

TEST(EstunServoStreamEngineTest, CposSmoothingUsesShortestAngularPath)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 2;
  config.command_mode = EstunServoStreamMode::CPOS;
  config.callback_paced = true;

  std::mutex samples_mutex;
  std::vector<double> angle_samples;

  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      if (angle_samples.size() < 32) {
        angle_samples.push_back(values[3]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 3.10, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  EXPECT_TRUE(engine.enqueue({{0.0, 0.0, 0.0, 3.10, 0.0, 0.0}}, base));
  EXPECT_TRUE(
    engine.enqueue(
      {{0.0, 0.0, 0.0, -3.10, 0.0, 0.0}},
      base + std::chrono::milliseconds(4)));

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  engine.stop();

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = angle_samples;
  }

  bool saw_positive = false;
  bool saw_negative = false;
  for (const double sample : local_samples) {
    ASSERT_TRUE(std::isfinite(sample));
    EXPECT_TRUE(sample > 3.0 || sample < -3.0);
    saw_positive = saw_positive || sample > 3.0;
    saw_negative = saw_negative || sample < -3.0;
  }

  EXPECT_TRUE(saw_positive);
  EXPECT_TRUE(saw_negative);
}

TEST(EstunServoStreamEngineTest, AposSmoothingKeepsLinearJointInterpolation)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 2;
  config.command_mode = EstunServoStreamMode::APOS;
  config.callback_paced = true;

  std::mutex samples_mutex;
  std::vector<double> joint_samples;

  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      if (joint_samples.size() < 32) {
        joint_samples.push_back(values[3]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  EXPECT_TRUE(engine.enqueue({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, base));
  EXPECT_TRUE(
    engine.enqueue(
      {{0.0, 0.0, 0.0, 10.0, 0.0, 0.0}},
      base + std::chrono::milliseconds(4)));

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  engine.stop();

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = joint_samples;
  }

  bool saw_positive = false;
  bool saw_negative = false;
  for (const double sample : local_samples) {
    ASSERT_TRUE(std::isfinite(sample));
    EXPECT_GE(sample, 0.0);
    EXPECT_LE(sample, 10.0);
    saw_positive = saw_positive || sample > 0.0;
    saw_negative = saw_negative || sample < 0.0;
  }

  EXPECT_TRUE(saw_positive);
  EXPECT_FALSE(saw_negative);
}

TEST(EstunServoStreamEngineTest, VirtualFifoSlowsPhaseInsteadOfRepeatingPoint)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 4;
  config.command_mode = EstunServoStreamMode::APOS;
  config.callback_paced = true;

  std::mutex samples_mutex;
  std::vector<double> samples;

  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta &) {
      std::lock_guard<std::mutex> lock(samples_mutex);
      if (samples.size() < 256) {
        samples.push_back(values[0]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  std::array<double, 6> cmd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  for (int i = 0; i < 4; ++i) {
    cmd[0] = static_cast<double>(i * 10);
    EXPECT_TRUE(engine.enqueue(cmd, base + std::chrono::milliseconds(i * 4)));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  const auto stats = engine.snapshot();
  engine.stop();

  std::vector<double> local_samples;
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    local_samples = samples;
  }

  bool saw_fractional_phase_sample = false;
  for (const double sample : local_samples) {
    const double nearest_ten = std::round(sample / 10.0) * 10.0;
    if (sample > 1e-6 && std::fabs(sample - nearest_ten) > 1e-6) {
      saw_fractional_phase_sample = true;
      break;
    }
  }

  EXPECT_TRUE(saw_fractional_phase_sample);
  EXPECT_LT(stats.smoothing_phase_step, 1.0);
  EXPECT_GT(stats.zero_pop_cycles, 0u);
}

TEST(EstunServoStreamEngineTest, LocalTimedModeCatchUpDropsStaleCommandsOnBlockingSend)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 1;
  config.command_mode = EstunServoStreamMode::APOS;
  config.callback_paced = false;

  std::atomic<uint64_t> callback_total{0};
  EstunServoStreamEngine engine(
    config,
    [&](const std::array<double, 6> &, const estun_libs::EstunServoSendMeta &) {
      callback_total.fetch_add(1, std::memory_order_relaxed);
      // 人为制造明显超周期阻塞，触发 local_timed 追赶逻辑。
      std::this_thread::sleep_for(std::chrono::milliseconds(12));
    });

  std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  std::array<double, 6> cmd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  for (int i = 0; i < 80; ++i) {
    cmd[0] = static_cast<double>(i);
    EXPECT_TRUE(engine.enqueue(cmd, base + std::chrono::milliseconds(i * 4)));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(260));
  const auto stats = engine.snapshot();
  engine.stop();

  EXPECT_GT(callback_total.load(std::memory_order_relaxed), 0u);
  EXPECT_GT(stats.request_total, 0u);
  EXPECT_GT(stats.take_total, 0u);
  EXPECT_GT(stats.timing_overrun_count, 0u);
  EXPECT_GT(stats.stale_drop_count, 0u);
}

TEST(EstunServoStreamEngineTest, LatestOverwriteDepthStaysOneDuringHighRateCommands)
{
  EstunServoStreamConfig config;
  config.period = std::chrono::milliseconds(4);
  config.target_depth = 8;
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
    [&](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta & meta) {
      if (meta.queue_depth > 1u) {
        ADD_FAILURE() << "latest-overwrite queue_depth exceeded 1";
      }
      if (!capture_samples.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return;
      }
      std::lock_guard<std::mutex> lock(samples_mutex);
      if (samples.size() < 128) {
        samples.push_back(values[0]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });

  const std::array<double, 6> hold{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  engine.start(hold);

  const auto base = std::chrono::steady_clock::now();
  for (int i = 0; i < 80; ++i) {
    EXPECT_TRUE(
      engine.updateLatestCommand(
        {{static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0}},
        base + std::chrono::microseconds(i * 100)));
  }

  capture_samples.store(true, std::memory_order_relaxed);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  const auto stats = engine.snapshot();
  engine.stop();

  EXPECT_EQ(stats.write_total, 80u);
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
}

}  // namespace
