// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include <gtest/gtest.h>

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "estun_hardware/estun_hardware_config.hpp"

namespace estun_hardware
{
namespace
{
std::unordered_map<std::string, std::string> make_required_params()
{
  return {
    {"robot_ip", "127.0.0.1"},
    {"cmd_port", "10000"},
    {"servo_port", "10001"},
    {"status_port", "10002"},
    {"robot_model", "ER20-1780-A6"},
  };
}
}  // namespace

TEST(EstunHardwareConfigTest, AppliesDefaultsForOptionalParams)
{
  unsetenv("ESTUN_TEST_STUB_SDK");
  unsetenv("ESTUN_TEST_FORCE_DISCONNECT");
  unsetenv("ESTUN_TEST_FORCE_ALARM");
  unsetenv("ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL");

  const auto config = parse_estun_hardware_config(make_required_params());

  EXPECT_EQ(config.prefix, "");
  EXPECT_EQ(config.robot_ip, "127.0.0.1");
  EXPECT_EQ(config.cmd_port, 10000);
  EXPECT_EQ(config.servo_port, 10001);
  EXPECT_EQ(config.status_port, 10002);
  EXPECT_EQ(config.servo_mode, "apos");
  EXPECT_EQ(config.stream_policy, EstunStreamPolicy::FIFO);
  EXPECT_EQ(config.motion_period_ms, 4);
  EXPECT_EQ(config.stream_target_depth, 5U);
  EXPECT_FALSE(config.servo_trace_enable);
  EXPECT_EQ(config.servo_trace_capacity, 900000U);
  EXPECT_TRUE(config.servo_trace_file.empty());
  EXPECT_FALSE(config.test_stub_sdk);
  EXPECT_FALSE(config.test_force_disconnect);
  EXPECT_FALSE(config.test_force_alarm);
  EXPECT_FALSE(config.test_force_service_bridge_fail);
}

TEST(EstunHardwareConfigTest, ParsesCanonicalServoModeAndTrimsStringParams)
{
  auto params = make_required_params();
  params["prefix"] = "r1_";
  params["servo_mode"] = " CPOS ";
  params["stream_policy"] = " latest-overwrite ";
  params["motion_period_ms"] = "8";
  params["stream_target_depth"] = "9";
  params["servo_tracker_lookahead_time"] = "0.05";
  params["servo_tracker_gain"] = "1800";
  params["servo_trace_enable"] = "YES";
  params["servo_trace_capacity"] = "123";
  params["servo_trace_file"] = " /tmp/trace.csv ";
  params["robot_model"] = " ER20-1780-A6 ";

  const auto config = parse_estun_hardware_config(params);

  EXPECT_EQ(config.prefix, "r1_");
  EXPECT_EQ(config.servo_mode, "cpos");
  EXPECT_EQ(config.stream_policy, EstunStreamPolicy::LATEST_OVERWRITE);
  EXPECT_EQ(config.motion_period_ms, 8);
  EXPECT_EQ(config.stream_target_depth, 9U);
  EXPECT_DOUBLE_EQ(config.servo_tracker_lookahead_time, 0.05);
  EXPECT_DOUBLE_EQ(config.servo_tracker_gain, 1800.0);
  EXPECT_TRUE(config.servo_trace_enable);
  EXPECT_EQ(config.servo_trace_capacity, 123U);
  EXPECT_EQ(config.servo_trace_file, "/tmp/trace.csv");
  EXPECT_EQ(config.robot_model, "ER20-1780-A6");
}

TEST(EstunHardwareConfigTest, RejectsInvalidServoMode)
{
  auto params = make_required_params();
  params["servo_mode"] = "cpso";

  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::invalid_argument);

  params["servo_mode"] = "cartesian";
  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::invalid_argument);
}

TEST(EstunHardwareConfigTest, RejectsInvalidStreamPolicy)
{
  auto params = make_required_params();
  params["stream_policy"] = "unknown";

  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::invalid_argument);
}

TEST(EstunHardwareConfigTest, RejectsOutOfRangeDepthAndPeriod)
{
  auto params = make_required_params();
  params["stream_target_depth"] = "101";
  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::out_of_range);

  params = make_required_params();
  params["motion_period_ms"] = "0";
  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::out_of_range);
}

TEST(EstunHardwareConfigTest, RejectsOutOfRangePorts)
{
  auto params = make_required_params();
  params["cmd_port"] = "70000";
  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::out_of_range);

  params = make_required_params();
  params["servo_port"] = "0";
  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::out_of_range);

  params = make_required_params();
  params["status_port"] = "-1";
  EXPECT_THROW(
    (void)parse_estun_hardware_config(params),
    std::out_of_range);
}

TEST(EstunHardwareConfigTest, ReadsTestFaultInjectionEnvironment)
{
  setenv("ESTUN_TEST_STUB_SDK", "true", 1);
  setenv("ESTUN_TEST_FORCE_DISCONNECT", "on", 1);
  setenv("ESTUN_TEST_FORCE_ALARM", "yes", 1);
  setenv("ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL", "1", 1);

  const auto config = parse_estun_hardware_config(make_required_params());

  EXPECT_TRUE(config.test_stub_sdk);
  EXPECT_TRUE(config.test_force_disconnect);
  EXPECT_TRUE(config.test_force_alarm);
  EXPECT_TRUE(config.test_force_service_bridge_fail);

  unsetenv("ESTUN_TEST_STUB_SDK");
  unsetenv("ESTUN_TEST_FORCE_DISCONNECT");
  unsetenv("ESTUN_TEST_FORCE_ALARM");
  unsetenv("ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL");
}

}  // namespace estun_hardware
