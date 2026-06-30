#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "estun_hardware/estun_hardware_interface.hpp"

namespace estun_hardware
{
namespace
{
hardware_interface::HardwareInfo make_test_hardware_info(const std::string & prefix)
{
  hardware_interface::HardwareInfo info;
  info.name = prefix.empty() ? "estun_test_system" : prefix + "estun_test_system";
  info.type = "system";
  info.hardware_class_type = "estun_hardware/EstunHardwareInterface";
  info.hardware_parameters["prefix"] = prefix;
  info.hardware_parameters["robot_ip"] = "127.0.0.1";
  info.hardware_parameters["cmd_port"] = "10000";
  info.hardware_parameters["servo_port"] = "10001";
  info.hardware_parameters["status_port"] = "10002";
  info.hardware_parameters["servo_mode"] = "apos";
  info.hardware_parameters["stream_policy"] = "fifo";
  info.hardware_parameters["motion_period_ms"] = "4";
  info.hardware_parameters["stream_target_depth"] = "5";
  info.hardware_parameters["servo_tracker_lookahead_time"] = "0.03";
  info.hardware_parameters["servo_tracker_gain"] = "2000";
  info.hardware_parameters["robot_model"] = "ER20-1780-A6";
  info.hardware_parameters["servo_trace_enable"] = "false";

  for (int i = 0; i < 6; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = prefix + "joint_" + std::to_string(i + 1);

    hardware_interface::InterfaceInfo command_interface;
    command_interface.name = hardware_interface::HW_IF_POSITION;
    joint.command_interfaces.push_back(command_interface);

    hardware_interface::InterfaceInfo state_interface;
    state_interface.name = hardware_interface::HW_IF_POSITION;
    joint.state_interfaces.push_back(state_interface);

    info.joints.push_back(joint);
  }

  hardware_interface::ComponentInfo cartesian_gpio;
  cartesian_gpio.name = prefix + "cartesian_tcp";
  for (const char * interface_name : {"x", "y", "z", "a", "b", "c"}) {
    hardware_interface::InterfaceInfo command_interface;
    command_interface.name = interface_name;
    cartesian_gpio.command_interfaces.push_back(command_interface);

    hardware_interface::InterfaceInfo state_interface;
    state_interface.name = interface_name;
    cartesian_gpio.state_interfaces.push_back(state_interface);
  }
  info.gpios.push_back(cartesian_gpio);

  return info;
}
}  // namespace

class EstunHardwareInterfaceRuntimeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setenv("ESTUN_TEST_STUB_SDK", "1", 1);
    unsetenv("ESTUN_TEST_FORCE_DISCONNECT");
    unsetenv("ESTUN_TEST_FORCE_ALARM");
    unsetenv("ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL");
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    unsetenv("ESTUN_TEST_STUB_SDK");
    unsetenv("ESTUN_TEST_FORCE_DISCONNECT");
    unsetenv("ESTUN_TEST_FORCE_ALARM");
    unsetenv("ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL");
  }

  static bool shutdown_requested(const EstunHardwareInterface & robot)
  {
    return robot.shutdown_stop_state_.requested.load(std::memory_order_acquire);
  }

  static void request_shutdown(EstunHardwareInterface & robot)
  {
    robot.request_shutdown_stop();
  }

  static void mark_shutdown_resolved(EstunHardwareInterface & robot)
  {
    robot.mark_shutdown_stop_resolved();
  }

  static bool acquire_slot(EstunHardwareInterface & robot)
  {
    return robot.acquire_status_callback_slot();
  }

  static void release_slot(EstunHardwareInterface & robot)
  {
    robot.release_status_callback_slot();
  }

  static int callback_slot_index(const EstunHardwareInterface & robot)
  {
    return robot.callback_slot_index_;
  }

  static void set_stop_in_progress(EstunHardwareInterface & robot, bool value)
  {
    robot.stop_in_progress_.store(value, std::memory_order_relaxed);
  }

  static bool stop_completed(const EstunHardwareInterface & robot)
  {
    return robot.stop_completed_.load(std::memory_order_acquire);
  }

  static bool stop_in_progress(const EstunHardwareInterface & robot)
  {
    return robot.stop_in_progress_.load(std::memory_order_acquire);
  }

  static bool is_control_active(const EstunHardwareInterface & robot)
  {
    return robot.is_control_active_;
  }

  static void invoke_slot(size_t slot, RobotStatus status)
  {
    EstunHardwareInterface::invoke_status_callback_slot_for_test(slot, status);
  }

  static double latest_joint(const EstunHardwareInterface & robot, size_t index)
  {
    return robot.status_cache_.load_joint_value(index);
  }

  static double latest_world(const EstunHardwareInterface & robot, size_t index)
  {
    return robot.status_cache_.load_world_pose_value(index);
  }

  static bool robot_error(const EstunHardwareInterface & robot)
  {
    return robot.status_cache_.robot_error();
  }

  static const EstunHardwareInterface::Config & config(const EstunHardwareInterface & robot)
  {
    return robot.cfg_;
  }
};

TEST_F(EstunHardwareInterfaceRuntimeTest, TrackerJointLimitsConvertMoveItRadiansToSdkDegrees)
{
  EstunHardwareInterface robot;

  ASSERT_EQ(
    robot.on_init(make_test_hardware_info("")),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto & cfg = config(robot);
  EXPECT_NEAR(cfg.servo_tracker_axis_max_velocity[0], 185.0, 1e-9);
  EXPECT_NEAR(cfg.servo_tracker_axis_max_acceleration[0], 1000.0, 1e-9);
  EXPECT_NEAR(cfg.servo_tracker_axis_max_velocity[5], 705.0, 1e-9);
  EXPECT_NEAR(cfg.servo_tracker_axis_max_acceleration[5], 4600.0, 1e-9);
  EXPECT_NEAR(cfg.servo_tracker_max_velocity, 185.0, 1e-9);
  EXPECT_NEAR(cfg.servo_tracker_max_acceleration, 900.0, 1e-9);
}

TEST_F(EstunHardwareInterfaceRuntimeTest, BroadcastShutdownStopMarksAllInstancesRequested)
{
  EstunHardwareInterface robot_a;
  EstunHardwareInterface robot_b;

  ASSERT_EQ(robot_a.on_init(make_test_hardware_info("r1_")), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(robot_b.on_init(make_test_hardware_info("r2_")), hardware_interface::CallbackReturn::SUCCESS);

  request_estun_shutdown_stop();

  EXPECT_TRUE(shutdown_requested(robot_a));
  EXPECT_TRUE(shutdown_requested(robot_b));
}

TEST_F(EstunHardwareInterfaceRuntimeTest, WaitForShutdownStopCompletionWaitsForAllInstances)
{
  EstunHardwareInterface robot_a;
  EstunHardwareInterface robot_b;

  ASSERT_EQ(robot_a.on_init(make_test_hardware_info("r1_")), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(robot_b.on_init(make_test_hardware_info("r2_")), hardware_interface::CallbackReturn::SUCCESS);

  set_stop_in_progress(robot_b, true);
  request_shutdown(robot_a);
  request_shutdown(robot_b);
  mark_shutdown_resolved(robot_a);

  EXPECT_FALSE(wait_for_estun_shutdown_stop_completion(std::chrono::milliseconds(20)));

  set_stop_in_progress(robot_b, false);
  mark_shutdown_resolved(robot_b);
  EXPECT_TRUE(wait_for_estun_shutdown_stop_completion(std::chrono::milliseconds(20)));
}

TEST_F(EstunHardwareInterfaceRuntimeTest, CallbackSlotsRouteRobotStatusToMatchingInstances)
{
  EstunHardwareInterface robot_a;
  EstunHardwareInterface robot_b;

  ASSERT_EQ(robot_a.on_init(make_test_hardware_info("r1_")), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(robot_b.on_init(make_test_hardware_info("r2_")), hardware_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(acquire_slot(robot_a));
  ASSERT_TRUE(acquire_slot(robot_b));
  ASSERT_EQ(callback_slot_index(robot_a), 0);
  ASSERT_EQ(callback_slot_index(robot_b), 1);

  RobotStatus status_a{};
  status_a.isRobotError = false;
  status_a.isEndPoint = true;
  status_a.worldCpos[0] = 101.0;
  status_a.worldCpos[3] = 0.1;
  status_a.worldCpos[4] = 0.2;
  status_a.worldCpos[5] = 0.3;
  status_a.jointValue[0] = 11.0;

  RobotStatus status_b{};
  status_b.isRobotError = true;
  status_b.isEndPoint = false;
  status_b.worldCpos[0] = 202.0;
  status_b.worldCpos[3] = 0.4;
  status_b.worldCpos[4] = 0.5;
  status_b.worldCpos[5] = 0.6;
  status_b.jointValue[0] = 22.0;

  invoke_slot(0, status_a);
  invoke_slot(1, status_b);

  EXPECT_DOUBLE_EQ(latest_joint(robot_a, 0), 11.0);
  EXPECT_DOUBLE_EQ(latest_joint(robot_b, 0), 22.0);
  EXPECT_DOUBLE_EQ(latest_world(robot_a, 0), 101.0);
  EXPECT_DOUBLE_EQ(latest_world(robot_b, 0), 202.0);
  EXPECT_FALSE(robot_error(robot_a));
  EXPECT_TRUE(robot_error(robot_b));
}

TEST_F(EstunHardwareInterfaceRuntimeTest, ReleasedCallbackSlotStopsRoutingToOldInstance)
{
  EstunHardwareInterface robot;

  ASSERT_EQ(
    robot.on_init(make_test_hardware_info("r1_")),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(acquire_slot(robot));
  ASSERT_EQ(callback_slot_index(robot), 0);
  release_slot(robot);

  RobotStatus status{};
  status.jointValue[0] = 77.0;
  invoke_slot(0, status);

  EXPECT_DOUBLE_EQ(latest_joint(robot, 0), 0.0);
}

TEST_F(EstunHardwareInterfaceRuntimeTest, ActivateDeactivateRoundTripResetsStopStateForNextSession)
{
  EstunHardwareInterface robot;

  ASSERT_EQ(
    robot.on_init(make_test_hardware_info("r1_")),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    robot.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(
    robot.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(is_control_active(robot));
  EXPECT_FALSE(stop_completed(robot));
  EXPECT_FALSE(stop_in_progress(robot));

  ASSERT_EQ(
    robot.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(is_control_active(robot));
  EXPECT_TRUE(stop_completed(robot));
  EXPECT_FALSE(stop_in_progress(robot));

  ASSERT_EQ(
    robot.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(is_control_active(robot));
  EXPECT_FALSE(stop_completed(robot));
  EXPECT_FALSE(stop_in_progress(robot));
  EXPECT_FALSE(shutdown_requested(robot));

  ASSERT_EQ(
    robot.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(is_control_active(robot));
  EXPECT_TRUE(stop_completed(robot));
  EXPECT_FALSE(stop_in_progress(robot));
}

TEST_F(EstunHardwareInterfaceRuntimeTest, FifthInstanceCannotAcquireCallbackSlot)
{
  std::vector<std::unique_ptr<EstunHardwareInterface>> robots;
  robots.reserve(5);
  for (int i = 0; i < 5; ++i) {
    auto robot = std::make_unique<EstunHardwareInterface>();
    ASSERT_EQ(
      robot->on_init(make_test_hardware_info("r" + std::to_string(i + 1) + "_")),
      hardware_interface::CallbackReturn::SUCCESS);
    robots.push_back(std::move(robot));
  }

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(acquire_slot(*robots[i]));
  }
  EXPECT_FALSE(acquire_slot(*robots[4]));
}

}  // namespace estun_hardware
