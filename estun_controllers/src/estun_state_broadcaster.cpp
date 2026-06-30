// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include "estun_controllers/estun_state_broadcaster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "controller_interface/controller_interface_base.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"

namespace estun_controllers
{
namespace
{
constexpr size_t kConnectedIndex = 0;
constexpr size_t kRobotErrorIndex = 1;
constexpr size_t kDisconnectedIndex = 2;
constexpr size_t kFirstPacketReceivedIndex = 3;
constexpr size_t kStatusPacketCountIndex = 4;
constexpr size_t kActiveCommandModeIndex = 5;
constexpr size_t kConfiguredServoModeIndex = 6;
constexpr size_t kQueueDepthIndex = 7;
constexpr size_t kQueueMaxDepthIndex = 8;
constexpr size_t kQueuePushCountIndex = 9;
constexpr size_t kQueueTakeCountIndex = 10;
constexpr size_t kQueueFullDropCountIndex = 11;
constexpr size_t kQueueUnderflowCountIndex = 12;
constexpr size_t kRepeatedSendCountIndex = 13;
constexpr size_t kServoCallCountIndex = 14;
constexpr size_t kServoSdkFailCountIndex = 15;
constexpr size_t kServoBlockNsMaxIndex = 16;

uint64_t to_u64(double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return 0;
  }
  const auto max_value = static_cast<double>(std::numeric_limits<uint64_t>::max());
  return static_cast<uint64_t>(std::min(value, max_value));
}

uint32_t to_u32(double value)
{
  return static_cast<uint32_t>(
    std::min<uint64_t>(to_u64(value), std::numeric_limits<uint32_t>::max()));
}

uint8_t to_u8(double value)
{
  return static_cast<uint8_t>(
    std::min<uint64_t>(to_u64(value), std::numeric_limits<uint8_t>::max()));
}

bool to_bool(double value)
{
  return std::isfinite(value) && value != 0.0;
}
}  // namespace

controller_interface::CallbackReturn EstunStateBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("prefix", "");
    auto_declare<double>("state_publish_rate", 10.0);
    state_interface_names_ = state_interface_names();
  } catch (const std::exception & e) {
    fprintf(stderr, "EstunStateBroadcaster init failed: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  prefix_ = get_node()->get_parameter("prefix").as_string();
  status_interface_name_ = prefix_ + "estun_status";
  state_interface_names_ = state_interface_names();
  state_publish_rate_ = get_node()->get_parameter("state_publish_rate").as_double();
  if (state_publish_rate_ <= 0.0 || !std::isfinite(state_publish_rate_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "state_publish_rate must be a positive finite value.");
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_ = get_node()->create_publisher<estun_msgs::msg::EstunRobotStatus>(
    "~/status",
    rclcpp::SystemDefaultsQoS());
  publish_elapsed_ns_ = 0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  publish_elapsed_ns_ = 0;
  if (publisher_) {
    publisher_->on_activate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EstunStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
EstunStateBroadcaster::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::INDIVIDUAL,
    state_interface_names_};
}

controller_interface::return_type EstunStateBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (!publisher_) {
    return controller_interface::return_type::ERROR;
  }
  if (state_interfaces_.size() != state_interface_names_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      5000,
      "Estun status state interface count mismatch.");
    return controller_interface::return_type::ERROR;
  }

  publish_elapsed_ns_ += period.nanoseconds();
  const int64_t publish_period_ns = static_cast<int64_t>(1000000000.0 / state_publish_rate_);
  if (publish_elapsed_ns_ < publish_period_ns) {
    return controller_interface::return_type::OK;
  }
  publish_elapsed_ns_ = 0;

  publisher_->publish(build_status_msg(time));
  return controller_interface::return_type::OK;
}

std::vector<std::string> EstunStateBroadcaster::state_interface_names() const
{
  return {
    status_interface_name_ + "/connected",
    status_interface_name_ + "/robot_error",
    status_interface_name_ + "/disconnected",
    status_interface_name_ + "/first_packet_received",
    status_interface_name_ + "/status_packet_count",
    status_interface_name_ + "/active_command_mode",
    status_interface_name_ + "/configured_servo_mode",
    status_interface_name_ + "/queue_depth",
    status_interface_name_ + "/queue_max_depth",
    status_interface_name_ + "/queue_push_count",
    status_interface_name_ + "/queue_take_count",
    status_interface_name_ + "/queue_full_drop_count",
    status_interface_name_ + "/queue_underflow_count",
    status_interface_name_ + "/repeated_send_count",
    status_interface_name_ + "/servo_call_count",
    status_interface_name_ + "/servo_sdk_fail_count",
    status_interface_name_ + "/servo_block_ns_max",
  };
}

estun_msgs::msg::EstunRobotStatus EstunStateBroadcaster::build_status_msg(
  const rclcpp::Time & time) const
{
  estun_msgs::msg::EstunRobotStatus msg;
  const int64_t ns = time.nanoseconds();
  msg.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

  msg.connected = to_bool(state_interfaces_[kConnectedIndex].get_value());
  msg.robot_error = to_bool(state_interfaces_[kRobotErrorIndex].get_value());
  msg.disconnected = to_bool(state_interfaces_[kDisconnectedIndex].get_value());
  msg.first_packet_received = to_bool(state_interfaces_[kFirstPacketReceivedIndex].get_value());
  msg.status_packet_count = to_u64(state_interfaces_[kStatusPacketCountIndex].get_value());
  msg.active_command_mode = to_u8(state_interfaces_[kActiveCommandModeIndex].get_value());
  msg.configured_servo_mode = to_u8(state_interfaces_[kConfiguredServoModeIndex].get_value());
  msg.queue_depth = to_u32(state_interfaces_[kQueueDepthIndex].get_value());
  msg.queue_max_depth = to_u32(state_interfaces_[kQueueMaxDepthIndex].get_value());
  msg.queue_push_count = to_u64(state_interfaces_[kQueuePushCountIndex].get_value());
  msg.queue_take_count = to_u64(state_interfaces_[kQueueTakeCountIndex].get_value());
  msg.queue_full_drop_count = to_u64(state_interfaces_[kQueueFullDropCountIndex].get_value());
  msg.queue_underflow_count = to_u64(state_interfaces_[kQueueUnderflowCountIndex].get_value());
  msg.repeated_send_count = to_u64(state_interfaces_[kRepeatedSendCountIndex].get_value());
  msg.servo_call_count = to_u64(state_interfaces_[kServoCallCountIndex].get_value());
  msg.servo_sdk_fail_count = to_u64(state_interfaces_[kServoSdkFailCountIndex].get_value());
  msg.servo_block_ns_max = to_u64(state_interfaces_[kServoBlockNsMaxIndex].get_value());
  return msg;
}
}  // namespace estun_controllers

PLUGINLIB_EXPORT_CLASS(
  estun_controllers::EstunStateBroadcaster,
  controller_interface::ControllerInterface)
