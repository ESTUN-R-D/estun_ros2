// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#ifndef ESTUN_CONTROLLERS__ESTUN_STATE_BROADCASTER_HPP_
#define ESTUN_CONTROLLERS__ESTUN_STATE_BROADCASTER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "estun_msgs/msg/estun_robot_status.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace estun_controllers
{
class EstunStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<std::string> state_interface_names() const;
  estun_msgs::msg::EstunRobotStatus build_status_msg(const rclcpp::Time & time) const;

  std::string prefix_;
  std::string status_interface_name_{"estun_status"};
  std::vector<std::string> state_interface_names_;
  double state_publish_rate_{10.0};
  int64_t publish_elapsed_ns_{0};
  rclcpp_lifecycle::LifecyclePublisher<estun_msgs::msg::EstunRobotStatus>::SharedPtr publisher_;
};
}  // namespace estun_controllers

#endif  // ESTUN_CONTROLLERS__ESTUN_STATE_BROADCASTER_HPP_
