// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#ifndef ESTUN_CONTROLLERS__ESTUN_DO_CONTROLLER_HPP_
#define ESTUN_CONTROLLERS__ESTUN_DO_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "controller_interface/controller_interface.hpp"
#include "estun_msgs/srv/get_do.hpp"
#include "estun_msgs/srv/set_do.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace estun_controllers
{
class EstunDOController : public controller_interface::ControllerInterface
{
public:
  ~EstunDOController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void stop_client_executor();
  std::string resolve_sdk_namespace() const;
  void on_set_do(
    const std::shared_ptr<estun_msgs::srv::SetDo::Request> request,
    std::shared_ptr<estun_msgs::srv::SetDo::Response> response);
  void on_get_do(
    const std::shared_ptr<estun_msgs::srv::GetDo::Request> request,
    std::shared_ptr<estun_msgs::srv::GetDo::Response> response);

  std::string prefix_;
  std::string sdk_namespace_;
  int service_timeout_ms_{500};
  bool active_{false};

  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;
  std::thread client_thread_;
  rclcpp::Client<estun_msgs::srv::SetDo>::SharedPtr set_do_client_;
  rclcpp::Client<estun_msgs::srv::GetDo>::SharedPtr get_do_client_;
  rclcpp::Service<estun_msgs::srv::SetDo>::SharedPtr set_do_service_;
  rclcpp::Service<estun_msgs::srv::GetDo>::SharedPtr get_do_service_;
  std::mutex client_mutex_;
};
}  // namespace estun_controllers

#endif  // ESTUN_CONTROLLERS__ESTUN_DO_CONTROLLER_HPP_
