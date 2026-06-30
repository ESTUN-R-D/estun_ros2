// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#ifndef ESTUN_HARDWARE__ESTUN_SERVICE_BRIDGE_HPP_
#define ESTUN_HARDWARE__ESTUN_SERVICE_BRIDGE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "estun_msgs/srv/get_connection_status.hpp"
#include "estun_msgs/srv/get_cur_err_msg.hpp"
#include "estun_msgs/srv/get_do.hpp"
#include "estun_msgs/srv/get_joint_value.hpp"
#include "estun_msgs/srv/get_robot_conn_status.hpp"
#include "estun_msgs/srv/get_tool.hpp"
#include "estun_msgs/srv/get_user.hpp"
#include "estun_msgs/srv/get_world_cpos.hpp"
#include "estun_msgs/srv/set_do.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace estun_hardware
{

class EstunServiceBridge
{
public:
  using GetConnectionStatusHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Request>,
        std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Response>)>;
  using GetRobotConnStatusHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Request>,
        std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Response>)>;
  using GetCurErrMsgHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Request>,
        std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Response>)>;
  using GetWorldCposHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetWorldCpos::Request>,
        std::shared_ptr<estun_msgs::srv::GetWorldCpos::Response>)>;
  using GetJointValueHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetJointValue::Request>,
        std::shared_ptr<estun_msgs::srv::GetJointValue::Response>)>;
  using SetDoHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::SetDo::Request>,
        std::shared_ptr<estun_msgs::srv::SetDo::Response>)>;
  using GetDoHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetDo::Request>,
        std::shared_ptr<estun_msgs::srv::GetDo::Response>)>;
  using GetToolHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetTool::Request>,
        std::shared_ptr<estun_msgs::srv::GetTool::Response>)>;
  using GetUserHandler = std::function<void (
        const std::shared_ptr<estun_msgs::srv::GetUser::Request>,
        std::shared_ptr<estun_msgs::srv::GetUser::Response>)>;

  struct Handlers
  {
    GetConnectionStatusHandler get_connection_status;
    GetRobotConnStatusHandler get_robot_conn_status;
    GetCurErrMsgHandler get_cur_err_msg;
    GetWorldCposHandler get_world_cpos;
    GetJointValueHandler get_joint_value;
    SetDoHandler set_do;
    GetDoHandler get_do;
    GetToolHandler get_tool;
    GetUserHandler get_user;
  };

  ~EstunServiceBridge();

  bool start(
    const std::string & prefix,
    bool force_fail,
    const Handlers & handlers,
    const rclcpp::Logger & logger);
  void stop();
  bool running() const;
  int64_t now_nanoseconds() const;

private:
  rclcpp::Node::SharedPtr service_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> service_executor_;
  std::thread service_thread_;
  rclcpp::Service<estun_msgs::srv::GetConnectionStatus>::SharedPtr srv_get_connection_status_;
  rclcpp::Service<estun_msgs::srv::GetRobotConnStatus>::SharedPtr srv_get_robot_conn_status_;
  rclcpp::Service<estun_msgs::srv::GetCurErrMsg>::SharedPtr srv_get_cur_err_msg_;
  rclcpp::Service<estun_msgs::srv::GetWorldCpos>::SharedPtr srv_get_world_cpos_;
  rclcpp::Service<estun_msgs::srv::GetJointValue>::SharedPtr srv_get_joint_value_;
  rclcpp::Service<estun_msgs::srv::SetDo>::SharedPtr srv_set_do_;
  rclcpp::Service<estun_msgs::srv::GetDo>::SharedPtr srv_get_do_;
  rclcpp::Service<estun_msgs::srv::GetTool>::SharedPtr srv_get_tool_;
  rclcpp::Service<estun_msgs::srv::GetUser>::SharedPtr srv_get_user_;
};

}  // namespace estun_hardware

#endif  // ESTUN_HARDWARE__ESTUN_SERVICE_BRIDGE_HPP_
