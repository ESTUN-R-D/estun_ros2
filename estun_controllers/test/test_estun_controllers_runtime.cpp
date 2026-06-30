// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "estun_controllers/estun_do_controller.hpp"
#include "estun_controllers/estun_state_broadcaster.hpp"
#include "estun_msgs/msg/estun_robot_status.hpp"
#include "estun_msgs/srv/get_do.hpp"
#include "estun_msgs/srv/set_do.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using namespace std::chrono_literals;

constexpr auto kWaitTimeout = 2s;

class RosRuntimeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    thread_ = std::thread([this]() {executor_->spin();});
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
    executor_.reset();
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

private:
  std::thread thread_;
};

void wait_for_executor()
{
  std::this_thread::sleep_for(50ms);
}

bool wait_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

std::vector<hardware_interface::StateInterface> make_status_state_interfaces(
  std::vector<double> & values,
  const std::string & prefix = "")
{
  const std::string status_name = prefix + "estun_status";
  std::vector<std::string> names{
    "connected",
    "robot_error",
    "disconnected",
    "first_packet_received",
    "status_packet_count",
    "active_command_mode",
    "configured_servo_mode",
    "queue_depth",
    "queue_max_depth",
    "queue_push_count",
    "queue_take_count",
    "queue_full_drop_count",
    "queue_underflow_count",
    "repeated_send_count",
    "servo_call_count",
    "servo_sdk_fail_count",
    "servo_block_ns_max",
  };

  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    state_interfaces.emplace_back(status_name, names[i], &values[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::LoanedStateInterface> loan_state_interfaces(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  std::vector<hardware_interface::LoanedStateInterface> loaned_interfaces;
  loaned_interfaces.reserve(state_interfaces.size());
  for (auto & state_interface : state_interfaces) {
    loaned_interfaces.emplace_back(state_interface);
  }
  return loaned_interfaces;
}

TEST(EstunControllerPluginTest, LoadsExpectedControllerPlugins)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface",
    "controller_interface::ControllerInterface");

  {
    auto state_broadcaster = loader.createSharedInstance(
      "estun_controllers/EstunStateBroadcaster");
    ASSERT_NE(state_broadcaster, nullptr);
    EXPECT_EQ(
      state_broadcaster->init("plugin_state_broadcaster"),
      controller_interface::return_type::OK);
    EXPECT_EQ(
      state_broadcaster->command_interface_configuration().type,
      controller_interface::interface_configuration_type::NONE);
    state_broadcaster.reset();
  }

  {
    auto do_controller = loader.createSharedInstance("estun_controllers/EstunDOController");
    ASSERT_NE(do_controller, nullptr);
    EXPECT_EQ(do_controller->init("plugin_do_controller"), controller_interface::return_type::OK);
    EXPECT_EQ(
      do_controller->state_interface_configuration().type,
      controller_interface::interface_configuration_type::NONE);
    do_controller.reset();
  }
}

TEST_F(RosRuntimeTest, StateBroadcasterPublishesMappedStatusFields)
{
  auto controller = std::make_shared<estun_controllers::EstunStateBroadcaster>();
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(
    controller->init("runtime_state_broadcaster"),
    controller_interface::return_type::OK);
  controller->get_node()->set_parameter(rclcpp::Parameter("state_publish_rate", 100.0));
  executor_->add_node(controller->get_node()->get_node_base_interface());

  ASSERT_EQ(
    controller->on_configure(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  const auto state_config = controller->state_interface_configuration();
  ASSERT_EQ(state_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(state_config.names.size(), 17u);
  EXPECT_EQ(state_config.names.front(), "estun_status/connected");
  EXPECT_EQ(state_config.names.back(), "estun_status/servo_block_ns_max");

  std::vector<double> values{
    1.0, 0.0, 0.0, 1.0, 42.0, 1.0, 0.0, 3.0, 8.0,
    10.0, 7.0, 2.0, 4.0, 5.0,
    100.0, 1.0, 123456.0,
  };
  auto state_interfaces = make_status_state_interfaces(values);
  controller->assign_interfaces({}, loan_state_interfaces(state_interfaces));
  ASSERT_EQ(
    controller->on_activate(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);

  auto observer = std::make_shared<rclcpp::Node>("runtime_state_broadcaster_observer");
  estun_msgs::msg::EstunRobotStatus::SharedPtr last_msg;
  auto subscription = observer->create_subscription<estun_msgs::msg::EstunRobotStatus>(
    "/runtime_state_broadcaster/status",
    rclcpp::SystemDefaultsQoS(),
    [&last_msg](estun_msgs::msg::EstunRobotStatus::SharedPtr msg) {
      last_msg = std::move(msg);
    });
  (void)subscription;
  executor_->add_node(observer);
  wait_for_executor();

  EXPECT_EQ(
    controller->update(rclcpp::Time(1, 2), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_TRUE(wait_until([&last_msg]() {return last_msg != nullptr;}, kWaitTimeout));
  EXPECT_TRUE(last_msg->connected);
  EXPECT_FALSE(last_msg->robot_error);
  EXPECT_TRUE(last_msg->first_packet_received);
  EXPECT_EQ(last_msg->status_packet_count, 42u);
  EXPECT_EQ(last_msg->active_command_mode, estun_msgs::msg::EstunRobotStatus::COMMAND_MODE_CPOS);
  EXPECT_EQ(last_msg->queue_depth, 3u);
  EXPECT_EQ(last_msg->queue_max_depth, 8u);
  EXPECT_EQ(last_msg->queue_push_count, 10u);
  EXPECT_EQ(last_msg->queue_take_count, 7u);
  EXPECT_EQ(last_msg->queue_full_drop_count, 2u);
  EXPECT_EQ(last_msg->queue_underflow_count, 4u);
  EXPECT_EQ(last_msg->repeated_send_count, 5u);
  EXPECT_EQ(last_msg->servo_call_count, 100u);
  EXPECT_EQ(last_msg->servo_sdk_fail_count, 1u);
  EXPECT_EQ(last_msg->servo_block_ns_max, 123456u);

  EXPECT_EQ(
    controller->on_deactivate(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  controller->release_interfaces();
  executor_->remove_node(observer);
  executor_->remove_node(controller->get_node()->get_node_base_interface());
}

TEST_F(RosRuntimeTest, DoControllerRejectsRequestsUntilActivated)
{
  auto controller = std::make_shared<estun_controllers::EstunDOController>();
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(controller->init("inactive_do_controller"), controller_interface::return_type::OK);
  controller->get_node()->set_parameter(rclcpp::Parameter("service_timeout_ms", 50));
  executor_->add_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(
    controller->on_configure(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("inactive_do_controller_client");
  auto set_client = client_node->create_client<estun_msgs::srv::SetDo>(
    "/inactive_do_controller/set_do");
  executor_->add_node(client_node);
  ASSERT_TRUE(set_client->wait_for_service(kWaitTimeout));

  auto request = std::make_shared<estun_msgs::srv::SetDo::Request>();
  request->port = 18;
  request->value = true;
  auto future = set_client->async_send_request(request);
  ASSERT_EQ(future.wait_for(kWaitTimeout), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "controller is inactive");

  EXPECT_EQ(
    controller->on_cleanup(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  executor_->remove_node(client_node);
  executor_->remove_node(controller->get_node()->get_node_base_interface());
}

TEST_F(RosRuntimeTest, DoControllerStopsClientThreadOnDestructionWithoutCleanup)
{
  auto controller = std::make_shared<estun_controllers::EstunDOController>();
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(controller->init("destructed_do_controller"), controller_interface::return_type::OK);
  executor_->add_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(
    controller->on_configure(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);

  executor_->remove_node(controller->get_node()->get_node_base_interface());
  controller.reset();
  SUCCEED();
}

TEST_F(RosRuntimeTest, DoControllerEnforcesPortRulesBeforeSdkForwarding)
{
  auto controller = std::make_shared<estun_controllers::EstunDOController>();
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(controller->init("guarded_do_controller"), controller_interface::return_type::OK);
  controller->get_node()->set_parameter(rclcpp::Parameter("service_timeout_ms", 50));
  executor_->add_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(
    controller->on_configure(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller->on_activate(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("guarded_do_controller_client");
  auto set_client = client_node->create_client<estun_msgs::srv::SetDo>(
    "/guarded_do_controller/set_do");
  auto get_client = client_node->create_client<estun_msgs::srv::GetDo>(
    "/guarded_do_controller/get_do");
  executor_->add_node(client_node);
  ASSERT_TRUE(set_client->wait_for_service(kWaitTimeout));
  ASSERT_TRUE(get_client->wait_for_service(kWaitTimeout));

  auto invalid_get = std::make_shared<estun_msgs::srv::GetDo::Request>();
  invalid_get->port = 0;
  auto invalid_get_future = get_client->async_send_request(invalid_get);
  ASSERT_EQ(invalid_get_future.wait_for(kWaitTimeout), std::future_status::ready);
  EXPECT_FALSE(invalid_get_future.get()->success);

  auto readonly_set = std::make_shared<estun_msgs::srv::SetDo::Request>();
  readonly_set->port = 17;
  readonly_set->value = true;
  auto readonly_set_future = set_client->async_send_request(readonly_set);
  ASSERT_EQ(readonly_set_future.wait_for(kWaitTimeout), std::future_status::ready);
  const auto readonly_response = readonly_set_future.get();
  EXPECT_FALSE(readonly_response->success);
  EXPECT_EQ(readonly_response->message, "external write is not allowed for port 1..17");

  auto unavailable_set = std::make_shared<estun_msgs::srv::SetDo::Request>();
  unavailable_set->port = 18;
  unavailable_set->value = true;
  auto unavailable_set_future = set_client->async_send_request(unavailable_set);
  ASSERT_EQ(unavailable_set_future.wait_for(kWaitTimeout), std::future_status::ready);
  const auto unavailable_response = unavailable_set_future.get();
  EXPECT_FALSE(unavailable_response->success);
  EXPECT_EQ(unavailable_response->message, "SDK set_do service is not available");

  EXPECT_EQ(
    controller->on_cleanup(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  executor_->remove_node(client_node);
  executor_->remove_node(controller->get_node()->get_node_base_interface());
}

TEST_F(RosRuntimeTest, DoControllerForwardsRequestsToSdkServices)
{
  auto sdk_node = std::make_shared<rclcpp::Node>("fake_estun_sdk");
  std::atomic<uint16_t> set_port{0};
  std::atomic<bool> set_value{false};
  std::atomic<uint16_t> get_port{0};
  auto set_service = sdk_node->create_service<estun_msgs::srv::SetDo>(
    "/fake_estun/set_do",
    [&set_port, &set_value](
      const std::shared_ptr<estun_msgs::srv::SetDo::Request> request,
      std::shared_ptr<estun_msgs::srv::SetDo::Response> response) {
      set_port = request->port;
      set_value = request->value;
      response->success = true;
      response->message = "fake set ok";
    });
  auto get_service = sdk_node->create_service<estun_msgs::srv::GetDo>(
    "/fake_estun/get_do",
    [&get_port](
      const std::shared_ptr<estun_msgs::srv::GetDo::Request> request,
      std::shared_ptr<estun_msgs::srv::GetDo::Response> response) {
      get_port = request->port;
      response->success = true;
      response->value = true;
      response->message = "fake get ok";
    });
  (void)set_service;
  (void)get_service;
  executor_->add_node(sdk_node);

  auto controller = std::make_shared<estun_controllers::EstunDOController>();
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(controller->init("forwarding_do_controller"), controller_interface::return_type::OK);
  controller->get_node()->set_parameter(rclcpp::Parameter("sdk_namespace", "/fake_estun/"));
  controller->get_node()->set_parameter(rclcpp::Parameter("service_timeout_ms", 500));
  executor_->add_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(
    controller->on_configure(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller->on_activate(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("forwarding_do_controller_client");
  auto set_client = client_node->create_client<estun_msgs::srv::SetDo>(
    "/forwarding_do_controller/set_do");
  auto get_client = client_node->create_client<estun_msgs::srv::GetDo>(
    "/forwarding_do_controller/get_do");
  executor_->add_node(client_node);
  ASSERT_TRUE(set_client->wait_for_service(kWaitTimeout));
  ASSERT_TRUE(get_client->wait_for_service(kWaitTimeout));

  auto set_request = std::make_shared<estun_msgs::srv::SetDo::Request>();
  set_request->port = 18;
  set_request->value = true;
  auto set_future = set_client->async_send_request(set_request);
  ASSERT_EQ(set_future.wait_for(kWaitTimeout), std::future_status::ready);
  const auto set_response = set_future.get();
  EXPECT_TRUE(set_response->success);
  EXPECT_EQ(set_response->message, "fake set ok");
  EXPECT_EQ(set_port.load(), 18u);
  EXPECT_TRUE(set_value.load());

  auto get_request = std::make_shared<estun_msgs::srv::GetDo::Request>();
  get_request->port = 19;
  auto get_future = get_client->async_send_request(get_request);
  ASSERT_EQ(get_future.wait_for(kWaitTimeout), std::future_status::ready);
  const auto get_response = get_future.get();
  EXPECT_TRUE(get_response->success);
  EXPECT_TRUE(get_response->value);
  EXPECT_EQ(get_response->message, "fake get ok");
  EXPECT_EQ(get_port.load(), 19u);

  EXPECT_EQ(
    controller->on_cleanup(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  executor_->remove_node(client_node);
  executor_->remove_node(controller->get_node()->get_node_base_interface());
  executor_->remove_node(sdk_node);
}

TEST_F(RosRuntimeTest, DoControllerReportsSdkTimeout)
{
  auto sdk_node = std::make_shared<rclcpp::Node>("slow_estun_sdk");
  auto get_service = sdk_node->create_service<estun_msgs::srv::GetDo>(
    "/slow_estun/get_do",
    [](
      const std::shared_ptr<estun_msgs::srv::GetDo::Request>,
      std::shared_ptr<estun_msgs::srv::GetDo::Response> response) {
      std::this_thread::sleep_for(200ms);
      response->success = true;
      response->value = true;
      response->message = "late get ok";
    });
  (void)get_service;
  executor_->add_node(sdk_node);

  auto controller = std::make_shared<estun_controllers::EstunDOController>();
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(controller->init("timeout_do_controller"), controller_interface::return_type::OK);
  controller->get_node()->set_parameter(rclcpp::Parameter("sdk_namespace", "/slow_estun"));
  controller->get_node()->set_parameter(rclcpp::Parameter("service_timeout_ms", 25));
  executor_->add_node(controller->get_node()->get_node_base_interface());
  ASSERT_EQ(
    controller->on_configure(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller->on_activate(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);

  auto client_node = std::make_shared<rclcpp::Node>("timeout_do_controller_client");
  auto get_client = client_node->create_client<estun_msgs::srv::GetDo>(
    "/timeout_do_controller/get_do");
  executor_->add_node(client_node);
  ASSERT_TRUE(get_client->wait_for_service(kWaitTimeout));

  auto request = std::make_shared<estun_msgs::srv::GetDo::Request>();
  request->port = 18;
  auto future = get_client->async_send_request(request);
  ASSERT_EQ(future.wait_for(kWaitTimeout), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->value);
  EXPECT_EQ(response->message, "SDK get_do service timeout");

  EXPECT_EQ(
    controller->on_cleanup(
      controller->get_state()), controller_interface::CallbackReturn::SUCCESS);
  executor_->remove_node(client_node);
  executor_->remove_node(controller->get_node()->get_node_base_interface());
  executor_->remove_node(sdk_node);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
