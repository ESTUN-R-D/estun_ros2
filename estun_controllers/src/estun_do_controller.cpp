// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include "estun_controllers/estun_do_controller.hpp"

#include <future>

#include "controller_interface/controller_interface_base.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace estun_controllers
{
namespace
{
constexpr uint16_t kDoPortMin = 1;
constexpr uint16_t kDoPortMax = 256;
constexpr uint16_t kDoExternalReadonlyMax = 17;

std::string normalize_namespace(std::string value)
{
  if (value.empty()) {
    return value;
  }
  if (value.front() != '/') {
    value = "/" + value;
  }
  while (value.find("//") != std::string::npos) {
    value.replace(value.find("//"), 2, "/");
  }
  if (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}
}  // namespace

EstunDOController::~EstunDOController()
{
  active_ = false;
  set_do_service_.reset();
  get_do_service_.reset();
  stop_client_executor();
}

controller_interface::CallbackReturn EstunDOController::on_init()
{
  try {
    auto_declare<std::string>("prefix", "");
    auto_declare<std::string>("sdk_namespace", "");
    auto_declare<int>("service_timeout_ms", 500);
  } catch (const std::exception & e) {
    fprintf(stderr, "EstunDOController init failed: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunDOController::on_configure(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  set_do_service_.reset();
  get_do_service_.reset();
  stop_client_executor();

  prefix_ = get_node()->get_parameter("prefix").as_string();
  sdk_namespace_ = get_node()->get_parameter("sdk_namespace").as_string();
  service_timeout_ms_ = static_cast<int>(get_node()->get_parameter("service_timeout_ms").as_int());
  if (service_timeout_ms_ <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "service_timeout_ms must be positive.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::string base_ns = resolve_sdk_namespace();
  client_node_ = std::make_shared<rclcpp::Node>(std::string(get_node()->get_name()) + "_clients");
  set_do_client_ = client_node_->create_client<estun_msgs::srv::SetDo>(base_ns + "/set_do");
  get_do_client_ = client_node_->create_client<estun_msgs::srv::GetDo>(base_ns + "/get_do");
  client_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  client_executor_->add_node(client_node_);
  client_thread_ = std::thread([this]() {client_executor_->spin();});

  set_do_service_ = get_node()->create_service<estun_msgs::srv::SetDo>(
    "~/set_do",
    [this](
      const std::shared_ptr<estun_msgs::srv::SetDo::Request> request,
      std::shared_ptr<estun_msgs::srv::SetDo::Response> response) {
      on_set_do(request, response);
    });
  get_do_service_ = get_node()->create_service<estun_msgs::srv::GetDo>(
    "~/get_do",
    [this](
      const std::shared_ptr<estun_msgs::srv::GetDo::Request> request,
      std::shared_ptr<estun_msgs::srv::GetDo::Response> response) {
      on_get_do(request, response);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "EstunDOController configured against SDK namespace: %s",
    base_ns.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunDOController::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunDOController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunDOController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  set_do_service_.reset();
  get_do_service_.reset();
  stop_client_executor();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EstunDOController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  set_do_service_.reset();
  get_do_service_.reset();
  stop_client_executor();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EstunDOController::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
EstunDOController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type EstunDOController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

void EstunDOController::stop_client_executor()
{
  if (client_executor_) {
    client_executor_->cancel();
  }
  if (client_thread_.joinable()) {
    client_thread_.join();
  }
  set_do_client_.reset();
  get_do_client_.reset();
  client_executor_.reset();
  client_node_.reset();
}

std::string EstunDOController::resolve_sdk_namespace() const
{
  if (!sdk_namespace_.empty()) {
    return normalize_namespace(sdk_namespace_);
  }
  return normalize_namespace(prefix_.empty() ? "/estun" : "/" + prefix_ + "estun");
}

void EstunDOController::on_set_do(
  const std::shared_ptr<estun_msgs::srv::SetDo::Request> request,
  std::shared_ptr<estun_msgs::srv::SetDo::Response> response)
{
  if (!active_) {
    response->success = false;
    response->message = "controller is inactive";
    return;
  }
  if (request->port < kDoPortMin || request->port > kDoPortMax) {
    response->success = false;
    response->message = "invalid port, expected 1..256";
    return;
  }
  if (request->port <= kDoExternalReadonlyMax) {
    response->success = false;
    response->message = "external write is not allowed for port 1..17";
    return;
  }
  if (!set_do_client_ || !set_do_client_->wait_for_service(std::chrono::milliseconds(0))) {
    response->success = false;
    response->message = "SDK set_do service is not available";
    return;
  }

  auto sdk_request = std::make_shared<estun_msgs::srv::SetDo::Request>();
  sdk_request->port = request->port;
  sdk_request->value = request->value;

  std::lock_guard<std::mutex> lock(client_mutex_);
  auto future = set_do_client_->async_send_request(sdk_request);
  if (future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) !=
    std::future_status::ready)
  {
    response->success = false;
    response->message = "SDK set_do service timeout";
    return;
  }
  const auto sdk_response = future.get();
  response->success = sdk_response->success;
  response->message = sdk_response->message;
}

void EstunDOController::on_get_do(
  const std::shared_ptr<estun_msgs::srv::GetDo::Request> request,
  std::shared_ptr<estun_msgs::srv::GetDo::Response> response)
{
  if (!active_) {
    response->success = false;
    response->value = false;
    response->message = "controller is inactive";
    return;
  }
  if (request->port < kDoPortMin || request->port > kDoPortMax) {
    response->success = false;
    response->value = false;
    response->message = "invalid port, expected 1..256";
    return;
  }
  if (!get_do_client_ || !get_do_client_->wait_for_service(std::chrono::milliseconds(0))) {
    response->success = false;
    response->value = false;
    response->message = "SDK get_do service is not available";
    return;
  }

  auto sdk_request = std::make_shared<estun_msgs::srv::GetDo::Request>();
  sdk_request->port = request->port;

  std::lock_guard<std::mutex> lock(client_mutex_);
  auto future = get_do_client_->async_send_request(sdk_request);
  if (future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) !=
    std::future_status::ready)
  {
    response->success = false;
    response->value = false;
    response->message = "SDK get_do service timeout";
    return;
  }
  const auto sdk_response = future.get();
  response->success = sdk_response->success;
  response->value = sdk_response->value;
  response->message = sdk_response->message;
}
}  // namespace estun_controllers

PLUGINLIB_EXPORT_CLASS(
  estun_controllers::EstunDOController,
  controller_interface::ControllerInterface)
