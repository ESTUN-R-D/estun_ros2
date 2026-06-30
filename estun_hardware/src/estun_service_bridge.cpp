// Copyright 2026 ESTUN AUTOMATION CO., LTD.

#include "estun_hardware/estun_service_bridge.hpp"

#include <dlfcn.h>
#include <cctype>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "estun_hardware/estun_hardware_interface.hpp"
#include "rclcpp/logging.hpp"

namespace estun_hardware
{
namespace
{
static constexpr uint16_t kDoPortMin = 1;
static constexpr uint16_t kDoPortMax = 256;
static constexpr uint16_t kDoExternalReadonlyMax = 17;

void * g_estun_msgs_fastrtps_typesupport_handle = nullptr;

std::string make_node_name(const std::string & prefix)
{
  std::string node_name = prefix.empty() ? "estun_sdk_service_bridge" :
    prefix + "estun_sdk_service_bridge";
  for (char & c : node_name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
      c = '_';
    }
  }
  return node_name;
}

std::string make_base_namespace(const std::string & prefix)
{
  std::string base_ns = prefix.empty() ? "/estun" : "/" + prefix + "estun";
  while (base_ns.find("//") != std::string::npos) {
    base_ns.replace(base_ns.find("//"), 2, "/");
  }
  if (!base_ns.empty() && base_ns.back() == '/') {
    base_ns.pop_back();
  }
  return base_ns;
}

void preload_estun_msgs_fastrtps_typesupport(const rclcpp::Logger & logger)
{
  // ros2_control_node 通常会设置 file capabilities，运行时可能忽略 LD_LIBRARY_PATH。
  // 这里预加载绝对路径，确保后续 create_service 能拿到 fastrtps typesupport。
  if (g_estun_msgs_fastrtps_typesupport_handle != nullptr) {
    return;
  }

  try {
    const auto pkg_prefix = ament_index_cpp::get_package_prefix("estun_msgs");
    const std::string ts_lib =
      pkg_prefix + "/lib/libestun_msgs__rosidl_typesupport_fastrtps_cpp.so";
    g_estun_msgs_fastrtps_typesupport_handle = dlopen(ts_lib.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (g_estun_msgs_fastrtps_typesupport_handle == nullptr) {
      const char * err = dlerror();
      RCLCPP_WARN(
        logger,
        "预加载 estun_msgs fastrtps typesupport 失败: %s",
        err ? err : "unknown dlopen error");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger,
      "查询 estun_msgs 安装路径失败: %s",
      e.what());
  }
}
}  // namespace

EstunServiceBridge::~EstunServiceBridge()
{
  stop();
}

bool EstunServiceBridge::start(
  const std::string & prefix,
  bool force_fail,
  const Handlers & handlers,
  const rclcpp::Logger & logger)
{
  if (service_node_) {
    return true;
  }
  if (force_fail) {
    RCLCPP_ERROR(
      logger,
      "测试注入：强制 service bridge 初始化失败（ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL=1）");
    return false;
  }
  if (!rclcpp::ok()) {
    RCLCPP_WARN(logger, "rclcpp 未初始化，跳过 SDK service bridge 启动");
    return false;
  }

  const std::string node_name = make_node_name(prefix);
  const std::string base_ns = make_base_namespace(prefix);
  auto service_name = [&base_ns](const std::string & endpoint) {
      return base_ns + "/" + endpoint;
    };

  preload_estun_msgs_fastrtps_typesupport(logger);

  try {
    service_node_ = std::make_shared<rclcpp::Node>(node_name);

    srv_get_connection_status_ =
      service_node_->create_service<estun_msgs::srv::GetConnectionStatus>(
      service_name("get_connection_status"),
      handlers.get_connection_status);
    srv_get_robot_conn_status_ =
      service_node_->create_service<estun_msgs::srv::GetRobotConnStatus>(
      service_name("get_robot_conn_status"),
      handlers.get_robot_conn_status);
    srv_get_cur_err_msg_ =
      service_node_->create_service<estun_msgs::srv::GetCurErrMsg>(
      service_name("get_cur_err_msg"),
      handlers.get_cur_err_msg);
    srv_get_world_cpos_ =
      service_node_->create_service<estun_msgs::srv::GetWorldCpos>(
      service_name("get_world_cpos"),
      handlers.get_world_cpos);
    srv_get_joint_value_ =
      service_node_->create_service<estun_msgs::srv::GetJointValue>(
      service_name("get_joint_value"),
      handlers.get_joint_value);
    srv_set_do_ =
      service_node_->create_service<estun_msgs::srv::SetDo>(
      service_name("set_do"),
      handlers.set_do);
    srv_get_do_ =
      service_node_->create_service<estun_msgs::srv::GetDo>(
      service_name("get_do"),
      handlers.get_do);
    srv_get_tool_ =
      service_node_->create_service<estun_msgs::srv::GetTool>(
      service_name("get_tool"),
      handlers.get_tool);
    srv_get_user_ =
      service_node_->create_service<estun_msgs::srv::GetUser>(
      service_name("get_user"),
      handlers.get_user);

    service_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    service_executor_->add_node(service_node_);
    service_thread_ = std::thread(
      [this]() {
        service_executor_->spin();
      });

    RCLCPP_INFO(
      logger,
      "SDK service bridge 已启动，命名空间: %s",
      base_ns.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger,
      "SDK service bridge 启动失败，将跳过 srv 挂载: %s",
      e.what());
    stop();
    return false;
  }
}

void EstunServiceBridge::stop()
{
  if (!service_node_) {
    return;
  }

  if (service_executor_) {
    service_executor_->cancel();
  }
  if (service_thread_.joinable()) {
    service_thread_.join();
  }
  if (service_executor_) {
    service_executor_->remove_node(service_node_);
  }

  srv_get_connection_status_.reset();
  srv_get_robot_conn_status_.reset();
  srv_get_cur_err_msg_.reset();
  srv_get_world_cpos_.reset();
  srv_get_joint_value_.reset();
  srv_set_do_.reset();
  srv_get_do_.reset();
  srv_get_tool_.reset();
  srv_get_user_.reset();

  service_executor_.reset();
  service_node_.reset();
}

bool EstunServiceBridge::running() const
{
  return service_node_ != nullptr;
}

int64_t EstunServiceBridge::now_nanoseconds() const
{
  if (!service_node_) {
    return 0;
  }
  return service_node_->get_clock()->now().nanoseconds();
}

bool EstunHardwareInterface::start_service_bridge()
{
  EstunServiceBridge::Handlers handlers;
  handlers.get_connection_status =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Request> request,
    std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Response> response) {
      on_get_connection_status(request, response);
    };
  handlers.get_robot_conn_status =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Request> request,
    std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Response> response) {
      on_get_robot_conn_status(request, response);
    };
  handlers.get_cur_err_msg =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Request> request,
    std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Response> response) {
      on_get_cur_err_msg(request, response);
    };
  handlers.get_world_cpos =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetWorldCpos::Request> request,
    std::shared_ptr<estun_msgs::srv::GetWorldCpos::Response> response) {
      on_get_world_cpos(request, response);
    };
  handlers.get_joint_value =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetJointValue::Request> request,
    std::shared_ptr<estun_msgs::srv::GetJointValue::Response> response) {
      on_get_joint_value(request, response);
    };
  handlers.set_do =
    [this](
    const std::shared_ptr<estun_msgs::srv::SetDo::Request> request,
    std::shared_ptr<estun_msgs::srv::SetDo::Response> response) {
      on_set_do(request, response);
    };
  handlers.get_do =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetDo::Request> request,
    std::shared_ptr<estun_msgs::srv::GetDo::Response> response) {
      on_get_do(request, response);
    };
  handlers.get_tool =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetTool::Request> request,
    std::shared_ptr<estun_msgs::srv::GetTool::Response> response) {
      on_get_tool(request, response);
    };
  handlers.get_user =
    [this](
    const std::shared_ptr<estun_msgs::srv::GetUser::Request> request,
    std::shared_ptr<estun_msgs::srv::GetUser::Response> response) {
      on_get_user(request, response);
    };

  return service_bridge_.start(
    cfg_.prefix,
    cfg_.test_force_service_bridge_fail,
    handlers,
    rclcpp::get_logger("EstunHardwareInterface"));
}

void EstunHardwareInterface::stop_service_bridge()
{
  service_bridge_.stop();
}

// 当前 service bridge 中走 SDK 的服务调用，已和 SDK 开发确认可与实时 servo 并发；
// 这里的锁只负责避免多个 service 同时重入 service 类 SDK 调用。
// 若未来新增的 SDK 接口会影响控制权、servo 状态机或实时链路状态，则必须改走 sdk_mutex_
// 或重新评估并发合同，不能默认沿用 service 锁。
std::unique_lock<std::mutex> EstunHardwareInterface::acquire_service_sdk_lock()
{
  return std::unique_lock<std::mutex>(sdk_service_mutex_);
}

void EstunHardwareInterface::on_get_connection_status(
  const std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Request> request,
  std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Response> response)
{
  (void)request;
  char cmd = 0;
  char servo = 0;
  char udp = 0;
  if (cfg_.test_stub_sdk) {
    cmd = 1;
    servo = 1;
    udp = 1;
  } else {
    auto sdk_lock = acquire_service_sdk_lock();
    eri_manager_.getRobotConnStatus(cmd, servo, udp);
  }

  const int64_t ns = service_bridge_.now_nanoseconds();
  response->status.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  response->status.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  response->status.connected =
    (cmd != 0 && servo != 0 && udp != 0 &&
    !status_cache_.is_disconnected());
  response->status.robot_ip = cfg_.robot_ip;
  std::ostringstream oss;
  oss << "cmd=" << static_cast<int>(cmd)
      << ", servo=" << static_cast<int>(servo)
      << ", udp=" << static_cast<int>(udp);
  response->status.detail = oss.str();
}

void EstunHardwareInterface::on_get_robot_conn_status(
  const std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Request> request,
  std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Response> response)
{
  (void)request;
  char cmd = 0;
  char servo = 0;
  char udp = 0;
  if (cfg_.test_stub_sdk) {
    cmd = 1;
    servo = 1;
    udp = 1;
  } else {
    auto sdk_lock = acquire_service_sdk_lock();
    eri_manager_.getRobotConnStatus(cmd, servo, udp);
  }
  response->cmd_status = static_cast<uint8_t>(static_cast<unsigned char>(cmd));
  response->servo_status = static_cast<uint8_t>(static_cast<unsigned char>(servo));
  response->udp_status = static_cast<uint8_t>(static_cast<unsigned char>(udp));
}

void EstunHardwareInterface::on_get_cur_err_msg(
  const std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Request> request,
  std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Response> response)
{
  (void)request;
  if (cfg_.test_stub_sdk) {
    response->success = true;
    response->err_id = cfg_.test_force_alarm ? 1 : 0;
    response->err_msg = cfg_.test_force_alarm ? "test stub alarm injected" : "";
    return;
  }

  int err_id = 0;
  char err_msg[1024];
  std::memset(err_msg, 0, sizeof(err_msg));

  {
    auto sdk_lock = acquire_service_sdk_lock();
    response->success = eri_manager_.getCurErrMsg(err_id, err_msg);
  }
  response->err_id = err_id;
  response->err_msg = response->success ? std::string(err_msg) : std::string("");
}

void EstunHardwareInterface::on_get_world_cpos(
  const std::shared_ptr<estun_msgs::srv::GetWorldCpos::Request> request,
  std::shared_ptr<estun_msgs::srv::GetWorldCpos::Response> response)
{
  (void)request;
  const auto pos = status_cache_.load_world_pose();
  response->success =
    status_cache_.first_packet_received() &&
    !status_cache_.is_disconnected();
  for (size_t i = 0; i < 16; ++i) {
    response->pos[i] = pos[i];
  }
}

void EstunHardwareInterface::on_get_joint_value(
  const std::shared_ptr<estun_msgs::srv::GetJointValue::Request> request,
  std::shared_ptr<estun_msgs::srv::GetJointValue::Response> response)
{
  (void)request;
  const auto pos = status_cache_.load_joint_values();
  response->success =
    status_cache_.first_packet_received() &&
    !status_cache_.is_disconnected();
  for (size_t i = 0; i < 16; ++i) {
    response->pos[i] = pos[i];
  }
}

void EstunHardwareInterface::on_set_do(
  const std::shared_ptr<estun_msgs::srv::SetDo::Request> request,
  std::shared_ptr<estun_msgs::srv::SetDo::Response> response)
{
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

  if (cfg_.test_stub_sdk) {
    test_stub_do_values_[request->port] = request->value;
    response->success = true;
    response->message = "ok";
    return;
  }

  {
    auto sdk_lock = acquire_service_sdk_lock();
    response->success = eri_manager_.setDo(
      static_cast<UINT32>(request->port),
      request->value ? 1U : 0U);
  }
  response->message = response->success ? "ok" : "setDo failed";
}

void EstunHardwareInterface::on_get_do(
  const std::shared_ptr<estun_msgs::srv::GetDo::Request> request,
  std::shared_ptr<estun_msgs::srv::GetDo::Response> response)
{
  if (request->port < kDoPortMin || request->port > kDoPortMax) {
    response->success = false;
    response->value = false;
    response->message = "invalid port, expected 1..256";
    return;
  }

  if (cfg_.test_stub_sdk) {
    response->success = true;
    response->value = test_stub_do_values_[request->port];
    response->message = "ok";
    return;
  }

  UINT32 value = 0;
  {
    auto sdk_lock = acquire_service_sdk_lock();
    value = eri_manager_.getCurrentDO(static_cast<UINT32>(request->port));
  }
  response->success = true;
  response->value = (value != 0U);
  response->message = "ok";
}

void EstunHardwareInterface::on_get_tool(
  const std::shared_ptr<estun_msgs::srv::GetTool::Request> request,
  std::shared_ptr<estun_msgs::srv::GetTool::Response> response)
{
  double tool_data[6]{0.0};
  if (request->tool_id < 0) {
    response->success = false;
    return;
  }
  if (cfg_.test_stub_sdk) {
    response->success = true;
    for (size_t i = 0; i < 6; ++i) {
      response->tool_data[i] =
        static_cast<double>(request->tool_id * 10 + static_cast<int>(i));
    }
    return;
  }
  {
    auto sdk_lock = acquire_service_sdk_lock();
    response->success = eri_manager_.getTool(static_cast<int>(request->tool_id), tool_data);
  }
  for (size_t i = 0; i < 6; ++i) {
    response->tool_data[i] = tool_data[i];
  }
}

void EstunHardwareInterface::on_get_user(
  const std::shared_ptr<estun_msgs::srv::GetUser::Request> request,
  std::shared_ptr<estun_msgs::srv::GetUser::Response> response)
{
  double user_data[6]{0.0};
  if (request->user_id < 0) {
    response->success = false;
    return;
  }
  if (cfg_.test_stub_sdk) {
    response->success = true;
    for (size_t i = 0; i < 6; ++i) {
      response->user_data[i] =
        static_cast<double>(request->user_id * 100 + static_cast<int>(i));
    }
    return;
  }
  {
    auto sdk_lock = acquire_service_sdk_lock();
    response->success = eri_manager_.getUser(static_cast<int>(request->user_id), user_data);
  }
  for (size_t i = 0; i < 6; ++i) {
    response->user_data[i] = user_data[i];
  }
}

}  // namespace estun_hardware
