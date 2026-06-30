#include "estun_hardware/estun_hardware_interface.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <array>
#include <filesystem>
#include <cctype>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace estun_hardware
{
namespace
{
constexpr size_t kMaxEstunCallbackSlots = 4;

struct CallbackSlotEntry
{
  EstunHardwareInterface * instance{nullptr};
  bool occupied{false};
};

std::mutex g_estun_runtime_registry_mutex;
std::array<CallbackSlotEntry, kMaxEstunCallbackSlots> g_status_callback_slots{};
std::vector<EstunHardwareInterface *> g_registered_instances;
}  // namespace

static constexpr int kQueueUnderflowWarnThrottleMs = 500;
static constexpr size_t kStopSoftHoldCycles = 8;
static constexpr int64_t kEndpointReleaseReadyTimeoutMs = 3000;
static constexpr size_t kInternalStreamLowWatermark = 3;
static constexpr double kInternalStreamSmoothingDepthFilterAlpha = 0.02;
static constexpr double kInternalStreamSmoothingDepthGain = 0.01;
static constexpr double kInternalStreamSmoothingMinPhaseStep = 0.95;
static constexpr double kInternalStreamSmoothingMaxPhaseStep = 1.05;
static constexpr std::array<const char *, 17> kEstunStatusStateInterfaceNames{{
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
  }};

static inline int64_t steady_now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

static inline double rad_to_deg(double value)
{
    return value * 180.0 / M_PI;
}

static inline void cartesian_ros_to_sdk(const std::array<double, 6> & ros_cmd, double sdk_cmd[6])
{
    // ROS 侧统一语义: [x,y,z,a,b,c] = [mm, mm, mm, rad, rad, rad]
    // SDK CPOS 入参语义: [x,y,z,a,b,c] = [mm, mm, mm, deg, deg, deg]
    sdk_cmd[0] = ros_cmd[0];
    sdk_cmd[1] = ros_cmd[1];
    sdk_cmd[2] = ros_cmd[2];
    sdk_cmd[3] = ros_cmd[3] * 180.0 / M_PI;
    sdk_cmd[4] = ros_cmd[4] * 180.0 / M_PI;
    sdk_cmd[5] = ros_cmd[5] * 180.0 / M_PI;
}

void request_estun_shutdown_stop()
{
  std::lock_guard<std::mutex> lock(g_estun_runtime_registry_mutex);
  for (EstunHardwareInterface * instance : g_registered_instances) {
    if (instance != nullptr) {
      instance->request_shutdown_stop();
    }
  }
}

bool wait_for_estun_shutdown_stop_completion(std::chrono::milliseconds timeout)
{
  std::vector<EstunHardwareInterface *> instances;
  {
    std::lock_guard<std::mutex> lock(g_estun_runtime_registry_mutex);
    instances = g_registered_instances;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    bool all_resolved = true;
    for (EstunHardwareInterface * instance : instances) {
      if (instance != nullptr && !instance->is_shutdown_stop_resolved()) {
        all_resolved = false;
        break;
      }
    }
    if (all_resolved) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  for (EstunHardwareInterface * instance : instances) {
    if (instance != nullptr && !instance->is_shutdown_stop_resolved()) {
      return false;
    }
  }
  return true;
}

EstunHardwareInterface::~EstunHardwareInterface()
{
  stop_service_bridge();
  release_status_callback_slot();
  unregister_instance();
}

void EstunHardwareInterface::register_instance()
{
  std::lock_guard<std::mutex> lock(g_estun_runtime_registry_mutex);
  if (instance_registered_) {
    return;
  }
  g_registered_instances.push_back(this);
  instance_registered_ = true;
}

void EstunHardwareInterface::unregister_instance()
{
  std::lock_guard<std::mutex> lock(g_estun_runtime_registry_mutex);
  if (!instance_registered_) {
    return;
  }
  g_registered_instances.erase(
    std::remove(g_registered_instances.begin(), g_registered_instances.end(), this),
    g_registered_instances.end());
  instance_registered_ = false;
}

bool EstunHardwareInterface::acquire_status_callback_slot()
{
  if (callback_slot_index_ >= 0) {
    return true;
  }
  std::lock_guard<std::mutex> lock(g_estun_runtime_registry_mutex);
  for (size_t i = 0; i < g_status_callback_slots.size(); ++i) {
    if (!g_status_callback_slots[i].occupied) {
      g_status_callback_slots[i].instance = this;
      g_status_callback_slots[i].occupied = true;
      callback_slot_index_ = static_cast<int>(i);
      return true;
    }
  }
  return false;
}

void EstunHardwareInterface::release_status_callback_slot()
{
  if (callback_slot_index_ < 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_estun_runtime_registry_mutex);
  CallbackSlotEntry & slot = g_status_callback_slots[static_cast<size_t>(callback_slot_index_)];
  slot.instance = nullptr;
  slot.occupied = false;
  callback_slot_index_ = -1;
}

void EstunHardwareInterface::dispatch_status_callback_slot(size_t slot, const RobotStatus & status)
{
  EstunHardwareInterface * instance = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_estun_runtime_registry_mutex);
    if (slot >= g_status_callback_slots.size()) {
      return;
    }
    if (!g_status_callback_slots[slot].occupied) {
      return;
    }
    instance = g_status_callback_slots[slot].instance;
  }
  if (instance != nullptr) {
    instance->handle_robot_status(status);
  }
}

void EstunHardwareInterface::status_callback_slot_0(RobotStatus status)
{
  dispatch_status_callback_slot(0, status);
}

void EstunHardwareInterface::status_callback_slot_1(RobotStatus status)
{
  dispatch_status_callback_slot(1, status);
}

void EstunHardwareInterface::status_callback_slot_2(RobotStatus status)
{
  dispatch_status_callback_slot(2, status);
}

void EstunHardwareInterface::status_callback_slot_3(RobotStatus status)
{
  dispatch_status_callback_slot(3, status);
}

void EstunHardwareInterface::invoke_status_callback_slot_for_test(size_t slot, RobotStatus status)
{
  dispatch_status_callback_slot(slot, status);
}

void EstunHardwareInterface::handle_robot_status(const RobotStatus & status)
{
  status_cache_.update_from_robot_status(status, steady_now_ms());
}

void EstunHardwareInterface::request_shutdown_stop()
{
  shutdown_stop_state_.requested.store(true, std::memory_order_release);
  const bool already_resolved =
    !is_control_active_ && !stop_in_progress_.load(std::memory_order_acquire);
  shutdown_stop_state_.completed.store(already_resolved, std::memory_order_release);
}

bool EstunHardwareInterface::consume_shutdown_stop_request()
{
  return shutdown_stop_state_.requested.exchange(false, std::memory_order_acq_rel);
}

void EstunHardwareInterface::reset_shutdown_stop_state()
{
  shutdown_stop_state_.requested.store(false, std::memory_order_relaxed);
  shutdown_stop_state_.completed.store(false, std::memory_order_relaxed);
}

void EstunHardwareInterface::mark_shutdown_stop_resolved()
{
  shutdown_stop_state_.requested.store(false, std::memory_order_release);
  shutdown_stop_state_.completed.store(true, std::memory_order_release);
}

bool EstunHardwareInterface::is_shutdown_stop_resolved() const
{
  return shutdown_stop_state_.completed.load(std::memory_order_acquire);
}

void EstunHardwareInterface::reset_status_cache()
{
  status_cache_.reset();
}

static inline double parse_required_finite_yaml_double(
  const YAML::Node & node,
  const std::string & field_name,
  const std::string & context_name)
{
  if (!node[field_name]) {
    throw std::runtime_error(
      "配置缺少字段 " + context_name + "." + field_name);
  }
  const double value = node[field_name].as<double>();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(
      "配置字段非法 " + context_name + "." + field_name);
  }
  return value;
}

static inline double parse_optional_finite_yaml_double(
  const YAML::Node & node,
  const std::string & field_name,
  double default_value,
  const std::string & context_name)
{
  if (!node[field_name]) {
    return default_value;
  }
  const double value = node[field_name].as<double>();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(
      "配置字段非法 " + context_name + "." + field_name);
  }
  return value;
}

hardware_interface::CallbackReturn EstunHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    cfg_ = parse_estun_hardware_config(info_.hardware_parameters);
    estun_status_interface_name_ = cfg_.prefix + "estun_status";
    load_tracker_joint_limits_from_description();
    load_tracker_cartesian_limits_from_description();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("EstunHardwareInterface"), "硬件参数缺失或解析错误: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  cartesian_interface_name_ = cfg_.prefix + "cartesian_tcp";

  std::string mode_lower = cfg_.servo_mode;
  std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (mode_lower == "cpos" || mode_lower == "cartesian") {
    configured_servo_mode_ = CommandMode::CARTESIAN_POSE;
    cfg_.servo_mode = "cpos";
  } else {
    configured_servo_mode_ = CommandMode::JOINT_POSITION;
    cfg_.servo_mode = "apos";
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "ERI 运动周期: %u ms (理论控制频率约 %.3f Hz)",
    cfg_.motion_period_ms,
    1000.0 / static_cast<double>(cfg_.motion_period_ms));
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "空队列策略: hold（空队列时每周期发送保持帧）");
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "流引擎配置: target=%zu low=%zu max_depth=%zu",
    cfg_.stream_target_depth,
    kInternalStreamLowWatermark,
    kEstunStreamMaxQueueDepth);
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "stream smoothing: forced enabled (internal alpha=%.3f gain=%.4f step=[%.3f, %.3f])",
    kInternalStreamSmoothingDepthFilterAlpha,
    kInternalStreamSmoothingDepthGain,
    kInternalStreamSmoothingMinPhaseStep,
    kInternalStreamSmoothingMaxPhaseStep);
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "流引擎节拍模式: callback_paced(固定)");
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "命令链路策略: %s",
    cfg_.stream_policy == StreamPolicy::LATEST_OVERWRITE ? "latest_overwrite" : "fifo");
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "latest servo tracker: forced enabled (lookahead=%.3f gain=%.1f vmax=%.3f amax=%.3f)",
    cfg_.servo_tracker_lookahead_time,
    cfg_.servo_tracker_gain,
    cfg_.servo_tracker_max_velocity,
    cfg_.servo_tracker_max_acceleration);
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "latest servo tracker joint limits: vmax=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] amax=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    cfg_.servo_tracker_axis_max_velocity[0],
    cfg_.servo_tracker_axis_max_velocity[1],
    cfg_.servo_tracker_axis_max_velocity[2],
    cfg_.servo_tracker_axis_max_velocity[3],
    cfg_.servo_tracker_axis_max_velocity[4],
    cfg_.servo_tracker_axis_max_velocity[5],
    cfg_.servo_tracker_axis_max_acceleration[0],
    cfg_.servo_tracker_axis_max_acceleration[1],
    cfg_.servo_tracker_axis_max_acceleration[2],
    cfg_.servo_tracker_axis_max_acceleration[3],
    cfg_.servo_tracker_axis_max_acceleration[4],
    cfg_.servo_tracker_axis_max_acceleration[5]);
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "cpos tracker cartesian limits: trans_v=%.3f mm/s trans_a=%.3f mm/s^2 rot_v=%.3f rad/s rot_a=%.3f rad/s^2",
    cfg_.cartesian_max_trans_velocity,
    cfg_.cartesian_max_trans_acceleration,
    cfg_.cartesian_max_rot_velocity,
    cfg_.cartesian_max_rot_acceleration);
  std::string servo_trace_file_desc = cfg_.servo_trace_file;
  if (servo_trace_file_desc.empty()) {
    servo_trace_file_desc = "<auto:" + EstunServoTrace::default_trace_dir() + ">";
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "servo trace: %s (capacity=%zu file=%s)",
    cfg_.servo_trace_enable ? "enabled" : "disabled",
    cfg_.servo_trace_capacity,
    servo_trace_file_desc.c_str());
  if (cfg_.test_stub_sdk) {
    RCLCPP_WARN(
      rclcpp::get_logger("EstunHardwareInterface"),
      "检测到 ESTUN_TEST_STUB_SDK=1，启用测试桩模式（仅用于自动化测试）。");
  }
  if (cfg_.test_force_disconnect || cfg_.test_force_alarm || cfg_.test_force_service_bridge_fail) {
    RCLCPP_WARN(
      rclcpp::get_logger("EstunHardwareInterface"),
      "检测到测试故障注入开关: disconnect=%d alarm=%d service_bridge_fail=%d",
      cfg_.test_force_disconnect ? 1 : 0,
      cfg_.test_force_alarm ? 1 : 0,
      cfg_.test_force_service_bridge_fail ? 1 : 0);
  }

  is_configured_ = false;
  is_control_active_ = false;
  stop_in_progress_.store(false, std::memory_order_relaxed);
  stop_completed_.store(false, std::memory_order_relaxed);
  reset_shutdown_stop_state();
  reset_status_cache();
  register_instance();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool EstunHardwareInterface::is_joint_position_interface(const std::string & interface_name) const
{
  for (const auto & joint : info_.joints) {
    if (interface_name == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
      return true;
    }
  }
  return false;
}

bool EstunHardwareInterface::is_cartesian_pose_interface(const std::string & interface_name) const
{
  const std::array<std::string, 6> kCartesianInterfaces{
    cartesian_interface_name_ + "/x", cartesian_interface_name_ + "/y", cartesian_interface_name_ + "/z",
    cartesian_interface_name_ + "/a", cartesian_interface_name_ + "/b", cartesian_interface_name_ + "/c"
  };
  return std::find(kCartesianInterfaces.begin(), kCartesianInterfaces.end(), interface_name) !=
         kCartesianInterfaces.end();
}

int EstunHardwareInterface::to_servo_motion_type(CommandMode mode) const
{
  return mode == CommandMode::JOINT_POSITION ? 0 : 1;
}

estun_libs::EstunServoStreamMode EstunHardwareInterface::to_stream_command_mode(CommandMode mode) const
{
  return mode == CommandMode::CARTESIAN_POSE
         ? estun_libs::EstunServoStreamMode::CPOS
         : estun_libs::EstunServoStreamMode::APOS;
}

void EstunHardwareInterface::start_stream_engine(const ServoCommandPacket & hold_packet)
{
  estun_libs::EstunServoStreamConfig stream_cfg;
  stream_cfg.period = std::chrono::milliseconds(cfg_.motion_period_ms);
  stream_cfg.target_depth = cfg_.stream_target_depth;
  stream_cfg.command_mode = to_stream_command_mode(active_command_mode_);
  stream_cfg.stream_policy =
    (cfg_.stream_policy == StreamPolicy::LATEST_OVERWRITE)
      ? estun_libs::EstunServoStreamPolicy::LATEST_OVERWRITE
      : estun_libs::EstunServoStreamPolicy::FIFO;
  stream_cfg.callback_paced = true;
  stream_cfg.servo_tracker_lookahead_time = cfg_.servo_tracker_lookahead_time;
  stream_cfg.servo_tracker_gain = cfg_.servo_tracker_gain;
  stream_cfg.servo_tracker_max_velocity = cfg_.servo_tracker_max_velocity;
  stream_cfg.servo_tracker_max_acceleration = cfg_.servo_tracker_max_acceleration;
  stream_cfg.servo_tracker_axis_max_velocity = cfg_.servo_tracker_axis_max_velocity;
  stream_cfg.servo_tracker_axis_max_acceleration = cfg_.servo_tracker_axis_max_acceleration;
  stream_cfg.cartesian_max_trans_velocity = cfg_.cartesian_max_trans_velocity;
  stream_cfg.cartesian_max_trans_acceleration = cfg_.cartesian_max_trans_acceleration;
  stream_cfg.cartesian_max_rot_velocity = cfg_.cartesian_max_rot_velocity;
  stream_cfg.cartesian_max_rot_acceleration = cfg_.cartesian_max_rot_acceleration;

  auto send_cb = [this](const std::array<double, 6> & values, const estun_libs::EstunServoSendMeta & meta) {
    if (cfg_.test_stub_sdk) {
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.motion_period_ms));
      return;
    }

    ServoCommandPacket packet;
    packet.values = values;
    dispatch_servo_packet_blocking(packet, false, &meta);
  };

  stream_engine_ = std::make_unique<estun_libs::EstunServoStreamEngine>(stream_cfg, send_cb);
  servo_trace_.reset(cfg_.servo_trace_enable, cfg_.servo_trace_capacity, cfg_.servo_trace_file);
  stream_engine_->start(hold_packet.values);
}

void EstunHardwareInterface::load_tracker_joint_limits_from_description()
{
  const std::string desc_share =
    ament_index_cpp::get_package_share_directory("estun_description");
  const std::filesystem::path joint_limits_path =
    std::filesystem::path(desc_share) / "config" / cfg_.robot_model / "joint_limits.yaml";
  const YAML::Node root = YAML::LoadFile(joint_limits_path.string());
  const YAML::Node joint_limits = root["joint_limits"];
  if (!joint_limits || !joint_limits.IsMap()) {
    throw std::runtime_error("joint_limits.yaml 缺少 joint_limits 映射");
  }

  const std::array<std::string, 6> joint_names{{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  }};

  for (size_t i = 0; i < joint_names.size(); ++i) {
    const YAML::Node joint_cfg = joint_limits[joint_names[i]];
    if (!joint_cfg || !joint_cfg.IsMap()) {
      throw std::runtime_error("joint_limits 缺少关节配置: " + joint_names[i]);
    }
    // 共享机型 joint_limits 遵循 ROS 标准单位 rad/s 与 rad/s^2；APOS 流引擎在
    // write() 后已经切到 SDK 的 deg 口径，因此 tracker 限速也必须同步转换。
    cfg_.servo_tracker_axis_max_velocity[i] = rad_to_deg(
      parse_required_finite_yaml_double(joint_cfg, "max_velocity", joint_names[i]));
    cfg_.servo_tracker_axis_max_acceleration[i] = rad_to_deg(
      parse_required_finite_yaml_double(joint_cfg, "max_acceleration", joint_names[i]));
  }

  // 兼容仍依赖统一标量统计/日志/回退路径的旧逻辑，统一标量取 6 轴中的最小值。
  cfg_.servo_tracker_max_velocity = *std::min_element(
    cfg_.servo_tracker_axis_max_velocity.begin(),
    cfg_.servo_tracker_axis_max_velocity.end());
  cfg_.servo_tracker_max_acceleration = *std::min_element(
    cfg_.servo_tracker_axis_max_acceleration.begin(),
    cfg_.servo_tracker_axis_max_acceleration.end());
}

void EstunHardwareInterface::load_tracker_cartesian_limits_from_description()
{
  const std::string desc_share =
    ament_index_cpp::get_package_share_directory("estun_description");
  const std::filesystem::path model_limits_path =
    std::filesystem::path(desc_share) / "config" / cfg_.robot_model / "cartesian_limits.yaml";

  const YAML::Node root = YAML::LoadFile(model_limits_path.string());
  const YAML::Node cartesian_limits = root["cartesian_limits"];
  if (!cartesian_limits || !cartesian_limits.IsMap()) {
    throw std::runtime_error("cartesian_limits.yaml 缺少 cartesian_limits 映射");
  }

  cfg_.cartesian_max_trans_velocity =
    parse_required_finite_yaml_double(
      cartesian_limits,
      "max_trans_vel",
      "cartesian_limits") * 1000.0;
  cfg_.cartesian_max_trans_acceleration =
    parse_required_finite_yaml_double(
      cartesian_limits,
      "max_trans_acc",
      "cartesian_limits") * 1000.0;
  cfg_.cartesian_max_rot_velocity =
    parse_required_finite_yaml_double(
      cartesian_limits,
      "max_rot_vel",
      "cartesian_limits");
  cfg_.cartesian_max_rot_acceleration =
    parse_optional_finite_yaml_double(
      cartesian_limits,
      "max_rot_acc",
      cfg_.cartesian_max_rot_acceleration,
      "cartesian_limits");
}

bool EstunHardwareInterface::wait_for_endpoint_release_ready(std::chrono::milliseconds timeout)
{
  if (!status_cache_.latest_is_endpoint()) {
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!status_cache_.latest_is_endpoint()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return !status_cache_.latest_is_endpoint();
}

void EstunHardwareInterface::stop_stream_engine()
{
  if (stream_engine_) {
    stream_engine_->stop();
    stream_engine_.reset();
  }
}

void EstunHardwareInterface::flush_stream_engine(const ServoCommandPacket & hold_packet)
{
  if (stream_engine_) {
    stream_engine_->set_mode(to_stream_command_mode(active_command_mode_), hold_packet.values);
  }
}

void EstunHardwareInterface::clear_command_queue()
{
  const auto hold_packet = streamed_hold_command_packet();
  if (stream_engine_) {
    stream_engine_->flush(hold_packet.values);
  }
}

void EstunHardwareInterface::sync_commands_from_latest_status()
{
  for (int i = 0; i < 6; ++i) {
    hw_states_[i] = status_cache_.load_joint_value(i);
    hw_commands_[i] = hw_states_[i];
    last_sent_joint_cmd_deg_[i] = hw_states_[i] * 180.0 / M_PI;
    last_streamed_joint_cmd_deg_[i] = last_sent_joint_cmd_deg_[i];

    cartesian_states_[i] = status_cache_.load_world_pose_value(i);
    cartesian_commands_[i] = cartesian_states_[i];
    last_sent_cart_cmd_[i] = cartesian_states_[i];
    last_streamed_cart_cmd_[i] = last_sent_cart_cmd_[i];
  }
}

void EstunHardwareInterface::update_estun_status_state_interfaces()
{
  const auto write_state = [this](EstunStatusStateIndex index, double value) {
    estun_status_states_[static_cast<size_t>(index)] = value;
  };

  const bool disconnected = status_cache_.is_disconnected();
  const bool first_packet = status_cache_.first_packet_received();

  write_state(EstunStatusStateIndex::CONNECTED, first_packet && !disconnected ? 1.0 : 0.0);
  write_state(
    EstunStatusStateIndex::ROBOT_ERROR,
    status_cache_.robot_error() ? 1.0 : 0.0);
  write_state(EstunStatusStateIndex::DISCONNECTED, disconnected ? 1.0 : 0.0);
  write_state(EstunStatusStateIndex::FIRST_PACKET_RECEIVED, first_packet ? 1.0 : 0.0);
  write_state(EstunStatusStateIndex::STATUS_PACKET_COUNT,
    static_cast<double>(status_cache_.status_packet_count()));
  write_state(EstunStatusStateIndex::ACTIVE_COMMAND_MODE,
    static_cast<double>(active_command_mode_ == CommandMode::CARTESIAN_POSE ? 1 : 0));
  write_state(EstunStatusStateIndex::CONFIGURED_SERVO_MODE,
    static_cast<double>(configured_servo_mode_ == CommandMode::CARTESIAN_POSE ? 1 : 0));
  write_state(EstunStatusStateIndex::QUEUE_DEPTH,
    static_cast<double>(stat_queue_current_depth_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::QUEUE_MAX_DEPTH,
    static_cast<double>(stat_queue_max_depth_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::QUEUE_PUSH_COUNT,
    static_cast<double>(stat_queue_push_count_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::QUEUE_TAKE_COUNT,
    static_cast<double>(stat_queue_pop_success_count_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::QUEUE_FULL_DROP_COUNT,
    static_cast<double>(stat_queue_full_drop_count_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::QUEUE_UNDERFLOW_COUNT,
    static_cast<double>(stat_queue_underflow_count_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::REPEATED_SEND_COUNT,
    static_cast<double>(stat_repeated_send_count_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::SERVO_CALL_COUNT,
    static_cast<double>(stat_servo_call_count_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::SERVO_SDK_FAIL_COUNT,
    static_cast<double>(stat_servo_sdk_fail_count_.load(std::memory_order_relaxed)));
  write_state(EstunStatusStateIndex::SERVO_BLOCK_NS_MAX,
    static_cast<double>(stat_servo_block_ns_max_.load(std::memory_order_relaxed)));
}

EstunHardwareInterface::ServoCommandPacket EstunHardwareInterface::hold_command_packet() const
{
  ServoCommandPacket packet;
  if (active_command_mode_ == CommandMode::CARTESIAN_POSE) {
    packet.values = last_sent_cart_cmd_;
  } else {
    packet.values = last_sent_joint_cmd_deg_;
  }
  return packet;
}

EstunHardwareInterface::ServoCommandPacket EstunHardwareInterface::streamed_hold_command_packet() const
{
  ServoCommandPacket packet;
  if (stream_engine_) {
    packet.values = stream_engine_->current_hold_values();
    return packet;
  }

  if (active_command_mode_ == CommandMode::CARTESIAN_POSE) {
    packet.values = last_streamed_cart_cmd_;
  } else {
    packet.values = last_streamed_joint_cmd_deg_;
  }
  return packet;
}

void EstunHardwareInterface::dispatch_servo_packet_blocking(
  const ServoCommandPacket & packet,
  bool is_end_point,
  const estun_libs::EstunServoSendMeta * send_meta)
{
  // 注意：servoToAPOS/servoToCPOS 等 SDK 接口是阻塞调用，
  // 是否继续读取由控制器侧调度决定。这里不要额外 sleep。
  // 当前 SDK 版本中 time_stamp_ 主要用于接口兼容，暂不作为有效节拍依据。
  const auto start_time = std::chrono::steady_clock::now();
  int sdk_ret = -1;
  std::array<double, 6> sdk_trace_values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (active_command_mode_ == CommandMode::CARTESIAN_POSE) {
    double sdk_cart_cmd[6];
    cartesian_ros_to_sdk(packet.values, sdk_cart_cmd);
    for (size_t i = 0; i < 6; ++i) {
      sdk_trace_values[i] = sdk_cart_cmd[i];
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    sdk_ret = eri_manager_.servoToCPOS(time_stamp_, sdk_cart_cmd, is_end_point);
  } else {
    double sdk_joint_cmd[6];
    std::copy(packet.values.begin(), packet.values.end(), sdk_joint_cmd);
    for (size_t i = 0; i < 6; ++i) {
      sdk_trace_values[i] = sdk_joint_cmd[i];
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    sdk_ret = eri_manager_.servoToAPOS(time_stamp_, sdk_joint_cmd, is_end_point);
  }
  // SDK 文档语义：仅返回 -1 表示失败；0/1 等非负值都视为成功态。
  if (sdk_ret < 0) {
    const uint64_t fail_total =
      stat_servo_sdk_fail_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    const int64_t now_ms = steady_now_ms();
    const int64_t last_log_ms = last_sdk_fail_log_time_ms_.load(std::memory_order_relaxed);
    if (now_ms - last_log_ms > 2000) {
      RCLCPP_WARN(
        rclcpp::get_logger("EstunHardwareInterface"),
        "SDK servo 发包返回失败(ret=%d)，累计失败 %llu 次 (timestamp=%llu)",
        sdk_ret,
        static_cast<unsigned long long>(fail_total),
        static_cast<unsigned long long>(time_stamp_));
      last_sdk_fail_log_time_ms_.store(now_ms, std::memory_order_relaxed);
    }
  }
  const auto block_duration_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start_time).count());
  stat_servo_call_count_.fetch_add(1, std::memory_order_relaxed);
  stat_servo_block_ns_total_.fetch_add(block_duration_ns, std::memory_order_relaxed);
  uint64_t observed_max_ns = stat_servo_block_ns_max_.load(std::memory_order_relaxed);
  while (block_duration_ns > observed_max_ns &&
         !stat_servo_block_ns_max_.compare_exchange_weak(
           observed_max_ns, block_duration_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {
  }

  // 停机/清队列必须锚定“真实已发出的最后一帧”，而不是控制器侧更靠前的目标缓存，
  // 否则在 FIFO 仍有尾队列或 Ctrl+C 中断时，结束帧可能相对实际输出发生跳变。
  if (active_command_mode_ == CommandMode::CARTESIAN_POSE) {
    last_streamed_cart_cmd_ = packet.values;
  } else {
    last_streamed_joint_cmd_deg_ = packet.values;
  }
  servo_trace_.record(
    packet.values,
    sdk_trace_values,
    block_duration_ns,
    status_cache_.status_packet_count(),
    time_stamp_,
    send_meta);
}

hardware_interface::CallbackReturn EstunHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // 防止重复 configure 时残留线程或控制态影响当前流程
  (void)stop();
  reset_shutdown_stop_state();
  stop_completed_.store(false, std::memory_order_relaxed);
  stop_in_progress_.store(false, std::memory_order_relaxed);
  status_cache_.set_latest_is_endpoint(false);
  test_stub_do_values_.fill(false);

  bool synchronized = false;
  if (cfg_.test_stub_sdk) {
    RCLCPP_WARN(
      rclcpp::get_logger("EstunHardwareInterface"),
      "测试桩模式：跳过 SDK 通信初始化与首包等待。");
    const int64_t now_ms = steady_now_ms();
    reset_status_cache();
    status_cache_.set_first_packet_received(true);
    status_cache_.set_last_recv_timestamp_ms(now_ms);
    status_cache_.set_is_disconnected(cfg_.test_force_disconnect);
    status_cache_.set_robot_error(cfg_.test_force_alarm);
    synchronized = true;
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("EstunHardwareInterface"), "配置通信链路: %s", cfg_.robot_ip.c_str());
    if (!acquire_status_callback_slot()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EstunHardwareInterface"),
        "配置失败：Estun 状态回调槽位已耗尽（最多支持 4 个硬件实例）。");
      is_configured_ = false;
      return hardware_interface::CallbackReturn::ERROR;
    }
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      if (!eri_manager_.setCommunicationParam(cfg_.robot_ip.c_str(), cfg_.cmd_port, cfg_.servo_port, cfg_.status_port)) {
        RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "SDK 通信参数设置失败");
        is_configured_ = false;
        release_status_callback_slot();
        return hardware_interface::CallbackReturn::ERROR;
      }
      switch (callback_slot_index_) {
        case 0:
          eri_manager_.setRobotStatusCallback(status_callback_slot_0);
          break;
        case 1:
          eri_manager_.setRobotStatusCallback(status_callback_slot_1);
          break;
        case 2:
          eri_manager_.setRobotStatusCallback(status_callback_slot_2);
          break;
        case 3:
          eri_manager_.setRobotStatusCallback(status_callback_slot_3);
          break;
        default:
          RCLCPP_ERROR(
            rclcpp::get_logger("EstunHardwareInterface"),
            "配置失败：回调槽位索引非法=%d",
            callback_slot_index_);
          is_configured_ = false;
          release_status_callback_slot();
          return hardware_interface::CallbackReturn::ERROR;
      }
    }
    status_cache_.set_first_packet_received(false);
    status_cache_.set_is_disconnected(false);

    RCLCPP_DEBUG(rclcpp::get_logger("EstunHardwareInterface"), "等待首包状态并同步真机初始位置...");
    for (int retry = 0; retry < 100; ++retry) {
      if (status_cache_.first_packet_received()) {
        synchronized = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  if (!synchronized) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EstunHardwareInterface"),
      "配置失败：2 秒内未接收到 UDP 状态包。请检查机器人上电/网络连通、UDP 端口占用、"
      "以及 SDK socket 是否创建成功（权限/能力位）。");
    RCLCPP_ERROR(
      rclcpp::get_logger("EstunHardwareInterface"),
      "真机通信参数: robot_ip=%s cmd_port=%u servo_port=%u status_port=%u",
      cfg_.robot_ip.c_str(),
      static_cast<unsigned int>(cfg_.cmd_port),
      static_cast<unsigned int>(cfg_.servo_port),
      static_cast<unsigned int>(cfg_.status_port));
    is_configured_ = false;
    release_status_callback_slot();
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < 6; ++i) {
    hw_states_[i] = status_cache_.load_joint_value(i);
    hw_commands_[i] = hw_states_[i];
    last_sent_joint_cmd_deg_[i] = hw_states_[i] * 180.0 / M_PI;
    cartesian_states_[i] = status_cache_.load_world_pose_value(i);
    cartesian_commands_[i] = cartesian_states_[i];
    last_sent_cart_cmd_[i] = cartesian_states_[i];
  }

  active_command_mode_ = configured_servo_mode_;
  pending_command_mode_ = configured_servo_mode_;
  mode_switch_pending_ = false;
  is_configured_ = true;
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "当前 servo 模式固定为: %s (0=APOS,1=CPOS)",
    cfg_.servo_mode.c_str());
  if (!start_service_bridge()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EstunHardwareInterface"),
      "配置失败：SDK service bridge 启动失败。");
    is_configured_ = false;
    release_status_callback_slot();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "通信配置完成，硬件接口已就绪。");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EstunHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  state_interfaces.emplace_back(cartesian_interface_name_, "x", &cartesian_states_[0]);
  state_interfaces.emplace_back(cartesian_interface_name_, "y", &cartesian_states_[1]);
  state_interfaces.emplace_back(cartesian_interface_name_, "z", &cartesian_states_[2]);
  state_interfaces.emplace_back(cartesian_interface_name_, "a", &cartesian_states_[3]);
  state_interfaces.emplace_back(cartesian_interface_name_, "b", &cartesian_states_[4]);
  state_interfaces.emplace_back(cartesian_interface_name_, "c", &cartesian_states_[5]);

  update_estun_status_state_interfaces();
  for (size_t i = 0; i < kEstunStatusStateInterfaceNames.size(); ++i) {
    state_interfaces.emplace_back(
      estun_status_interface_name_,
      kEstunStatusStateInterfaceNames[i],
      &estun_status_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EstunHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  command_interfaces.emplace_back(cartesian_interface_name_, "x", &cartesian_commands_[0]);
  command_interfaces.emplace_back(cartesian_interface_name_, "y", &cartesian_commands_[1]);
  command_interfaces.emplace_back(cartesian_interface_name_, "z", &cartesian_commands_[2]);
  command_interfaces.emplace_back(cartesian_interface_name_, "a", &cartesian_commands_[3]);
  command_interfaces.emplace_back(cartesian_interface_name_, "b", &cartesian_commands_[4]);
  command_interfaces.emplace_back(cartesian_interface_name_, "c", &cartesian_commands_[5]);
  return command_interfaces;
}

hardware_interface::CallbackReturn EstunHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!is_configured_) {
    RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "尚未配置完成，请先完成 on_configure。");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("EstunHardwareInterface"), "正在执行 ERI 夺权并进入实时控制模式...");
  sync_commands_from_latest_status();
  const std::array<double, 6> movej_target_deg = last_sent_joint_cmd_deg_;
  double current_deg[6];
  std::copy(movej_target_deg.begin(), movej_target_deg.end(), current_deg);

  clear_command_queue();
  stop_stream_engine();
  stat_queue_push_count_.store(0, std::memory_order_relaxed);
  stat_queue_pop_success_count_.store(0, std::memory_order_relaxed);
  stat_queue_full_drop_count_.store(0, std::memory_order_relaxed);
  stat_queue_empty_hold_count_.store(0, std::memory_order_relaxed);
  stat_queue_underflow_count_.store(0, std::memory_order_relaxed);
  stat_repeated_send_count_.store(0, std::memory_order_relaxed);
  stat_queue_current_depth_.store(0, std::memory_order_relaxed);
  stat_queue_max_depth_.store(0, std::memory_order_relaxed);
  stat_servo_call_count_.store(0, std::memory_order_relaxed);
  stat_servo_block_ns_total_.store(0, std::memory_order_relaxed);
  stat_servo_block_ns_max_.store(0, std::memory_order_relaxed);
  stat_servo_sdk_fail_count_.store(0, std::memory_order_relaxed);

  if (cfg_.test_stub_sdk) {
    RCLCPP_WARN(
      rclcpp::get_logger("EstunHardwareInterface"),
      "测试桩模式：跳过 SDK 夺权与实时发包，启用本地实时环路桩。");
    time_stamp_ = 0;
    status_cache_.set_is_disconnected(cfg_.test_force_disconnect);
    start_stream_engine(hold_command_packet());
    RCLCPP_INFO(
      rclcpp::get_logger("EstunHardwareInterface"),
      "激活成功：测试桩模式已启用流引擎。");

    // 只有成功进入新一轮控制会话后，才清空上一轮 stop 状态。
    stop_completed_.store(false, std::memory_order_relaxed);
    stop_in_progress_.store(false, std::memory_order_relaxed);
    reset_shutdown_stop_state();
    is_control_active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  bool control_acquired = false;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!eri_manager_.getControlRobot()) {
      RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "获取控制权失败！");
      return hardware_interface::CallbackReturn::ERROR;
    }
    control_acquired = true;
    RCLCPP_DEBUG(rclcpp::get_logger("EstunHardwareInterface"), "获取控制权成功。");

    if (!eri_manager_.setERIMotionParam(cfg_.motion_period_ms, 0)) {
      RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "设置运动参数失败，正在回滚释放控制权。");
      if (!eri_manager_.releaseControlOverRobot()) {
        RCLCPP_WARN(rclcpp::get_logger("EstunHardwareInterface"), "回滚失败：releaseControlOverRobot 调用失败。");
      }
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("EstunHardwareInterface"),
      "设置运动参数成功：motion_period_ms=%u。",
      cfg_.motion_period_ms);
  }

  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!eri_manager_.movJ(current_deg)) {
      RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "启动前锚定位置 (moveJ) 失败！");
      if (control_acquired && !eri_manager_.releaseControlOverRobot()) {
        RCLCPP_WARN(rclcpp::get_logger("EstunHardwareInterface"), "回滚失败：releaseControlOverRobot 调用失败。");
      }
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_DEBUG(rclcpp::get_logger("EstunHardwareInterface"), "启动前锚定位置 (moveJ) 成功。");
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
  sync_commands_from_latest_status();
  double max_movej_settle_delta_deg = 0.0;
  for (size_t i = 0; i < movej_target_deg.size(); ++i) {
    max_movej_settle_delta_deg =
      std::max(max_movej_settle_delta_deg, std::fabs(last_sent_joint_cmd_deg_[i] - movej_target_deg[i]));
  }
  RCLCPP_INFO(
    rclcpp::get_logger("EstunHardwareInterface"),
    "启动锚点已按 moveJ 后状态重同步: max_delta_deg=%.9f",
    max_movej_settle_delta_deg);
  const ServoCommandPacket hold_packet = hold_command_packet();

  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!eri_manager_.startServo(to_servo_motion_type(active_command_mode_))) {
      RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "startServo 失败！");
      if (control_acquired && !eri_manager_.releaseControlOverRobot()) {
        RCLCPP_WARN(rclcpp::get_logger("EstunHardwareInterface"), "回滚失败：releaseControlOverRobot 调用失败。");
      }
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "startServo 成功（mode=%s）。",
    active_command_mode_ == CommandMode::CARTESIAN_POSE ? "CPOS" : "APOS");

  time_stamp_ = 0;
  status_cache_.set_is_disconnected(false);
  start_stream_engine(hold_packet);

  // 只有成功进入新一轮控制会话后，才清空上一轮 stop 状态。
  stop_completed_.store(false, std::memory_order_relaxed);
  stop_in_progress_.store(false, std::memory_order_relaxed);
  reset_shutdown_stop_state();
  is_control_active_ = true;
  RCLCPP_DEBUG(
    rclcpp::get_logger("EstunHardwareInterface"),
    "激活成功：机器人已进入%s控制模式。",
    " 流引擎 ");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EstunHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  if (status_cache_.robot_error()) {
    RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "机器人处于报警状态！");
    return hardware_interface::return_type::ERROR;
  }

  bool start_joint = false;
  bool start_cartesian = false;

  for (const auto & interface_name : start_interfaces) {
    if (is_joint_position_interface(interface_name)) {
      start_joint = true;
    } else if (is_cartesian_pose_interface(interface_name)) {
      start_cartesian = true;
    }
  }

  (void)stop_interfaces;

  if (start_joint && start_cartesian) {
    RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "模式冲突：不允许同时启动关节空间与笛卡尔空间控制。");
    return hardware_interface::return_type::ERROR;
  }

  std::optional<CommandMode> requested_mode;
  if (start_joint) {
    requested_mode = CommandMode::JOINT_POSITION;
  } else if (start_cartesian) {
    requested_mode = CommandMode::CARTESIAN_POSE;
  }

  if (requested_mode.has_value() && requested_mode.value() != configured_servo_mode_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EstunHardwareInterface"),
      "当前连接是固定模式 '%s'，不能切换到另一种 servo 模式。请重新启动并在 xacro/launch 里设置 servo_mode。",
      cfg_.servo_mode.c_str());
    return hardware_interface::return_type::ERROR;
  }

  mode_switch_pending_ = false;
  if (requested_mode.has_value()) {
    pending_command_mode_ = requested_mode.value();
    mode_switch_pending_ = (pending_command_mode_ != active_command_mode_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EstunHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  if (!mode_switch_pending_) {
    return hardware_interface::return_type::OK;
  }

  // SDK 约束：startServo 模式在连接时固定，此处只同步内部状态，不再重启 servo 模式。
  active_command_mode_ = pending_command_mode_;
  mode_switch_pending_ = false;
  clear_command_queue();
  flush_stream_engine(hold_command_packet());

  if (active_command_mode_ == CommandMode::CARTESIAN_POSE) {
    RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "模式已切换为笛卡尔空间实时控制 (servoToCPOS)");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "模式已切换为关节空间实时控制 (servoToAPOS)");
  }
  return hardware_interface::return_type::OK;
}

// read()：读取最新机器人状态并做故障门禁。
hardware_interface::return_type EstunHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!is_control_active_) {
    update_estun_status_state_interfaces();
    return hardware_interface::return_type::OK;
  }

  if (cfg_.test_stub_sdk) {
    const int64_t now_ms = steady_now_ms();
    status_cache_.set_last_recv_timestamp_ms(now_ms);
    if (cfg_.test_force_disconnect) {
      status_cache_.set_is_disconnected(true);
      update_estun_status_state_interfaces();
      return hardware_interface::return_type::ERROR;
    }
    status_cache_.set_is_disconnected(false);
    if (cfg_.test_force_alarm) {
      status_cache_.set_robot_error(true);
      update_estun_status_state_interfaces();
      return hardware_interface::return_type::ERROR;
    }
    status_cache_.set_robot_error(false);
    for (int i = 0; i < 6; i++) {
      hw_states_[i] = status_cache_.load_joint_value(i);
      cartesian_states_[i] = status_cache_.load_world_pose_value(i);
    }
    update_estun_status_state_interfaces();
    return hardware_interface::return_type::OK;
  }

  try 
  {
      // 看门狗超时判定：100ms 未收到状态包则视为断连。
      int64_t current_time_ms = steady_now_ms();
          
      if (current_time_ms - status_cache_.last_recv_timestamp_ms() > 100) {
          status_cache_.set_is_disconnected(true);
          const int64_t last_log_ms = last_read_disconnect_log_time_ms_.load(std::memory_order_relaxed);
          if (current_time_ms - last_log_ms > 5000) {
              RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "检测到机器人 UDP 连接超时断开。");
              last_read_disconnect_log_time_ms_.store(current_time_ms, std::memory_order_relaxed);
          }
          update_estun_status_state_interfaces();
          return hardware_interface::return_type::ERROR; 
      } else {
          status_cache_.set_is_disconnected(false);
      }

      // 2. 读取最新状态
      for (int i = 0; i < 6; i++) {
        hw_states_[i] = status_cache_.load_joint_value(i);
        cartesian_states_[i] = status_cache_.load_world_pose_value(i);
      }

      // 3. 状态监控与限流打印
      if (status_cache_.robot_error()) {
          const int64_t last_log_ms = last_read_alarm_log_time_ms_.load(std::memory_order_relaxed);
          if (current_time_ms - last_log_ms > 5000) {
              RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "检测到机器人硬件报警。");
              last_read_alarm_log_time_ms_.store(current_time_ms, std::memory_order_relaxed);
          }
          // 报警时返回 ERROR，让 ROS 规划器知道执行失败
          update_estun_status_state_interfaces();
          return hardware_interface::return_type::ERROR; 
      }
  }
  // 捕获标准异常。
  catch (const std::exception& e) 
  {
      const int64_t now_ms = steady_now_ms();
      const int64_t last_log_ms = last_read_exception_log_time_ms_.load(std::memory_order_relaxed);
      if (now_ms - last_log_ms > 5000) {
          RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "read() 发生异常 (很可能是线程安全问题): %s", e.what());
          last_read_exception_log_time_ms_.store(now_ms, std::memory_order_relaxed);
      }
      update_estun_status_state_interfaces();
      return hardware_interface::return_type::ERROR;
  }
  // 捕获未知异常并返回错误。
  catch (...) 
  {
      const int64_t now_ms = steady_now_ms();
      const int64_t last_log_ms = last_read_unknown_exception_log_time_ms_.load(std::memory_order_relaxed);
      if (now_ms - last_log_ms > 5000) {
          RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "read() 发生未知异常。");
          last_read_unknown_exception_log_time_ms_.store(now_ms, std::memory_order_relaxed);
      }
      update_estun_status_state_interfaces();
      return hardware_interface::return_type::ERROR;
  }

  update_estun_status_state_interfaces();
  return hardware_interface::return_type::OK;
}

// write()：将控制器命令写入流引擎。
hardware_interface::return_type EstunHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (consume_shutdown_stop_request()) {
      RCLCPP_INFO(
        rclcpp::get_logger("EstunHardwareInterface"),
        "检测到提前停机请求，优先执行 stop() 收尾。");
      const auto stop_ret = stop();
      update_estun_status_state_interfaces();
      return stop_ret == hardware_interface::CallbackReturn::SUCCESS ?
        hardware_interface::return_type::OK :
        hardware_interface::return_type::ERROR;
  }

  if (stop_in_progress_.load(std::memory_order_acquire)) {
      update_estun_status_state_interfaces();
      return hardware_interface::return_type::OK;
  }

  // 安全门禁：未激活、断线或报警时拒绝写入并清空队列。
  if (!is_control_active_ ||
      status_cache_.robot_error() ||
      status_cache_.is_disconnected()) {
      clear_command_queue(); // 立即清空流引擎队列
      stat_queue_current_depth_.store(0, std::memory_order_relaxed);
      update_estun_status_state_interfaces();
      return hardware_interface::return_type::ERROR; 
  }

  try 
  {
      ServoCommandPacket packet;
      if (active_command_mode_ == CommandMode::CARTESIAN_POSE) {
        for (int i = 0; i < 6; ++i) {
          double target = cartesian_commands_[i];
          if (!std::isfinite(target)) {
            target = last_sent_cart_cmd_[i];
            const int64_t now_ms = steady_now_ms();
            const int64_t last_log_ms = last_cartesian_nan_log_time_ms_.load(std::memory_order_relaxed);
            if (now_ms - last_log_ms > 2000) {
              RCLCPP_WARN(
                rclcpp::get_logger("EstunHardwareInterface"),
                "检测到非法笛卡尔指令(NaN/Inf)，本周期已回退到上一帧指令。");
              last_cartesian_nan_log_time_ms_.store(now_ms, std::memory_order_relaxed);
            }
          }
          packet.values[i] = target;
        }
      } else {
        for (int i = 0; i < 6; i++) {
          double target_deg = hw_commands_[i] * 180.0 / M_PI;
          if (!std::isfinite(target_deg)) {
            target_deg = last_sent_joint_cmd_deg_[i];
            const int64_t now_ms = steady_now_ms();
            const int64_t last_log_ms = last_joint_nan_log_time_ms_.load(std::memory_order_relaxed);
            if (now_ms - last_log_ms > 2000) {
              RCLCPP_WARN(
                rclcpp::get_logger("EstunHardwareInterface"),
                "检测到非法关节指令(NaN/Inf)，本周期已回退到上一帧指令。");
              last_joint_nan_log_time_ms_.store(now_ms, std::memory_order_relaxed);
            }
          }
          packet.values[i] = target_deg;
        }
      }

      if (!stream_engine_) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EstunHardwareInterface"),
          "流引擎未初始化。");
        return hardware_interface::return_type::ERROR;
      }

      uint64_t dropped_this_write = 0;
      const auto enqueue_time = std::chrono::steady_clock::now();
      const bool accepted =
        (cfg_.stream_policy == StreamPolicy::LATEST_OVERWRITE)
        ? stream_engine_->updateLatestCommand(packet.values, enqueue_time)
        : stream_engine_->enqueue(packet.values, enqueue_time);
      if (!accepted) {
        ++dropped_this_write;
      } else if (active_command_mode_ == CommandMode::CARTESIAN_POSE) {
        // 仅在成功进入流引擎后推进 hold 锚点，避免把被拒命令误记为 last_sent。
        last_sent_cart_cmd_ = packet.values;
      } else {
        // 仅在成功进入流引擎后推进 hold 锚点，避免把被拒命令误记为 last_sent。
        last_sent_joint_cmd_deg_ = packet.values;
      }

      const estun_libs::EstunServoStreamStats stream_stats = stream_engine_->snapshot();
      stat_queue_push_count_.store(stream_stats.write_total, std::memory_order_relaxed);
      stat_queue_pop_success_count_.store(stream_stats.take_total, std::memory_order_relaxed);
      stat_queue_empty_hold_count_.store(stream_stats.hold_send, std::memory_order_relaxed);
      stat_queue_underflow_count_.store(stream_stats.underflow_count, std::memory_order_relaxed);
      stat_repeated_send_count_.store(stream_stats.repeated_send_count, std::memory_order_relaxed);
      stat_queue_current_depth_.store(stream_stats.queue_depth, std::memory_order_relaxed);
      stat_queue_max_depth_.store(stream_stats.queue_max, std::memory_order_relaxed);
      update_estun_status_state_interfaces();

      // 队列空(underflow)属于危险信号：即使关闭了周期诊断，也要输出告警。
      // 为避免实时链路日志刷屏，采用短节流窗口，并把节流窗口内的事件按增量汇总。
      {
        const uint64_t total_underflow = stream_stats.underflow_count;
        uint64_t prev_underflow_warn_count =
          prev_underflow_warn_count_.load(std::memory_order_relaxed);

        if (total_underflow < prev_underflow_warn_count) {
          prev_underflow_warn_count_.store(total_underflow, std::memory_order_relaxed);
          prev_underflow_warn_count = total_underflow;
        }

        if (total_underflow > prev_underflow_warn_count) {
          const int64_t now_underflow_ms = steady_now_ms();
          const int64_t elapsed_ms =
            now_underflow_ms - last_underflow_warn_time_ms_.load(std::memory_order_relaxed);
          if (elapsed_ms >= kQueueUnderflowWarnThrottleMs) {
            const uint64_t delta_underflow = total_underflow - prev_underflow_warn_count;
            RCLCPP_WARN(
              rclcpp::get_logger("EstunHardwareInterface"),
              "检测到流引擎队列空(underflow): 新增=%llu 累计=%llu depth=%zu target=%zu low=%zu hold_send=%llu",
              static_cast<unsigned long long>(delta_underflow),
              static_cast<unsigned long long>(total_underflow),
              stream_stats.queue_depth,
              stream_stats.target_depth,
              kInternalStreamLowWatermark,
              static_cast<unsigned long long>(stream_stats.hold_send));
            last_underflow_warn_time_ms_.store(now_underflow_ms, std::memory_order_relaxed);
            prev_underflow_warn_count_.store(total_underflow, std::memory_order_relaxed);
          }
        }
      }

      if (dropped_this_write > 0) {
        const uint64_t dropped_total =
          stat_queue_full_drop_count_.fetch_add(dropped_this_write, std::memory_order_relaxed) +
          dropped_this_write;
        const int64_t now_ms = steady_now_ms();
        const int64_t last_log_ms = last_drop_log_time_ms_.load(std::memory_order_relaxed);
        if (now_ms - last_log_ms > 2000) {
          RCLCPP_WARN(
            rclcpp::get_logger("EstunHardwareInterface"),
            "流引擎拒绝写入(policy=%s depth=%zu max=%zu)，本周期丢弃 %llu 条，累计丢弃 %llu 条命令。",
            cfg_.stream_policy == StreamPolicy::LATEST_OVERWRITE ? "latest_overwrite" : "fifo",
            stream_stats.queue_depth,
            stream_stats.queue_max,
            static_cast<unsigned long long>(dropped_this_write),
            static_cast<unsigned long long>(dropped_total));
          last_drop_log_time_ms_.store(now_ms, std::memory_order_relaxed);
        }
      }

  }
  catch (const std::exception& e) 
  {
      const int64_t now_ms = steady_now_ms();
      const int64_t last_log_ms = last_write_exception_log_time_ms_.load(std::memory_order_relaxed);
      if (now_ms - last_log_ms > 5000) {
          RCLCPP_ERROR(rclcpp::get_logger("EstunHardwareInterface"), "write() 发生异常: %s", e.what());
          last_write_exception_log_time_ms_.store(now_ms, std::memory_order_relaxed);
      }
      return hardware_interface::return_type::ERROR;
  }
  catch (...) 
  {
      return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn EstunHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (stop_completed_.load(std::memory_order_acquire)) {
    RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "on_deactivate: stop 已提前完成，直接跳过重复收尾。");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "on_deactivate: 正在停止实时控制...");
  return stop();
}

hardware_interface::CallbackReturn EstunHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "on_cleanup: 回收运行时资源...");
  auto ret = hardware_interface::CallbackReturn::SUCCESS;
  if (!stop_completed_.load(std::memory_order_acquire)) {
    ret = stop();
  } else {
    RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "on_cleanup: stop 已提前完成，跳过重复 stop。");
  }
  stop_service_bridge();
  release_status_callback_slot();
  is_configured_ = false;
  return ret;
}

hardware_interface::CallbackReturn EstunHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "on_shutdown: 驱动即将退出...");
  auto ret = hardware_interface::CallbackReturn::SUCCESS;
  if (!stop_completed_.load(std::memory_order_acquire)) {
    ret = stop();
  } else {
    RCLCPP_INFO(rclcpp::get_logger("EstunHardwareInterface"), "on_shutdown: stop 已提前完成，跳过重复 stop。");
  }
  stop_service_bridge();
  release_status_callback_slot();
  is_configured_ = false;
  return ret;
}

hardware_interface::CallbackReturn EstunHardwareInterface::stop()
{
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  if (stop_completed_.load(std::memory_order_acquire)) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  if (stop_in_progress_.exchange(true, std::memory_order_acq_rel)) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  shutdown_stop_state_.completed.store(false, std::memory_order_release);

  const ServoCommandPacket soft_stop_anchor = streamed_hold_command_packet();

  if (stream_engine_ && stream_engine_->running()) {
    if (cfg_.stream_policy == StreamPolicy::LATEST_OVERWRITE) {
      stream_engine_->updateLatestCommand(soft_stop_anchor.values, std::chrono::steady_clock::now());
    } else {
      stream_engine_->flush(soft_stop_anchor.values);
    }

    if (!cfg_.test_stub_sdk) {
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int64_t>(cfg_.motion_period_ms * kStopSoftHoldCycles)));
    }
  }

  const ServoCommandPacket stop_hold_packet = streamed_hold_command_packet();

  // 1. 停止流引擎，确保不再有新的发包动作
  stop_stream_engine();

  // 2. 如处于控制态，发送结束帧并释放控制权
  if (is_control_active_) {
    if (!cfg_.test_stub_sdk) {
      // SDK 的结束帧调用同样是阻塞行为，保持和实时环路一致的调用语义。
      const ServoCommandPacket end_packet = stop_hold_packet;
      dispatch_servo_packet_blocking(end_packet, true);
      (void)wait_for_endpoint_release_ready(
        std::chrono::milliseconds(kEndpointReleaseReadyTimeoutMs));
      {
        std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
        if (!eri_manager_.releaseControlOverRobot()) {
          RCLCPP_WARN(rclcpp::get_logger("EstunHardwareInterface"), "releaseControlOverRobot 调用失败，已继续清理。");
        }
      }
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("EstunHardwareInterface"),
        "测试桩模式：跳过 SDK 结束帧与控制权释放。");
    }
    is_control_active_ = false;
  }

  // 3. 清空控制队列，重置缓存状态
  clear_command_queue();
  servo_trace_.dump_to_file(rclcpp::get_logger("EstunHardwareInterface"));
  status_cache_.set_is_disconnected(false);
  stop_completed_.store(true, std::memory_order_release);
  stop_in_progress_.store(false, std::memory_order_release);
  mark_shutdown_stop_resolved();

  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace estun_hardware

PLUGINLIB_EXPORT_CLASS(estun_hardware::EstunHardwareInterface, hardware_interface::SystemInterface)
