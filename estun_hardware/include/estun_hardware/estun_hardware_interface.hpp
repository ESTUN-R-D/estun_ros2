#ifndef ESTUN_HARDWARE__ESTUN_HARDWARE_INTERFACE_HPP_
#define ESTUN_HARDWARE__ESTUN_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <memory>
#include <optional>
#include <array>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "estun_msgs/srv/get_connection_status.hpp"
#include "estun_msgs/srv/get_cur_err_msg.hpp"
#include "estun_msgs/srv/get_do.hpp"
#include "estun_msgs/srv/get_joint_value.hpp"
#include "estun_msgs/srv/get_robot_conn_status.hpp"
#include "estun_msgs/srv/get_tool.hpp"
#include "estun_msgs/srv/get_user.hpp"
#include "estun_msgs/srv/get_world_cpos.hpp"
#include "estun_msgs/srv/set_do.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "estun_hardware/estun_hardware_config.hpp"
#include "estun_hardware/estun_service_bridge.hpp"
#include "estun_hardware/estun_servo_trace.hpp"
#include "estun_hardware/estun_status_cache.hpp"
#include "estun_libs/estun_servo_stream_engine.hpp"

// 引入埃斯顿 SDK 头文件
#include "ERIParamManager.h"

namespace estun_hardware
{
void request_estun_shutdown_stop();
bool wait_for_estun_shutdown_stop_completion(std::chrono::milliseconds timeout);

class EstunHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EstunHardwareInterface)
  ~EstunHardwareInterface() override;

  // 标准生命周期函数
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  
  // 核心读取与写入循环
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 模式切换预检：在控制器激活前完成合法性检查。
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // 模式切换执行：在控制器激活阶段同步内部模式状态。
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  friend class EstunHardwareInterfaceRuntimeTest;
  friend void request_estun_shutdown_stop();
  friend bool wait_for_estun_shutdown_stop_completion(std::chrono::milliseconds timeout);

  static constexpr size_t kMaxCallbackSlots = 4;

  enum class CommandMode
  {
    JOINT_POSITION = 0,
    CARTESIAN_POSE = 1,
  };

  enum class EstunStatusStateIndex : size_t
  {
    CONNECTED = 0,
    ROBOT_ERROR,
    DISCONNECTED,
    FIRST_PACKET_RECEIVED,
    STATUS_PACKET_COUNT,
    ACTIVE_COMMAND_MODE,
    CONFIGURED_SERVO_MODE,
    QUEUE_DEPTH,
    QUEUE_MAX_DEPTH,
    QUEUE_PUSH_COUNT,
    QUEUE_TAKE_COUNT,
    QUEUE_FULL_DROP_COUNT,
    QUEUE_UNDERFLOW_COUNT,
    REPEATED_SEND_COUNT,
    SERVO_CALL_COUNT,
    SERVO_SDK_FAIL_COUNT,
    SERVO_BLOCK_NS_MAX,
    COUNT,
  };

  struct ServoCommandPacket
  {
    std::array<double, 6> values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  };

  struct ShutdownStopState
  {
    std::atomic<bool> requested{false};
    std::atomic<bool> completed{false};
  };

  hardware_interface::CallbackReturn stop();
  bool start_service_bridge();
  void stop_service_bridge();

  bool is_joint_position_interface(const std::string & interface_name) const;
  bool is_cartesian_pose_interface(const std::string & interface_name) const;
  int to_servo_motion_type(CommandMode mode) const;
  estun_libs::EstunServoStreamMode to_stream_command_mode(CommandMode mode) const;
  void start_stream_engine(const ServoCommandPacket & hold_packet);
  void stop_stream_engine();
  void flush_stream_engine(const ServoCommandPacket & hold_packet);
  void clear_command_queue();
  void sync_commands_from_latest_status();
  ServoCommandPacket hold_command_packet() const;
  ServoCommandPacket streamed_hold_command_packet() const;
  void dispatch_servo_packet_blocking(
    const ServoCommandPacket & packet,
    bool is_end_point,
    const estun_libs::EstunServoSendMeta * send_meta = nullptr);
  void update_estun_status_state_interfaces();
  void load_tracker_joint_limits_from_description();
  void load_tracker_cartesian_limits_from_description();
  bool wait_for_endpoint_release_ready(std::chrono::milliseconds timeout);
  void handle_robot_status(const RobotStatus & status);
  void request_shutdown_stop();
  bool consume_shutdown_stop_request();
  void reset_shutdown_stop_state();
  void mark_shutdown_stop_resolved();
  bool is_shutdown_stop_resolved() const;
  void reset_status_cache();
  void register_instance();
  void unregister_instance();
  bool acquire_status_callback_slot();
  void release_status_callback_slot();
  static void invoke_status_callback_slot_for_test(size_t slot, RobotStatus status);
  static void dispatch_status_callback_slot(size_t slot, const RobotStatus & status);
  static void status_callback_slot_0(RobotStatus status);
  static void status_callback_slot_1(RobotStatus status);
  static void status_callback_slot_2(RobotStatus status);
  static void status_callback_slot_3(RobotStatus status);

  void on_get_connection_status(
    const std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Request> request,
    std::shared_ptr<estun_msgs::srv::GetConnectionStatus::Response> response);
  void on_get_robot_conn_status(
    const std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Request> request,
    std::shared_ptr<estun_msgs::srv::GetRobotConnStatus::Response> response);
  void on_get_cur_err_msg(
    const std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Request> request,
    std::shared_ptr<estun_msgs::srv::GetCurErrMsg::Response> response);
  void on_get_world_cpos(
    const std::shared_ptr<estun_msgs::srv::GetWorldCpos::Request> request,
    std::shared_ptr<estun_msgs::srv::GetWorldCpos::Response> response);
  void on_get_joint_value(
    const std::shared_ptr<estun_msgs::srv::GetJointValue::Request> request,
    std::shared_ptr<estun_msgs::srv::GetJointValue::Response> response);
  void on_set_do(
    const std::shared_ptr<estun_msgs::srv::SetDo::Request> request,
    std::shared_ptr<estun_msgs::srv::SetDo::Response> response);
  void on_get_do(
    const std::shared_ptr<estun_msgs::srv::GetDo::Request> request,
    std::shared_ptr<estun_msgs::srv::GetDo::Response> response);
  void on_get_tool(
    const std::shared_ptr<estun_msgs::srv::GetTool::Request> request,
    std::shared_ptr<estun_msgs::srv::GetTool::Response> response);
  void on_get_user(
    const std::shared_ptr<estun_msgs::srv::GetUser::Request> request,
    std::shared_ptr<estun_msgs::srv::GetUser::Response> response);
  std::unique_lock<std::mutex> acquire_service_sdk_lock();

  // SDK 管理实例与时间戳
  ERIParamManager eri_manager_;
  // 只保护实时 servo 与生命周期类 SDK 调用；这些路径之间必须串行。
  std::mutex sdk_mutex_;
  // 只保护 service bridge 内部 SDK 调用串行，不与实时 servo 互斥。
  // 并发假设：当前 service bridge 的 set/get 类 SDK 接口已和 SDK 开发确认，
  // 可与 servoToAPOS/servoToCPOS 并发，不共享会影响实时流的内部执行状态。
  std::mutex sdk_service_mutex_;
  uint64_t time_stamp_;
  
  // 内部状态与指令缓存
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::array<double, 6> cartesian_commands_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> cartesian_states_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, static_cast<size_t>(EstunStatusStateIndex::COUNT)> estun_status_states_{};

  // 生命周期与控制态标志位。
  bool is_configured_ = false;     // 通信链路与状态回调是否已准备好
  bool is_control_active_ = false; // 是否处于实时控制模式
  std::atomic<bool> stop_in_progress_{false};
  std::atomic<bool> stop_completed_{false};
  std::mutex stop_mutex_;
  ShutdownStopState shutdown_stop_state_;
  CommandMode active_command_mode_{CommandMode::JOINT_POSITION};
  CommandMode pending_command_mode_{CommandMode::JOINT_POSITION};
  bool mode_switch_pending_{false};
  EstunStatusCache status_cache_;
  bool instance_registered_{false};
  int callback_slot_index_{-1};

  std::array<double, 6> last_sent_joint_cmd_deg_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_sent_cart_cmd_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_streamed_joint_cmd_deg_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_streamed_cart_cmd_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::atomic<uint64_t> stat_queue_push_count_{0};
  std::atomic<uint64_t> stat_queue_pop_success_count_{0};
  std::atomic<uint64_t> stat_queue_full_drop_count_{0};
  std::atomic<uint64_t> stat_queue_empty_hold_count_{0};
  std::atomic<uint64_t> stat_queue_underflow_count_{0};
  std::atomic<uint64_t> stat_repeated_send_count_{0};
  std::atomic<size_t> stat_queue_current_depth_{0};
  std::atomic<size_t> stat_queue_max_depth_{0};
  std::atomic<uint64_t> stat_servo_call_count_{0};
  std::atomic<uint64_t> stat_servo_block_ns_total_{0};
  std::atomic<uint64_t> stat_servo_block_ns_max_{0};
  std::atomic<uint64_t> stat_servo_sdk_fail_count_{0};

  // SDK service bridge（独立于 write 实时循环）
  EstunServiceBridge service_bridge_;

  using StreamPolicy = EstunStreamPolicy;
  using Config = EstunHardwareConfig;

  // 存储从 Xacro 注入的部署参数
  Config cfg_;

  std::string cartesian_interface_name_{"cartesian_tcp"};
  std::string estun_status_interface_name_{"estun_status"};
  CommandMode configured_servo_mode_{CommandMode::JOINT_POSITION};
  std::unique_ptr<estun_libs::EstunServoStreamEngine> stream_engine_;
  EstunServoTrace servo_trace_;
  // 仅用于 ESTUN_TEST_STUB_SDK=1，保证无硬件回归时 service bridge 返回可预测结果。
  std::array<bool, 257> test_stub_do_values_{};
  std::atomic<int64_t> last_sdk_fail_log_time_ms_{0};
  std::atomic<int64_t> last_read_disconnect_log_time_ms_{0};
  std::atomic<int64_t> last_read_alarm_log_time_ms_{0};
  std::atomic<int64_t> last_read_exception_log_time_ms_{0};
  std::atomic<int64_t> last_read_unknown_exception_log_time_ms_{0};
  std::atomic<int64_t> last_cartesian_nan_log_time_ms_{0};
  std::atomic<int64_t> last_joint_nan_log_time_ms_{0};
  std::atomic<uint64_t> prev_underflow_warn_count_{0};
  std::atomic<int64_t> last_underflow_warn_time_ms_{0};
  std::atomic<int64_t> last_drop_log_time_ms_{0};
  std::atomic<int64_t> last_write_exception_log_time_ms_{0};
};
}  // namespace estun_hardware
#endif
