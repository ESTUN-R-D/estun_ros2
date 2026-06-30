#ifndef ESTUN_HARDWARE__ESTUN_STATUS_CACHE_HPP_
#define ESTUN_HARDWARE__ESTUN_STATUS_CACHE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "ERIParamManager.h"

namespace estun_hardware
{

class EstunStatusCache
{
public:
  EstunStatusCache();

  void reset();
  void update_from_robot_status(const RobotStatus & status, int64_t recv_timestamp_ms);

  std::array<double, 16> load_joint_values() const;
  std::array<double, 16> load_world_pose() const;
  double load_joint_value(size_t index) const;
  double load_world_pose_value(size_t index) const;

  bool robot_error() const;
  void set_robot_error(bool value);

  bool first_packet_received() const;
  void set_first_packet_received(bool value);

  uint64_t status_packet_count() const;

  bool latest_is_endpoint() const;
  void set_latest_is_endpoint(bool value);

  int64_t last_recv_timestamp_ms() const;
  void set_last_recv_timestamp_ms(int64_t value);

  bool is_disconnected() const;
  void set_is_disconnected(bool value);

private:
  std::array<std::atomic<double>, 16> latest_joint_values_;
  std::array<std::atomic<double>, 16> latest_world_pose_;
  std::atomic<bool> robot_error_{false};
  std::atomic<bool> first_packet_received_{false};
  std::atomic<uint64_t> status_packet_count_{0};
  std::atomic<bool> latest_is_endpoint_{false};
  std::atomic<int64_t> last_recv_timestamp_ms_{0};
  std::atomic<bool> is_disconnected_{false};
};

}  // namespace estun_hardware

#endif  // ESTUN_HARDWARE__ESTUN_STATUS_CACHE_HPP_
