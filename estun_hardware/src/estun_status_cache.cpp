#include "estun_hardware/estun_status_cache.hpp"

#include <cmath>

#include "ERIParamManager.h"

namespace estun_hardware
{
namespace
{

double normalize_rad(double value)
{
  while (value > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

}  // namespace

EstunStatusCache::EstunStatusCache()
{
  reset();
}

void EstunStatusCache::reset()
{
  for (auto & value : latest_joint_values_) {
    value.store(0.0, std::memory_order_relaxed);
  }
  for (auto & value : latest_world_pose_) {
    value.store(0.0, std::memory_order_relaxed);
  }
  robot_error_.store(false, std::memory_order_relaxed);
  first_packet_received_.store(false, std::memory_order_relaxed);
  status_packet_count_.store(0, std::memory_order_relaxed);
  latest_is_endpoint_.store(false, std::memory_order_relaxed);
  last_recv_timestamp_ms_.store(0, std::memory_order_relaxed);
  is_disconnected_.store(false, std::memory_order_relaxed);
}

void EstunStatusCache::update_from_robot_status(
  const RobotStatus & status,
  int64_t recv_timestamp_ms)
{
  last_recv_timestamp_ms_.store(recv_timestamp_ms, std::memory_order_relaxed);
  for (size_t i = 0; i < latest_joint_values_.size(); ++i) {
    latest_joint_values_[i].store(status.jointValue[i], std::memory_order_relaxed);
  }

  // SDK 回调是 [x,y,z,roll,pitch,yaw]，这里统一映射为 [x,y,z,yaw,pitch,roll]。
  latest_world_pose_[0].store(status.worldCpos[0], std::memory_order_relaxed);
  latest_world_pose_[1].store(status.worldCpos[1], std::memory_order_relaxed);
  latest_world_pose_[2].store(status.worldCpos[2], std::memory_order_relaxed);
  latest_world_pose_[3].store(normalize_rad(status.worldCpos[5]), std::memory_order_relaxed);
  latest_world_pose_[4].store(normalize_rad(status.worldCpos[4]), std::memory_order_relaxed);
  latest_world_pose_[5].store(normalize_rad(status.worldCpos[3]), std::memory_order_relaxed);
  for (size_t i = 6; i < latest_world_pose_.size(); ++i) {
    latest_world_pose_[i].store(status.worldCpos[i], std::memory_order_relaxed);
  }

  robot_error_.store(status.isRobotError, std::memory_order_relaxed);
  latest_is_endpoint_.store(status.isEndPoint, std::memory_order_relaxed);
  first_packet_received_.store(true, std::memory_order_relaxed);
  status_packet_count_.fetch_add(1, std::memory_order_relaxed);
}

std::array<double, 16> EstunStatusCache::load_joint_values() const
{
  std::array<double, 16> values{};
  for (size_t i = 0; i < values.size(); ++i) {
    values[i] = latest_joint_values_[i].load(std::memory_order_relaxed);
  }
  return values;
}

std::array<double, 16> EstunStatusCache::load_world_pose() const
{
  std::array<double, 16> values{};
  for (size_t i = 0; i < values.size(); ++i) {
    values[i] = latest_world_pose_[i].load(std::memory_order_relaxed);
  }
  return values;
}

double EstunStatusCache::load_joint_value(size_t index) const
{
  return latest_joint_values_[index].load(std::memory_order_relaxed);
}

double EstunStatusCache::load_world_pose_value(size_t index) const
{
  return latest_world_pose_[index].load(std::memory_order_relaxed);
}

bool EstunStatusCache::robot_error() const
{
  return robot_error_.load(std::memory_order_relaxed);
}

void EstunStatusCache::set_robot_error(bool value)
{
  robot_error_.store(value, std::memory_order_relaxed);
}

bool EstunStatusCache::first_packet_received() const
{
  return first_packet_received_.load(std::memory_order_relaxed);
}

void EstunStatusCache::set_first_packet_received(bool value)
{
  first_packet_received_.store(value, std::memory_order_relaxed);
}

uint64_t EstunStatusCache::status_packet_count() const
{
  return status_packet_count_.load(std::memory_order_relaxed);
}

bool EstunStatusCache::latest_is_endpoint() const
{
  return latest_is_endpoint_.load(std::memory_order_acquire);
}

void EstunStatusCache::set_latest_is_endpoint(bool value)
{
  latest_is_endpoint_.store(value, std::memory_order_relaxed);
}

int64_t EstunStatusCache::last_recv_timestamp_ms() const
{
  return last_recv_timestamp_ms_.load(std::memory_order_relaxed);
}

void EstunStatusCache::set_last_recv_timestamp_ms(int64_t value)
{
  last_recv_timestamp_ms_.store(value, std::memory_order_relaxed);
}

bool EstunStatusCache::is_disconnected() const
{
  return is_disconnected_.load(std::memory_order_relaxed);
}

void EstunStatusCache::set_is_disconnected(bool value)
{
  is_disconnected_.store(value, std::memory_order_relaxed);
}

}  // namespace estun_hardware
