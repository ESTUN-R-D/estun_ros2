#include <gtest/gtest.h>

#include <cmath>

#include "ERIParamManager.h"
#include "estun_hardware/estun_status_cache.hpp"

namespace estun_hardware
{
namespace
{

TEST(EstunStatusCacheTest, UpdateFromRobotStatusMapsJointAndWorldPose)
{
  EstunStatusCache cache;

  RobotStatus status{};
  status.isRobotError = true;
  status.isEndPoint = true;
  status.jointValue[0] = 1.23;
  status.jointValue[5] = -4.56;
  status.worldCpos[0] = 101.0;
  status.worldCpos[3] = 3.5;
  status.worldCpos[4] = -3.4;
  status.worldCpos[5] = 3.3;
  status.worldCpos[7] = 707.0;

  cache.update_from_robot_status(status, 12345);

  EXPECT_DOUBLE_EQ(cache.load_joint_value(0), 1.23);
  EXPECT_DOUBLE_EQ(cache.load_joint_value(5), -4.56);
  EXPECT_DOUBLE_EQ(cache.load_world_pose_value(0), 101.0);
  EXPECT_NEAR(cache.load_world_pose_value(3), 3.3 - 2.0 * M_PI, 1e-9);
  EXPECT_NEAR(cache.load_world_pose_value(4), -3.4 + 2.0 * M_PI, 1e-9);
  EXPECT_NEAR(cache.load_world_pose_value(5), 3.5 - 2.0 * M_PI, 1e-9);
  EXPECT_DOUBLE_EQ(cache.load_world_pose_value(7), 707.0);
  EXPECT_TRUE(cache.robot_error());
  EXPECT_TRUE(cache.latest_is_endpoint());
  EXPECT_TRUE(cache.first_packet_received());
  EXPECT_EQ(cache.status_packet_count(), 1u);
  EXPECT_EQ(cache.last_recv_timestamp_ms(), 12345);
}

TEST(EstunStatusCacheTest, ResetClearsAllLatchedState)
{
  EstunStatusCache cache;

  RobotStatus status{};
  status.isRobotError = true;
  status.isEndPoint = true;
  status.jointValue[0] = 9.87;
  status.worldCpos[0] = 321.0;
  cache.update_from_robot_status(status, 54321);
  cache.set_is_disconnected(true);

  cache.reset();

  EXPECT_DOUBLE_EQ(cache.load_joint_value(0), 0.0);
  EXPECT_DOUBLE_EQ(cache.load_world_pose_value(0), 0.0);
  EXPECT_FALSE(cache.robot_error());
  EXPECT_FALSE(cache.latest_is_endpoint());
  EXPECT_FALSE(cache.first_packet_received());
  EXPECT_EQ(cache.status_packet_count(), 0u);
  EXPECT_EQ(cache.last_recv_timestamp_ms(), 0);
  EXPECT_FALSE(cache.is_disconnected());
}

}  // namespace
}  // namespace estun_hardware
