#include "cmr_tracking_ros/input_adapter.hpp"

#include <gtest/gtest.h>

using namespace cmr::tracking_ros;

TEST(InputAdapter, WheelRequiresFrameAndExplicitCovarianceModel)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.header.stamp.sec = 42;
  odom.twist.twist.linear.x = 0.16;
  const auto result = wheel_odom(odom, {0.001, 0.0001, 0.0});
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header, odom.header);
  EXPECT_DOUBLE_EQ(result->twist.twist.linear.x, 0.16);
  EXPECT_DOUBLE_EQ(result->twist.covariance[0], 0.001);
  EXPECT_DOUBLE_EQ(odom.twist.covariance[0], 0.0);
  odom.child_frame_id = "odom";
  EXPECT_FALSE(wheel_odom(odom, {0.001, 0.0001, 0.0}));
}

TEST(InputAdapter, ImuConvertsOnlyAvailableAngularVelocity)
{
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "imu_link";
  imu.angular_velocity.z = 0.4;
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  const auto result = angular_imu(imu, {0.001, 0.0001, 0.1}, identity);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "base_footprint");
  EXPECT_NEAR(result->angular_velocity.z, 0.3, 1e-10);
  EXPECT_DOUBLE_EQ(result->angular_velocity_covariance[8], 0.0001);
  EXPECT_DOUBLE_EQ(result->orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(result->linear_acceleration_covariance[0], -1.0);
  imu.angular_velocity_covariance[0] = -1.0;
  EXPECT_FALSE(angular_imu(imu, {0.001, 0.0001, 0.1}, identity));
}

TEST(InputAdapter, ContextMappingPreservesIdentity)
{
  cmr::tracking::P2Context context;
  context.context_id = "context-A";
  context.clock_epoch = "clock-Z";
  context.gate_epoch = "gate-Y";
  context.odom_epoch = "odom-X";
  context.sequence = 17;
  const auto message = to_message(context);
  EXPECT_EQ(message.schema_version, "cmr.p2/1.1");
  EXPECT_EQ(message.context_id, context.context_id);
  EXPECT_EQ(message.clock_epoch, context.clock_epoch);
  EXPECT_EQ(message.gate_epoch, context.gate_epoch);
  EXPECT_EQ(message.odom_epoch, context.odom_epoch);
  EXPECT_EQ(message.sequence, 17U);
  EXPECT_EQ(message.health, cmr_interfaces::msg::P2Context::INVALID);
}
