#pragma once

#include <optional>

#include <cmr_interfaces/msg/p2_context.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmr_tracking/context.hpp>

namespace cmr::tracking_ros {

struct WheelImuModel {
  double wheel_vx_variance;
  double imu_wz_variance;
  double imu_wz_bias;
};

std::optional<nav_msgs::msg::Odometry> wheel_odom(
  const nav_msgs::msg::Odometry & raw, const WheelImuModel & model);
std::optional<sensor_msgs::msg::Imu> angular_imu(
  const sensor_msgs::msg::Imu & raw, const WheelImuModel & model,
  const geometry_msgs::msg::Quaternion & base_from_imu);
bool valid_scan(const sensor_msgs::msg::LaserScan & scan);
cmr_interfaces::msg::P2Context to_message(const tracking::P2Context & context);

}  // namespace cmr::tracking_ros
