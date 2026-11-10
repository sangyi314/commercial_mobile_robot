#include "cmr_tracking_ros/input_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <cmr_tracking/covariance.hpp>

namespace cmr::tracking_ros {
namespace {

template<int Size, typename Array>
auto covariance(const Array & data)
{
  return Eigen::Map<const Eigen::Matrix<double, Size, Size, Eigen::RowMajor>>(data.data());
}

bool positive_variance(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

std::optional<nav_msgs::msg::Odometry> wheel_odom(
  const nav_msgs::msg::Odometry & raw, const WheelImuModel & model)
{
  using tracking::CovarianceStatus;
  const auto state = tracking::covariance_status(covariance<6>(raw.twist.covariance));
  if (raw.header.frame_id != "odom" || raw.child_frame_id != "base_footprint" ||
    !std::isfinite(raw.twist.twist.linear.x) || state == CovarianceStatus::invalid ||
    !positive_variance(model.wheel_vx_variance) ||
    (state == CovarianceStatus::available && !positive_variance(raw.twist.covariance[0])))
  {
    return std::nullopt;
  }
  auto result = raw;
  // Preserve raw non-fused fields. Replace only the unknown twist matrix with the
  // calibrated vx variance and an explicit large variance on unused coordinates.
  if (state == CovarianceStatus::unknown) {
    result.twist.covariance.fill(0.0);
    result.twist.covariance[0] = model.wheel_vx_variance;
    for (std::size_t axis = 1; axis < 6; ++axis) {
      result.twist.covariance[axis * 7] = 1e6;
    }
  }
  return result;
}

std::optional<sensor_msgs::msg::Imu> angular_imu(
  const sensor_msgs::msg::Imu & raw, const WheelImuModel & model,
  const geometry_msgs::msg::Quaternion & base_from_imu)
{
  using tracking::CovarianceStatus;
  const auto state = tracking::covariance_status(covariance<3>(raw.angular_velocity_covariance), true);
  const Eigen::Quaterniond rotation(
    base_from_imu.w, base_from_imu.x, base_from_imu.y, base_from_imu.z);
  if (raw.header.frame_id != "imu_link" || state == CovarianceStatus::unavailable ||
    state == CovarianceStatus::invalid || !rotation.coeffs().allFinite() ||
    std::abs(rotation.norm() - 1.0) > 1e-6 ||
    !positive_variance(model.imu_wz_variance) || !std::isfinite(model.imu_wz_bias))
  {
    return std::nullopt;
  }
  const Eigen::Vector3d angular(
    raw.angular_velocity.x, raw.angular_velocity.y, raw.angular_velocity.z - model.imu_wz_bias);
  if (!angular.allFinite()) {
    return std::nullopt;
  }
  Eigen::Matrix3d source_cov = covariance<3>(raw.angular_velocity_covariance);
  if (state == CovarianceStatus::unknown) {
    // The fallback model qualifies only source z. Other axes cannot be rotated
    // into base z without their own calibration.
    if (!rotation.toRotationMatrix().row(2).transpose().isApprox(Eigen::Vector3d::UnitZ(), 1e-10)) {
      return std::nullopt;
    }
    source_cov = Eigen::Vector3d(1e6, 1e6, model.imu_wz_variance).asDiagonal();
  }
  const auto r = rotation.toRotationMatrix();
  const Eigen::Matrix3d transformed_cov = r * source_cov * r.transpose();
  const auto base_angular = rotation * angular;
  if (!transformed_cov.allFinite() || !positive_variance(transformed_cov(2, 2)) ||
    !base_angular.allFinite())
  {
    return std::nullopt;
  }
  sensor_msgs::msg::Imu result;
  result.header = raw.header;
  result.header.frame_id = "base_footprint";
  result.angular_velocity.x = base_angular.x();
  result.angular_velocity.y = base_angular.y();
  result.angular_velocity.z = base_angular.z();
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
    result.angular_velocity_covariance.data()) = transformed_cov;
  result.orientation_covariance[0] = -1.0;
  result.linear_acceleration_covariance[0] = -1.0;
  return result;
}

bool valid_scan(const sensor_msgs::msg::LaserScan & scan)
{
  if (scan.header.frame_id != "base_scan" || scan.ranges.empty() || scan.ranges.size() > 4096 ||
    !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max) ||
    !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0 ||
    scan.angle_max < scan.angle_min || !std::isfinite(scan.range_min) ||
    !std::isfinite(scan.range_max) || scan.range_min < 0 || scan.range_min >= scan.range_max ||
    !std::isfinite(scan.time_increment) || scan.time_increment < 0 ||
    !std::isfinite(scan.scan_time) || scan.scan_time < 0)
  {
    return false;
  }
  return std::none_of(scan.ranges.begin(), scan.ranges.end(), [](float value) {
    return std::isnan(value);
  });
}

cmr_interfaces::msg::P2Context to_message(const tracking::P2Context & context)
{
  cmr_interfaces::msg::P2Context msg;
  msg.schema_version = context.schema_version;
  msg.context_id = context.context_id;
  msg.authority_session = context.authority_session;
  msg.mode = context.mode;
  msg.run_id = context.run_id;
  msg.gate_epoch = context.gate_epoch;
  msg.clock_epoch = context.clock_epoch;
  msg.odom_epoch = context.odom_epoch;
  msg.localization_version = context.localization_version;
  msg.map_version = context.map_version;
  msg.config_hash = context.config_hash;
  msg.sequence = context.sequence;
  msg.health = static_cast<std::uint8_t>(context.health);
  msg.reason = context.reason;
  return msg;
}

}  // namespace cmr::tracking_ros
