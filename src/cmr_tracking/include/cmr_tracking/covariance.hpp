#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cmr::tracking {

enum class CovarianceStatus { available, unknown, unavailable, invalid };

// The IMU sentinel applies only to message fields with that documented convention.
CovarianceStatus covariance_status(const Eigen::MatrixXd & covariance, bool imu_sentinel = false);
Eigen::Vector3d express_twist_in_parent(
  const Eigen::Quaterniond & parent_from_child, const Eigen::Vector3d & child_velocity);

}  // namespace cmr::tracking
