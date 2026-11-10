#include "cmr_tracking/covariance.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>

namespace cmr::tracking {

CovarianceStatus covariance_status(const Eigen::MatrixXd & covariance, bool imu_sentinel)
{
  if (covariance.rows() == 0 || covariance.rows() != covariance.cols()) {
    return CovarianceStatus::invalid;
  }
  if (imu_sentinel && covariance(0, 0) == -1.0) {
    return CovarianceStatus::unavailable;
  }
  if (!covariance.allFinite()) {
    return CovarianceStatus::invalid;
  }
  const double scale = covariance.cwiseAbs().maxCoeff();
  if (scale == 0.0) {
    return CovarianceStatus::unknown;
  }
  // Scale before symmetrization to avoid overflow even for finite wire values.
  const Eigen::MatrixXd normalized = covariance / scale;
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(
    (normalized + normalized.transpose()) * 0.5, Eigen::EigenvaluesOnly);
  if (spectrum.info() != Eigen::Success || !spectrum.eigenvalues().allFinite()) {
    return CovarianceStatus::invalid;
  }
  const double tolerance = 1e-10 * std::max(
    1.0 / std::max(1.0, scale), spectrum.eigenvalues().cwiseAbs().maxCoeff());
  if ((normalized - normalized.transpose()).cwiseAbs().maxCoeff() > tolerance ||
    spectrum.eigenvalues().minCoeff() < -tolerance)
  {
    return CovarianceStatus::invalid;
  }
  return CovarianceStatus::available;
}

Eigen::Vector3d express_twist_in_parent(
  const Eigen::Quaterniond & parent_from_child, const Eigen::Vector3d & child_velocity)
{
  return parent_from_child * child_velocity;
}

}  // namespace cmr::tracking
