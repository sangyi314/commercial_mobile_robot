#include "cmr_tracking/context.hpp"
#include "cmr_tracking/covariance.hpp"
#include "cmr_tracking/time_contract.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <set>

using namespace cmr::tracking;

TEST(P201, TimeBoundary)
{
  for (const auto * epoch : {"simulation/1", "offline/1"}) {
    StampInput input{700'000'000, 1'000'000'000, epoch, epoch, {}, {}, true};
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::accepted);
    input.stamp = 699'999'999;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::stale);
    input.stamp = 1'020'000'000;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::pending);
    ++input.stamp;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::future);
    input.stamp = 1'030'000'000;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::future);
    input.sample_epoch = "old/0";
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::different_epoch);
    input.sample_epoch = epoch;
    input.previous_now = input.now + 1;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::clock_reset);
    input.previous_now.reset();
    input.stamp = input.now;
    input.previous_stamp = input.stamp;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::duplicate);
    --input.stamp;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::out_of_order);
    input.clock_initialized = false;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::uninitialized);
    input.clock_initialized = true;
    input.stamp = -1;
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::invalid);
    input.previous_stamp.reset();
    input.stamp = std::numeric_limits<Nanoseconds>::max();
    EXPECT_EQ(validate_stamp(input, scan_max_age_ns), StampStatus::future);
  }
  const auto max = std::numeric_limits<Nanoseconds>::max();
  EXPECT_EQ(checked_add(max - 1, 1), max);
  EXPECT_FALSE(checked_add(max, 1));
  EXPECT_FALSE(checked_add(0, -1));
  EXPECT_TRUE(before_deadline(9, 10));
  EXPECT_FALSE(before_deadline(10, 10));
  ScanRecovery recovery;
  for (int i = 0; i < 9; ++i) {
    recovery.accept_scan(i * 200'000'000LL);
  }
  EXPECT_FALSE(recovery.ready(recovery_window_ns));
  recovery.accept_scan(1'800'000'000LL);
  EXPECT_FALSE(recovery.ready(recovery_window_ns - 1));
  EXPECT_TRUE(recovery.ready(recovery_window_ns));
  recovery.reset();
  EXPECT_FALSE(recovery.ready(recovery_window_ns));
}

TEST(P201, CovarianceAvailability)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  EXPECT_EQ(covariance_status(matrix, true), CovarianceStatus::unknown);
  matrix(0, 0) = -1.0;
  EXPECT_EQ(covariance_status(matrix, true), CovarianceStatus::unavailable);
  EXPECT_EQ(covariance_status(matrix), CovarianceStatus::invalid);
  matrix.setIdentity();
  EXPECT_EQ(covariance_status(matrix), CovarianceStatus::available);
  matrix(1, 1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(covariance_status(matrix), CovarianceStatus::invalid);
  matrix.setIdentity();
  matrix(1, 0) = 0.1;
  EXPECT_EQ(covariance_status(matrix), CovarianceStatus::invalid);
  matrix(0, 1) = matrix(1, 0) = 2.0;
  EXPECT_EQ(covariance_status(matrix), CovarianceStatus::invalid);
  matrix = Eigen::Matrix3d::Identity() * std::numeric_limits<double>::max();
  EXPECT_EQ(covariance_status(matrix), CovarianceStatus::available);
}

TEST(P201, FrameTwist)
{
  const double pi_over_two = std::acos(0.0);
  const Eigen::Vector3d expected[] = {
    Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
    -Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitY()
  };
  for (int i = 0; i < 4; ++i) {
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(i * pi_over_two, Eigen::Vector3d::UnitZ()));
    const auto parent = express_twist_in_parent(rotation, Eigen::Vector3d::UnitX());
    EXPECT_TRUE(parent.isApprox(expected[i], 1e-10));
    EXPECT_TRUE(express_twist_in_parent(rotation.inverse(), parent)
      .isApprox(Eigen::Vector3d::UnitX(), 1e-10));
  }
}

TEST(P201, RandomTimestamp)
{
  for (unsigned seed = 0; seed < 30; ++seed) {
    SCOPED_TRACE(seed);
    std::mt19937 rng(seed);
    const auto max = std::numeric_limits<Nanoseconds>::max();
    const Nanoseconds origin = seed % 2 == 0 ? 0 : max - 1'000'000'000;
    std::set<Nanoseconds> oracle;
    std::optional<Nanoseconds> last;
    for (int event = 0; event < 1000; ++event) {
      const auto stamp = origin + static_cast<Nanoseconds>(rng() % 1000);
      const auto result = validate_stamp(
        {stamp, origin + 1000, "source/1", "source/1", last, {}, true}, scan_max_age_ns);
      StampStatus expected = StampStatus::accepted;
      if (!oracle.empty() && stamp <= *oracle.rbegin()) {
        expected = stamp == *oracle.rbegin() ? StampStatus::duplicate : StampStatus::out_of_order;
      } else {
        oracle.insert(stamp);
      }
      ASSERT_EQ(result, expected) << event;
      if (result == StampStatus::accepted) {
        last = stamp;
      }
    }
  }
}
