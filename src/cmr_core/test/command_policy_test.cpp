#include <gtest/gtest.h>
#include <limits>
#include "cmr_core/command_policy.hpp"

using namespace cmr;
struct PolicyTest : ::testing::Test {
  CommandSample s{0.1, 0.2, 1000000000, 2000000000, 7, 11};
  Context c{1100000000, 2050000000, 7, 10};
  Limits l{};
};
TEST_F(PolicyTest, ValidSampleIsAccepted) {
  EXPECT_EQ(evaluate(s,c,l), Decision::accept);
}
TEST_F(PolicyTest, SourceDeadlineBoundaryIsInclusive) {
  c.now_ros_ns = 1200000000;
  EXPECT_EQ(evaluate(s,c,l), Decision::accept);
  ++c.now_ros_ns;
  EXPECT_EQ(evaluate(s,c,l), Decision::source_expired);
}
TEST_F(PolicyTest, SteadyDeadlineWorksWhileRosTimeIsPaused) {
  c.now_ros_ns = s.source_ros_ns;
  c.now_steady_ns = 2100000001;
  EXPECT_EQ(evaluate(s,c,l), Decision::receive_expired);
}
TEST_F(PolicyTest, RejectsNanAndInfinity) {
  s.v = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(evaluate(s,c,l), Decision::nonfinite);
  s.v = 0.1; s.w = std::numeric_limits<double>::infinity();
  EXPECT_EQ(evaluate(s,c,l), Decision::nonfinite);
}
TEST_F(PolicyTest, RejectsReplayAndOutOfOrder) {
  s.sequence = 10;
  EXPECT_EQ(evaluate(s,c,l), Decision::replay);
  s.sequence = 9;
  EXPECT_EQ(evaluate(s,c,l), Decision::replay);
}
TEST_F(PolicyTest, RejectsOldEpoch) {
  s.epoch = 6;
  EXPECT_EQ(evaluate(s,c,l), Decision::wrong_epoch);
}
TEST_F(PolicyTest, RejectsFutureSourceAndLocalClockRollback) {
  s.source_ros_ns = c.now_ros_ns + 1;
  EXPECT_EQ(evaluate(s,c,l), Decision::clock_error);
  s.source_ros_ns = 1000000000;
  c.now_steady_ns = s.first_receive_steady_ns - 1;
  EXPECT_EQ(evaluate(s,c,l), Decision::clock_error);
}
TEST_F(PolicyTest, ChecksLimitsWithoutClamping) {
  s.v = -0.2;
  EXPECT_EQ(evaluate(s,c,l), Decision::accept);
  s.v = -0.2001;
  EXPECT_EQ(evaluate(s,c,l), Decision::limit_violation);
}
TEST_F(PolicyTest, RejectsInvalidConfiguration) {
  l.max_source_age_ns = 0;
  EXPECT_EQ(evaluate(s,c,l), Decision::invalid_limits);
}
TEST_F(PolicyTest, FirstSampleNeedsNoPreviousSequence) {
  c.last_accepted_sequence.reset(); s.sequence = 0;
  EXPECT_EQ(evaluate(s,c,l), Decision::accept);
}
TEST_F(PolicyTest, PolicyDoesNotMutateSequenceOwnership) {
  EXPECT_EQ(evaluate(s,c,l), Decision::accept);
  EXPECT_EQ(c.last_accepted_sequence.value(), 10u);
  c.last_accepted_sequence = s.sequence;  // Caller commits accepted ownership.
  EXPECT_EQ(evaluate(s,c,l), Decision::replay);
}
