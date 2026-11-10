#include "cmr_tracking/time_contract.hpp"

#include <limits>

namespace cmr::tracking {

StampStatus validate_stamp(const StampInput & input, Nanoseconds max_age) noexcept
{
  if (!input.clock_initialized) {
    return StampStatus::uninitialized;
  }
  if (input.stamp < 0 || input.now < 0 || max_age < 0 ||
    input.sample_epoch.empty() || input.clock_epoch.empty() ||
    (input.previous_stamp && *input.previous_stamp < 0) ||
    (input.previous_now && *input.previous_now < 0))
  {
    return StampStatus::invalid;
  }
  if (input.sample_epoch != input.clock_epoch) {
    return StampStatus::different_epoch;
  }
  if (input.previous_now && input.now < *input.previous_now) {
    return StampStatus::clock_reset;
  }
  if (input.previous_stamp && input.stamp <= *input.previous_stamp) {
    return input.stamp == *input.previous_stamp ?
      StampStatus::duplicate : StampStatus::out_of_order;
  }
  if (input.stamp > input.now) {
    return input.stamp - input.now <= future_tolerance_ns ?
      StampStatus::pending : StampStatus::future;
  }
  return input.now - input.stamp <= max_age ? StampStatus::accepted : StampStatus::stale;
}

std::optional<Nanoseconds> checked_add(Nanoseconds time, Nanoseconds duration) noexcept
{
  if (time < 0 || duration < 0 || time > std::numeric_limits<Nanoseconds>::max() - duration) {
    return std::nullopt;
  }
  return time + duration;
}

bool before_deadline(Nanoseconds now, Nanoseconds deadline) noexcept
{
  return now >= 0 && now < deadline;
}

void ScanRecovery::reset() noexcept
{
  start_.reset();
  count_ = 0;
}

void ScanRecovery::accept_scan(Nanoseconds accepted_steady) noexcept
{
  if (accepted_steady < 0) {
    reset();
    return;
  }
  if (!start_) {
    start_ = accepted_steady;
  }
  if (count_ < recovery_scan_count) {
    ++count_;
  }
}

bool ScanRecovery::ready(Nanoseconds now_steady) const noexcept
{
  return start_ && count_ >= recovery_scan_count && now_steady >= *start_ &&
         now_steady - *start_ >= recovery_window_ns;
}

}  // namespace cmr::tracking
