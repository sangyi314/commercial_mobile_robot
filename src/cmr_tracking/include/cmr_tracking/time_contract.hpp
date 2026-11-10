#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace cmr::tracking {

using Nanoseconds = std::int64_t;

constexpr Nanoseconds future_tolerance_ns = 20'000'000;
constexpr Nanoseconds scan_max_age_ns = 300'000'000;
constexpr Nanoseconds odom_max_age_ns = 200'000'000;
constexpr Nanoseconds imu_max_age_ns = 100'000'000;
constexpr Nanoseconds steady_timeout_ns = 300'000'000;
constexpr Nanoseconds recovery_window_ns = 2'000'000'000;
constexpr unsigned recovery_scan_count = 10;

enum class StampStatus {
  accepted, pending, uninitialized, invalid, different_epoch, clock_reset,
  stale, future, duplicate, out_of_order
};

struct StampInput {
  Nanoseconds stamp;
  Nanoseconds now;
  std::string_view sample_epoch;
  std::string_view clock_epoch;
  std::optional<Nanoseconds> previous_stamp;
  std::optional<Nanoseconds> previous_now;
  bool clock_initialized;
};

StampStatus validate_stamp(const StampInput & input, Nanoseconds max_age) noexcept;
std::optional<Nanoseconds> checked_add(Nanoseconds time, Nanoseconds duration) noexcept;
bool before_deadline(Nanoseconds now, Nanoseconds deadline) noexcept;

class ScanRecovery {
public:
  void reset() noexcept;
  void accept_scan(Nanoseconds accepted_steady) noexcept;
  bool ready(Nanoseconds now_steady) const noexcept;

private:
  std::optional<Nanoseconds> start_;
  unsigned count_{0};
};

}  // namespace cmr::tracking
