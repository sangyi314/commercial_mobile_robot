#pragma once
#include <cmath>
#include <cstdint>
#include <optional>

namespace cmr {
enum class Decision {
  accept, invalid_limits, nonfinite, limit_violation,
  wrong_epoch, replay, clock_error, source_expired, receive_expired
};
struct CommandSample {
  double v{}, w{};
  std::int64_t source_ros_ns{}, first_receive_steady_ns{};
  std::uint64_t epoch{}, sequence{};
};
struct Context {
  std::int64_t now_ros_ns{}, now_steady_ns{};
  std::uint64_t epoch{};
  std::optional<std::uint64_t> last_accepted_sequence{};
};
struct Limits {
  double max_abs_v{0.2}, max_abs_w{0.6};
  std::int64_t max_source_age_ns{200000000};
  std::int64_t max_receive_age_ns{100000000};
};
inline Decision evaluate(const CommandSample& s, const Context& c,
                         const Limits& l) noexcept {
  if (!std::isfinite(l.max_abs_v) || !std::isfinite(l.max_abs_w) ||
      l.max_abs_v <= 0 || l.max_abs_w <= 0 ||
      l.max_source_age_ns <= 0 || l.max_receive_age_ns <= 0)
    return Decision::invalid_limits;
  if (!std::isfinite(s.v) || !std::isfinite(s.w))
    return Decision::nonfinite;
  if (std::abs(s.v) > l.max_abs_v || std::abs(s.w) > l.max_abs_w)
    return Decision::limit_violation;
  if (s.epoch != c.epoch) return Decision::wrong_epoch;
  if (c.last_accepted_sequence && s.sequence <= *c.last_accepted_sequence)
    return Decision::replay;
  // Nonnegative local counters; compare before subtracting to avoid underflow.
  if (s.source_ros_ns < 0 || s.first_receive_steady_ns < 0 ||
      c.now_ros_ns < s.source_ros_ns ||
      c.now_steady_ns < s.first_receive_steady_ns)
    return Decision::clock_error;
  if (c.now_ros_ns - s.source_ros_ns > l.max_source_age_ns)
    return Decision::source_expired;
  if (c.now_steady_ns - s.first_receive_steady_ns > l.max_receive_age_ns)
    return Decision::receive_expired;
  return Decision::accept;
}
}  
