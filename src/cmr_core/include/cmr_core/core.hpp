#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace cmr {
constexpr double pi = 3.14159265358979323846;
struct Pose2 { double x, y, yaw; };
struct Twist2 { double v, w; };

inline bool finite(Twist2 u) {
  return std::isfinite(u.v) && std::isfinite(u.w);
}

inline Pose2 propagate(Pose2 p, Twist2 u, double dt) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.yaw) || !finite(u) ||
      !std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("nonfinite state/control or invalid dt");
  }
  const double q = 0.5 * u.w * dt;
  const double q2 = q * q;
  const double sinc = std::abs(q) < 1e-6
      ? 1.0 - q2 / 6.0 + q2 * q2 / 120.0
      : std::sin(q) / q;
  const double d = u.v * dt * sinc;
  Pose2 next{p.x + d * std::cos(p.yaw + q),
             p.y + d * std::sin(p.yaw + q),
             std::remainder(p.yaw + 2.0 * q, 2.0 * pi)};
  if (!std::isfinite(next.x) || !std::isfinite(next.y) ||
      !std::isfinite(next.yaw)) {
    throw std::overflow_error("propagation overflow");
  }
  return next;
}

// Result order: left, right. Radius/track in metres; output in rad/s.
inline std::array<double, 2> wheel_rates(Twist2 u, double radius,
                                       double track) {
  if (!finite(u) || !std::isfinite(radius) || radius <= 0.0 ||
      !std::isfinite(track) || track <= 0.0) {
    throw std::invalid_argument("invalid wheel model");
  }
  return {(u.v - track * u.w / 2.0) / radius,
          (u.v + track * u.w / 2.0) / radius};
}

inline bool wheel_speed_ok(Twist2 u, double radius, double track,
                           double max_abs_rate) {
  if (!std::isfinite(max_abs_rate) || max_abs_rate <= 0.0) return false;
  try {
    const auto w = wheel_rates(u, radius, track);
    return std::isfinite(w[0]) && std::isfinite(w[1]) &&
           std::abs(w[0]) <= max_abs_rate &&
           std::abs(w[1]) <= max_abs_rate;
  } catch (const std::invalid_argument&) {
    return false;
  }
}

using Clock = std::chrono::steady_clock;
struct Command {
  Twist2 value;
  std::uint64_t seq;
  Clock::time_point created;
  std::chrono::milliseconds ttl;
};

// Single writer; rejected packets do not consume their sequence number.
class CommandGate {
 public:
  explicit CommandGate(std::chrono::milliseconds max_ttl)
      : max_ttl_(max_ttl) {
    if (max_ttl_.count() <= 0) throw std::invalid_argument("invalid TTL");
  }
  bool accept(const Command& c, Clock::time_point now) {
    if (!finite(c.value) || c.ttl.count() <= 0 || c.ttl > max_ttl_ ||
        now < c.created || now - c.created >= c.ttl ||
        (last_seq_ && c.seq <= *last_seq_)) return false;
    last_seq_ = c.seq;
    return true;
  }
 private:
  std::chrono::milliseconds max_ttl_;
  std::optional<std::uint64_t> last_seq_;
};
}  // namespace cmr
