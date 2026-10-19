#include "cmr_core/core.hpp"
#include <iostream>
#include <limits>
#include <string>

// Runtime checks remain active with NDEBUG. Every executed check logs its ID.
void check(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
  std::cout << "[check] " << message << '\n';
}
constexpr double position_tol = 1e-10;  // metres (legacy endpoint tolerance)
constexpr double angle_tol = 1e-12;     // radians
constexpr double roundoff = 16 * std::numeric_limits<double>::epsilon();
bool near(double a, double b, double tol = position_tol) {
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(tol) &&
         tol >= 0 && std::abs(a - b) <= tol;
}
double angle_error(double actual, double expected) {
  return std::remainder(actual - expected, 2 * cmr::pi);
}
void expect_pose(cmr::Pose2 p, cmr::Pose2 expected, const std::string& id) {
  check(near(p.x, expected.x) && near(p.y, expected.y), id + " position");
  check(std::isfinite(p.yaw) &&
        near(angle_error(p.yaw, expected.yaw), 0, angle_tol), id + " heading");
  check(std::isfinite(p.yaw) && p.yaw >= -cmr::pi && p.yaw <= cmr::pi,
        id + " normalized yaw");
}
template <class Expected, class F>
void expect_throws(F action, const std::string& id) {
  try {
    action();
  } catch (const Expected&) {
    check(true, id + " expected exception");
    return;
  } catch (const std::exception& e) {
    throw std::runtime_error(id + " wrong exception: " + e.what());
  } catch (...) {
    throw std::runtime_error(id + " wrong non-standard exception");
  }
  throw std::runtime_error(id + " missing exception");
}
const std::array<double, 3> nonfinite{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
const std::array<const char*, 3> nonfinite_name{"NaN", "+Inf", "-Inf"};

int main(int argc, char** argv) {
  try {
    check(argc == 2, "pass one test name");
    const std::string name = argv[1];
    if (name == "straight") {
      // P01: retain the original northbound endpoint, add heading/range.
      const auto p = cmr::propagate({1, 2, cmr::pi / 2}, {0.5, 0}, 2);
      check(near(p.x, 1) && near(p.y, 3), "P01 straight analytic endpoint");
      expect_pose(p, {1, 3, cmr::pi / 2}, "P01");
      expect_pose(cmr::propagate({1, 2, 0}, {-0.5, 0}, 2), {0, 2, 0}, "P02 reverse");
      const auto zero_dt = cmr::propagate({1, 2, 4 * cmr::pi + 0.25}, {0.5, 2}, 0);
      check(near(zero_dt.x, 1, 0) && near(zero_dt.y, 2, 0), "P03 dt=0 unchanged x/y");
      expect_pose(zero_dt, {1, 2, 0.25}, "P03 dt=0 angle equivalent");
      expect_throws<std::invalid_argument>([] {
        cmr::propagate({0, 0, 0}, {1, 0}, -0.1);
      }, "P04 negative dt");
      for (std::size_t i = 0; i < nonfinite.size(); ++i) {
        expect_throws<std::invalid_argument>([&] {
          cmr::propagate({0, 0, 0}, {1, 0}, nonfinite[i]);
        }, std::string("P05 dt=") + nonfinite_name[i]);
      }
      const std::array<const char*, 5> fields{"x", "y", "yaw", "v", "w"};
      for (std::size_t f = 0; f < fields.size(); ++f) {
        for (std::size_t i = 0; i < nonfinite.size(); ++i) {
          cmr::Pose2 state{1, 2, 0.3};
          cmr::Twist2 control{0.5, 0.2};
          if (f == 0) state.x = nonfinite[i];
          if (f == 1) state.y = nonfinite[i];
          if (f == 2) state.yaw = nonfinite[i];
          if (f == 3) control.v = nonfinite[i];
          if (f == 4) control.w = nonfinite[i];
          expect_throws<std::invalid_argument>([&] {
            cmr::propagate(state, control, 1);
          }, std::string("P06 ") + fields[f] + "=" + nonfinite_name[i]);
        }
      }
      const double largest = std::numeric_limits<double>::max();
      expect_throws<std::overflow_error>([&] {
        cmr::propagate({0, 0, 0}, {largest, 0}, 2);
      }, "P07 finite translation product overflow");
      expect_throws<std::overflow_error>([&] {
        cmr::propagate({largest, 0, 0}, {largest, 0}, 1);
      }, "P07 finite position sum overflow");
      expect_throws<std::overflow_error>([&] {
        cmr::propagate({0, 0, 0}, {0, largest}, 4);
      }, "P07 finite angular product overflow");
    } else if (name == "arc") {
      const auto p = cmr::propagate({0, 0, 0}, {1, 1}, cmr::pi / 2);
      check(near(p.x, 1) && near(p.y, 1), "A01 unit quarter circle");
      check(near(p.yaw, cmr::pi / 2), "A01 quarter circle heading");
      expect_pose(p, {1, 1, cmr::pi / 2}, "A01");
      expect_pose(cmr::propagate({0, 0, 0}, {1, -1}, cmr::pi / 2),
                  {1, -1, -cmr::pi / 2}, "A02 negative turn");
      expect_pose(cmr::propagate({0, 0, 0}, {-1, 1}, cmr::pi / 2),
                  {-1, -1, cmr::pi / 2}, "A03 reverse arc");
      expect_pose(cmr::propagate({2, -3, 0.25}, {0, 0.5}, 2),
                  {2, -3, 1.25}, "A04 pure rotation");
      expect_pose(cmr::propagate({0, 0, 0}, {1, 1}, 2 * cmr::pi),
                  {0, 0, 0}, "A05 full circle");
      expect_pose(cmr::propagate({2, -3, cmr::pi - 0.1}, {0, 0.2}, 1),
                  {2, -3, -cmr::pi + 0.1}, "A05 cross +pi");
      expect_pose(cmr::propagate({2, -3, -cmr::pi + 0.1}, {0, -0.2}, 1),
                  {2, -3, cmr::pi - 0.1}, "A05 cross -pi");
      const cmr::Pose2 start{1.2, -0.7, 2.8};
      const auto forward = cmr::propagate(start, {0.8, 0.6}, 1.3);
      expect_pose(cmr::propagate(forward, {-0.8, -0.6}, 1.3), start,
                  "A06 inverse motion with positive dt");
    } else if (name == "near_zero") {
      const auto p = cmr::propagate({0, 0, 0}, {1, 1e-12}, 1);
      check(near(p.x, 1) && std::isfinite(p.y) && std::abs(p.y) < 1e-10,
            "N01 zero-rate limit (legacy)");
      // Independent geometry: x=integral cos(w*t)dt, y=integral sin(w*t)dt.
      // At dt=v=1: 0<=1-x<=w^2/6 and 0<=sign(w)*y<=|w|/2.
      // The roundoff allowance is absolute, dimensionless here (unit path).
      auto small_angle = [](double w, const std::string& id) {
        const auto a = cmr::propagate({0, 0, 0}, {1, w}, 1);
        check(std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.yaw),
              id + " finite");
        check(1 - a.x >= -roundoff && 1 - a.x <= w * w / 6 + roundoff,
              id + " x geometric bound");
        if (w == 0) {
          check(near(a.y, 0, roundoff), id + " straight y");
        } else {
          const double signed_y = w > 0 ? a.y : -a.y;
          check(signed_y > 0 && signed_y <= std::abs(w) / 2 + roundoff,
                id + " y sign and geometric bound");
        }
        check(near(angle_error(a.yaw, w), 0, roundoff), id + " heading");
        return a;
      };
      for (double w : {0.0, 1e-12, -1e-12}) {
        small_angle(w, w == 0 ? "N01 w=0" : (w > 0 ? "N01 w=+1e-12" : "N01 w=-1e-12"));
      }
      for (double sign : {1.0, -1.0}) {
        const double lo = sign * 2e-6 * (1 - 1e-3);
        const double hi = sign * 2e-6 * (1 + 1e-3);
        const std::string id = sign > 0 ? "N02 positive" : "N02 negative";
        const auto a = small_angle(lo, id + " below branch");
        const auto b = small_angle(hi, id + " above branch");
        // Integral derivatives give |dx/dw|<=|w|/3 and |dy/dw|<=1/2.
        check(near(a.x, b.x, std::abs(hi) * std::abs(hi - lo) / 3 + roundoff) &&
              near(a.y, b.y, std::abs(hi - lo) / 2 + roundoff), id + " continuity");
      }
    } else if (name == "wheels") {
      const auto w = cmr::wheel_rates({0.5, 2}, 0.1, 0.4);
      check(near(w[0], 1) && near(w[1], 9), "W01 coupled wheel rates left/right");
      check(!cmr::wheel_speed_ok({0.5, 2}, 0.1, 0.4, 8), "W01 wheel saturation");
      check(cmr::wheel_speed_ok({0.5, 0}, 0.1, 0.4, 8), "W01 valid straight speed");
      check(!cmr::wheel_speed_ok({0.5, 0}, 0, 0.4, 8), "W05 invalid radius (legacy)");
      const auto spin = cmr::wheel_rates({0, 2}, 0.1, 0.4);
      check(near(spin[0], -4) && near(spin[1], 4), "W02 spin left/right");
      const auto reverse = cmr::wheel_rates({-0.5, 0}, 0.1, 0.4);
      check(near(reverse[0], -5) && near(reverse[1], -5), "W03 reverse left/right");
      const auto exact = cmr::wheel_rates({1, 0}, 0.5, 1);
      check(near(exact[0], 2, 0) && near(exact[1], 2, 0), "W04 exact rates");
      for (double v : {1.0, -1.0}) {
        const std::string id = v > 0 ? "W04 positive" : "W04 negative";
        check(cmr::wheel_speed_ok({v, 0}, 0.5, 1, 2), id + " equality accepted");
        check(!cmr::wheel_speed_ok({v, 0}, 0.5, 1, std::nextafter(2.0, 0.0)),
              id + " immediately smaller limit rejected");
      }
      const std::array<double, 5> invalid{0, -0.1, nonfinite[0], nonfinite[1], nonfinite[2]};
      const std::array<const char*, 5> labels{"zero", "negative", "NaN", "+Inf", "-Inf"};
      for (std::size_t i = 0; i < invalid.size(); ++i) {
        for (bool radius_bad : {true, false}) {
          const double r = radius_bad ? invalid[i] : 0.1;
          const double b = radius_bad ? 0.4 : invalid[i];
          const std::string id = std::string("W05 ") + (radius_bad ? "radius " : "track ") + labels[i];
          expect_throws<std::invalid_argument>([&] { cmr::wheel_rates({0.5, 0}, r, b); }, id);
          check(!cmr::wheel_speed_ok({0.5, 0}, r, b, 8), id + " checker false");
        }
        check(!cmr::wheel_speed_ok({0.5, 0}, 0.1, 0.4, invalid[i]),
              std::string("W06 invalid limit ") + labels[i]);
      }
      for (bool v_bad : {true, false}) {
        for (std::size_t i = 0; i < nonfinite.size(); ++i) {
          const cmr::Twist2 u = v_bad ? cmr::Twist2{nonfinite[i], 0} : cmr::Twist2{0.5, nonfinite[i]};
          const std::string id = std::string("W07 ") + (v_bad ? "v=" : "w=") + nonfinite_name[i];
          expect_throws<std::invalid_argument>([&] { cmr::wheel_rates(u, 0.1, 0.4); }, id);
          check(!cmr::wheel_speed_ok(u, 0.1, 0.4, 8), id + " checker false");
        }
      }
      const double largest = std::numeric_limits<double>::max();
      // Do not require wheel_rates to return Inf: a future reviewed API may throw.
      check(!cmr::wheel_speed_ok({largest, 0}, 0.5, 1, largest), "W07 division overflow rejected");
      check(!cmr::wheel_speed_ok({0, largest}, 1, 4, largest), "W07 track product overflow rejected");
    } else if (name == "gate") {
      using namespace std::chrono_literals;
      const auto t0 = cmr::Clock::time_point{};
      // Retain the original scenario; seq=2 isolates its expiry from duplicate rejection.
      cmr::CommandGate gate(250ms);
      cmr::Command c{{0.2, 0}, 1, t0, 100ms};
      check(gate.accept(c, t0 + 99ms), "G03 legacy valid command");
      check(!gate.accept(c, t0 + 99ms), "G04 legacy duplicate rejected");
      c.seq = 2;
      check(!gate.accept(c, t0 + 100ms), "G03 legacy expiry boundary rejected");
      check(!gate.accept(c, t0 - 1ms), "G05 legacy future timestamp rejected");
      c.created = t0 + 100ms;
      c.value.v = std::numeric_limits<double>::quiet_NaN();
      check(!gate.accept(c, t0 + 101ms), "G08 legacy NaN rejected");
      c.value.v = 0.2;
      check(gate.accept(c, t0 + 101ms), "G08 legacy rejected seq not consumed");
      for (auto ttl : {0ms, -1ms}) {
        expect_throws<std::invalid_argument>([&] { cmr::CommandGate bad(ttl); },
            ttl == 0ms ? "G01 zero max_ttl" : "G01 negative max_ttl");
      }
      cmr::CommandGate first(250ms);
      check(first.accept({{0.2, 0}, 0, t0, 100ms}, t0), "G02 first seq=0 at created");
      for (auto age : {99ms, 100ms, 101ms}) {
        cmr::CommandGate isolated(250ms);
        const cmr::Command cmd{{0.2, 0}, 7, t0, 100ms};
        const std::string id = "G03 age=" + std::to_string(age.count()) + "ms";
        check(isolated.accept(cmd, t0 + age) == (age < 100ms), id + " independent gate");
        if (age >= 100ms) {
          auto repaired = cmd;
          repaired.created = t0 + age;
          check(isolated.accept(repaired, t0 + age), id + " rejection does not consume seq");
        }
      }
      cmr::CommandGate ordering(250ms);
      check(ordering.accept({{0.2, 0}, 10, t0, 100ms}, t0), "G04 initial seq=10");
      check(!ordering.accept({{0.2, 0}, 10, t0, 100ms}, t0), "G04 duplicate seq=10");
      check(!ordering.accept({{0.2, 0}, 9, t0, 100ms}, t0), "G04 smaller seq=9");
      check(ordering.accept({{0.2, 0}, 11, t0, 100ms}, t0), "G04 larger seq=11");
      cmr::CommandGate future(250ms);
      cmr::Command future_cmd{{0.2, 0}, 8, t0 + 1ms, 100ms};
      check(!future.accept(future_cmd, t0), "G05 only future created rejected");
      future_cmd.created = t0;
      check(future.accept(future_cmd, t0), "G05 repaired same seq accepted");
      for (auto ttl : {0ms, -1ms, 251ms}) {
        cmr::CommandGate isolated(250ms);
        cmr::Command cmd{{0.2, 0}, 8, t0, ttl};
        const std::string id = "G06 ttl=" + std::to_string(ttl.count()) + "ms";
        check(!isolated.accept(cmd, t0), id + " rejected");
        cmd.ttl = 100ms;
        check(isolated.accept(cmd, t0), id + " repaired same seq accepted");
      }
      for (auto age : {0ms, 250ms}) {
        cmr::CommandGate isolated(250ms);
        check(isolated.accept({{0.2, 0}, 8, t0, 250ms}, t0 + age) == (age == 0ms),
              "G07 ttl=max age=" + std::to_string(age.count()) + "ms");
      }
      for (bool v_bad : {true, false}) {
        for (std::size_t i = 0; i < nonfinite.size(); ++i) {
          cmr::CommandGate isolated(250ms);
          cmr::Command cmd{{0.2, 0}, 8, t0, 100ms};
          if (v_bad) cmd.value.v = nonfinite[i]; else cmd.value.w = nonfinite[i];
          const std::string id = std::string("G08 ") + (v_bad ? "v=" : "w=") + nonfinite_name[i];
          check(!isolated.accept(cmd, t0), id + " rejected");
          cmd.value = {0.2, 0};
          check(isolated.accept(cmd, t0), id + " repaired same seq accepted");
        }
      }
      cmr::CommandGate high(250ms);
      check(high.accept({{0.2, 0}, 1, t0, 100ms}, t0), "G09 initial seq=1");
      check(!high.accept({{0.2, 0}, 100, t0, 0ms}, t0), "G09 invalid high seq=100");
      check(high.accept({{0.2, 0}, 2, t0, 100ms}, t0), "G09 lower valid seq=2 after rejection");
      cmr::CommandGate maximum(250ms);
      const auto max_seq = std::numeric_limits<std::uint64_t>::max();
      check(maximum.accept({{0.2, 0}, max_seq, t0, 100ms}, t0), "G10 first maximum seq accepted");
      check(!maximum.accept({{0.2, 0}, max_seq, t0, 100ms}, t0), "G10 duplicate maximum rejected");
      check(!maximum.accept({{0.2, 0}, 0, t0, 100ms}, t0), "G10 no wrap to seq=0");
    } else {
      throw std::runtime_error("unknown test");
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
