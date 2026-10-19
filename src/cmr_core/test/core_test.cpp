#include "cmr_core/core.hpp"
#include <iostream>
#include <limits>
#include <string>

void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
bool near(double a, double b, double tol = 1e-10) {
  return std::abs(a - b) <= tol;
}

int main(int argc, char** argv) {
  try {
    check(argc == 2, "pass one test name");
    const std::string name = argv[1];
    if (name == "straight") {
      const auto p = cmr::propagate({1, 2, cmr::pi / 2}, {0.5, 0}, 2);
      check(near(p.x, 1) && near(p.y, 3), "straight analytic endpoint");
    } else if (name == "arc") {
      const auto p = cmr::propagate({0, 0, 0}, {1, 1}, cmr::pi / 2);
      check(near(p.x, 1) && near(p.y, 1), "unit quarter circle");
      check(near(p.yaw, cmr::pi / 2), "quarter circle heading");
    } else if (name == "near_zero") {
      const auto p = cmr::propagate({0, 0, 0}, {1, 1e-12}, 1);
      check(near(p.x, 1) && std::abs(p.y) < 1e-10, "zero-rate limit");
    } else if (name == "wheels") {
      const auto w = cmr::wheel_rates({0.5, 2}, 0.1, 0.4);
      check(near(w[0], 1) && near(w[1], 9), "coupled wheel rates");
      check(!cmr::wheel_speed_ok({0.5, 2}, 0.1, 0.4, 8), "wheel saturation");
      check(cmr::wheel_speed_ok({0.5, 0}, 0.1, 0.4, 8), "valid straight speed");
      check(!cmr::wheel_speed_ok({0.5, 0}, 0, 0.4, 8), "invalid radius");
    } else if (name == "gate") {
      using namespace std::chrono_literals;
      const auto t0 = cmr::Clock::time_point{};
      cmr::CommandGate gate(250ms);
      cmr::Command c{{0.2, 0}, 1, t0, 100ms};
      check(gate.accept(c, t0 + 99ms), "valid command");
      check(!gate.accept(c, t0 + 99ms), "duplicate rejected");
      c.seq = 2;
      check(!gate.accept(c, t0 + 100ms), "expiry boundary rejected");
      check(!gate.accept(c, t0 - 1ms), "future timestamp rejected");
      c.created = t0 + 100ms;
      c.value.v = std::numeric_limits<double>::quiet_NaN();
      check(!gate.accept(c, t0 + 101ms), "NaN rejected");
      c.value.v = 0.2;
      check(gate.accept(c, t0 + 101ms), "rejected seq not consumed");
    } else {
      throw std::runtime_error("unknown test");
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}