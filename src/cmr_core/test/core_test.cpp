#include "cmr_core/core.hpp"
#include <iostream>
#include <limits>
#include <string>

namespace {
void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
bool near(double a, double b, double tol = 1e-10) {
  return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= tol;
}
void pose_near(cmr::Pose2 p, cmr::Pose2 expected, const char* message) {
  check(near(p.x, expected.x) && near(p.y, expected.y) &&
        near(p.yaw, expected.yaw), message);
}
template<class Exception, class F>
void throws(F f, const char* message) {
  try { f(); } catch (const Exception&) { return; }
  throw std::runtime_error(message);
}
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double inf = std::numeric_limits<double>::infinity();

void straight() {
  pose_near(cmr::propagate({1, 2, cmr::pi/2}, {0.5, 0}, 2),
            {1, 3, cmr::pi/2}, "straight analytic endpoint");
  pose_near(cmr::propagate({1, 2, 0}, {-0.5, 0}, 2),
            {0, 2, 0}, "reverse straight endpoint");
  pose_near(cmr::propagate({1, 2, 0.4}, {2, 3}, 0),
            {1, 2, 0.4}, "zero duration identity");
  throws<std::invalid_argument>([]{ cmr::propagate({0,0,0}, {1,0}, -1); },
                                "negative dt must throw");
  for (double dt : {nan, inf})
    throws<std::invalid_argument>([=]{ cmr::propagate({0,0,0}, {1,0}, dt); },
                                  "nonfinite dt must throw");
  for (auto p : {cmr::Pose2{nan,0,0}, cmr::Pose2{0,nan,0},
                 cmr::Pose2{0,0,nan}, cmr::Pose2{inf,0,0}})
    throws<std::invalid_argument>([=]{ cmr::propagate(p, {1,0}, 1); },
                                  "nonfinite pose must throw");
  for (auto u : {cmr::Twist2{nan,0}, cmr::Twist2{0,inf}})
    throws<std::invalid_argument>([=]{ cmr::propagate({0,0,0}, u, 1); },
                                  "nonfinite control must throw");
  throws<std::overflow_error>([]{ cmr::propagate({0,0,0},
      {std::numeric_limits<double>::max(),0}, 2); }, "overflow must throw");
}
void arc() {
  pose_near(cmr::propagate({0,0,0}, {1,1}, cmr::pi/2),
            {1,1,cmr::pi/2}, "unit quarter circle");
  pose_near(cmr::propagate({0,0,0}, {1,-1}, cmr::pi/2),
            {1,-1,-cmr::pi/2}, "clockwise unit quarter circle");
  pose_near(cmr::propagate({0,0,0}, {-1,1}, cmr::pi/2),
            {-1,-1,cmr::pi/2}, "reverse quarter circle");
  pose_near(cmr::propagate({2,3,0}, {0,1}, cmr::pi/2),
            {2,3,cmr::pi/2}, "pure rotation fixes position");
  pose_near(cmr::propagate({0,0,0}, {1,1}, 2*cmr::pi),
            {0,0,0}, "full circle closes and normalizes heading");
  const cmr::Pose2 start{2,-3,0.3};
  const auto forward = cmr::propagate(start, {0.7,0.8}, 1.2);
  // Reverse the physical motion with positive elapsed time, not negative dt.
  pose_near(cmr::propagate(forward, {-0.7,-0.8}, 1.2), start,
            "forward then reverse is identity");
}
void near_zero() {
  for (double w : {0.0, 1e-12, -1e-12}) {
    const auto p=cmr::propagate({0,0,0}, {1,w}, 1);
    check(near(p.x,1) && std::abs(p.y)<1e-10 && near(p.yaw,w),
          "near-zero rate has straight limit");
  }
  // Circle geometry: at total angle a, |y| <= a/2 and |x-1| <= a*a/6.
  // This bounds both sides of the numerical branch without duplicating sinc.
  for (double w : {1.999e-6, 2.001e-6, -1.999e-6, -2.001e-6}) {
    const auto p=cmr::propagate({0,0,0}, {1,w}, 1);
    check(std::abs(p.x-1)<=w*w/6+1e-15 &&
          std::abs(p.y)<=std::abs(w)/2+1e-15 && p.y*w>0 && near(p.yaw,w),
          "small-angle geometric bounds and turning direction");
  }
}
void wheels() {
  auto w=cmr::wheel_rates({0.5,2},0.1,0.4);
  check(near(w[0],1) && near(w[1],9), "left/right coupled wheel rates");
  w=cmr::wheel_rates({0,2},0.1,0.4);
  check(near(w[0],-4) && near(w[1],4), "pure spin opposite wheel directions");
  w=cmr::wheel_rates({-0.5,0},0.1,0.4);
  check(near(w[0],-5) && near(w[1],-5), "reverse both wheels");
  check(cmr::wheel_speed_ok({0.5,2},0.1,0.4,9), "inclusive wheel limit");
  check(!cmr::wheel_speed_ok({0.5,2},0.1,0.4,8), "wheel saturation");
  check(cmr::wheel_speed_ok({0.5,0},0.1,0.4,8), "valid straight speed");
  for (double bad : {0.0,-1.0,nan,inf}) {
    throws<std::invalid_argument>([=]{ cmr::wheel_rates({1,0},bad,0.4); },
                                  "invalid radius throws");
    throws<std::invalid_argument>([=]{ cmr::wheel_rates({1,0},0.1,bad); },
                                  "invalid track throws");
    check(!cmr::wheel_speed_ok({1,0},bad,0.4,8), "invalid radius rejects");
    check(!cmr::wheel_speed_ok({1,0},0.1,bad,8), "invalid track rejects");
    check(!cmr::wheel_speed_ok({1,0},0.1,0.4,bad), "invalid speed limit rejects");
  }
  check(!cmr::wheel_speed_ok({nan,0},0.1,0.4,8), "NaN command rejects");
  check(!cmr::wheel_speed_ok({std::numeric_limits<double>::max(),0},
        0.1,0.4,8), "overflowed rate rejects");
}
void gate() {
  using namespace std::chrono_literals;
  const auto t0=cmr::Clock::time_point{};
  throws<std::invalid_argument>([]{ cmr::CommandGate g(0ms); }, "zero max TTL throws");
  throws<std::invalid_argument>([]{ cmr::CommandGate g(-1ms); }, "negative max TTL throws");
  cmr::CommandGate gate(250ms);
  cmr::Command c{{0.2,0},0,t0,100ms};
  check(gate.accept(c,t0), "sequence zero and creation instant valid");
  c.seq=1;
  check(gate.accept(c,t0+99ms), "before expiry valid");
  check(!gate.accept(c,t0+99ms), "duplicate rejected");
  c.seq=0;
  check(!gate.accept(c,t0+1ms), "out of order rejected");
  c.seq=2;
  check(!gate.accept(c,t0+100ms), "expiry boundary rejected");
  check(!gate.accept(c,t0+101ms), "after expiry rejected");
  check(!gate.accept(c,t0-1ms), "future timestamp rejected");
  check(gate.accept(c,t0+1ms), "time rejection does not consume sequence");
  c.seq=3;
  for (auto ttl : {0ms,-1ms,251ms}) {
    c.ttl=ttl;
    check(!gate.accept(c,t0), "zero negative or overlong TTL rejected");
  }
  c.ttl=250ms;
  check(gate.accept(c,t0+249ms), "max TTL valid; rejected seq reusable");
  c.seq=4; c.created=t0+300ms; c.ttl=100ms;
  c.value.v=nan;
  check(!gate.accept(c,t0+301ms), "NaN rejected");
  c.value.v=0.2; c.value.w=inf;
  check(!gate.accept(c,t0+301ms), "infinite yaw rate rejected");
  c.value.w=0;
  check(gate.accept(c,t0+301ms), "nonfinite rejection does not consume sequence");
}
}  // namespace
int main(int argc, char** argv) {
  try {
    check(argc==2, "pass one test name");
    const std::string name=argv[1];
    if (name=="straight") straight();
    else if (name=="arc") arc();
    else if (name=="near_zero") near_zero();
    else if (name=="wheels") wheels();
    else if (name=="gate") gate();
    else throw std::runtime_error("unknown test");
    std::cout << name << ": PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
