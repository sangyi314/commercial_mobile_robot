// SPDX-License-Identifier: MIT
#include "cmr_mission/action_correlation.hpp"
#include <iostream>
#include <stdexcept>
using namespace cmr::mission;

void ck(bool b) {
  if (!b) {
    throw std::runtime_error("correlation contract");
  }
}

int main() {
  try {
    ck(!deadline_elapsed(129, 130));
    ck(deadline_elapsed(130, 130));
    ck(deadline_elapsed(131, 130));
    Record current;
    current.request = {1, "m", "a", "b", "map", "stations", 1};
    current.epoch = "epoch";
    current.operation = "return";
    current.attempt = "return0";
    current.state = State::Returning;
    Event diagnostic;
    diagnostic.id = "old-fatal";
    diagnostic.mission_id = "m";
    diagnostic.epoch = "epoch";
    diagnostic.operation = "out";
    diagnostic.attempt = "out0";
    diagnostic.kind = Kind::Fatal;
    auto scoped = scope_callback(diagnostic, current);
    ck(scoped.kind == Kind::Feedback);
    ck(!reduce(current, scoped).changed);
    Effect effect{"effect", "m", "out", "out0", "", EffectKind::Send};
    ActionCorrelation c("epoch", effect);
    Event e;
    e.id = "e";
    e.epoch = "epoch";
    e.mission_id = "m";
    e.operation = "out";
    e.attempt = "out0";
    e.uuid = "u";
    e.kind = Kind::Succeeded;
    auto old = e;
    old.epoch = "old";
    ck(c.push(old).empty());
    ck(c.discarded == 1);
    ck(c.push(e).empty());
    e.kind = Kind::Accepted;
    auto ready = c.push(e);
    ck(ready.size() == 2 && ready[0].kind == Kind::Accepted && ready[1].kind == Kind::Succeeded);
    ck(c.push(e).empty());
    e.kind = Kind::Aborted;
    e.id = "contradictory";
    auto conflict = c.push(e);
    ck(conflict.size() == 1 && conflict[0].kind == Kind::Feedback
       && conflict[0].detail.find("CONFLICTING_TERMINAL") != std::string::npos && c.conflicts == 1);
    e.uuid = "wrong";
    ck(c.push(e).empty() && c.fault);
    ActionCorrelation bounded("epoch", effect);
    e.uuid = "u";
    e.kind = Kind::Succeeded;
    bounded.push(e);
    bounded.push(e);
    bounded.push(e);
    ck(bounded.fault);
    std::cout << "PASS adapter correlation old epoch/UUID/reorder/duplicate/conflict/bound\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
