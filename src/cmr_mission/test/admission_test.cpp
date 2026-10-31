// SPDX-License-Identifier: MIT
#include "cmr_mission/admission.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace cmr::mission;

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

MissionRequest request() { return {1, "order-1", "room.A", "dock", "map-1", "stations-1", 1}; }

void run(const std::string& name) {
  auto a = request();
  if (name == "validation") {
    check(validate(a) == Validation::Valid, "valid request");
    a.schema_version = 0;
    check(validate(a) == Validation::Schema, "unsupported version");
    a = request(); a.created_at_unix_ms = 0;
    check(validate(a) == Validation::CreatedAt, "zero timestamp");
    a.created_at_unix_ms = -1;
    check(validate(a) == Validation::CreatedAt, "negative timestamp");
    a.created_at_unix_ms = std::numeric_limits<std::int64_t>::max();
    check(validate(a) == Validation::Valid, "audit time not deadline arithmetic");
    for (auto field : {&MissionRequest::mission_id, &MissionRequest::delivery_station,
                       &MissionRequest::return_station, &MissionRequest::map_version,
                       &MissionRequest::station_version}) {
      a = request(); a.*field = "";
      check(validate(a) == Validation::Identifier, "each required field checked");
    }
  } else if (name == "identity") {
    check(valid_identifier(std::string(64, 'a')), "64 byte boundary");
    check(!valid_identifier(std::string(65, 'a')), "65 bytes rejected");
    check(valid_identifier("AZaz09_.-"), "ASCII grammar");
    for (const auto& invalid : {std::string(" room"), std::string("room "), std::string("a/b"),
                               std::string("a\nb"), std::string("a\0b", 3), std::string("站点")}) {
      check(!valid_identifier(invalid), "invalid identifier rejected, not normalized");
    }
  } else if (name == "duplicate") {
    AdmissionContext c{a, true};
    a.created_at_unix_ms = 42;
    for (int n = 0; n < 100; ++n)
      check(decide_admission(a, c) == Admission::Existing, "duplicate precedes busy");
    c.active_slot_held = false;
    check(decide_admission(a, c) == Admission::Existing, "terminal lookup remains existing");
    check(c.existing->created_at_unix_ms == 1, "first audit time retained");
  } else if (name == "conflict") {
    const AdmissionContext c{a, true};
    for (auto field : {&MissionRequest::delivery_station, &MissionRequest::return_station,
                       &MissionRequest::map_version, &MissionRequest::station_version}) {
      auto changed = a; changed.*field += "X";
      check(decide_admission(changed, c) == Admission::Conflict, "business field conflict");
    }
    a.delivery_station = "Room.A";
    check(decide_admission(a, c) == Admission::Conflict, "case is meaningful");
  } else if (name == "busy") {
    check(decide_admission(a, {std::nullopt, false}) == Admission::CreateIntent, "free slot proposal");
    check(decide_admission(a, {std::nullopt, true}) == Admission::Busy, "unresolved slot held");
    a.mission_id.clear();
    check(decide_admission(a, {std::nullopt, true}) == Admission::Invalid, "validation before busy");
  } else if (name == "context") {
    auto other = a; other.mission_id = "other";
    check(decide_admission(a, {other, false}) == Admission::InvalidContext, "mismatched lookup");
    other = a; other.delivery_station.clear();
    check(decide_admission(a, {other, true}) == Admission::InvalidContext, "corrupt record");
  } else if (name == "immutable") {
    AdmissionContext c{a, true};
    const auto before = c.existing->delivery_station;
    a.delivery_station = "changed";
    check(decide_admission(a, c) == Admission::Conflict, "conflict");
    check(c.active_slot_held && c.existing->delivery_station == before, "no context mutation");
    check(a.delivery_station == "changed", "no request mutation");
  } else if (name == "permutations") {
    // Frozen held-slot snapshot: exhaustive query order invariance, NOT a runtime concurrency test.
    const AdmissionContext c{a, true};
    auto conflict = a; conflict.return_station = "other-dock";
    auto bad = a; bad.created_at_unix_ms = 0;
    std::array<MissionRequest, 3> queries{a, conflict, bad};
    const std::array<Admission, 3> expected{Admission::Existing, Admission::Conflict, Admission::Invalid};
    std::array<int, 3> order{0, 1, 2};
    int count = 0;
    do {
      for (auto i : order) check(decide_admission(queries[i], c) == expected[i], "query order invariance");
      ++count;
    } while (std::next_permutation(order.begin(), order.end()));
    check(count == 6, "all query permutations");
  } else {
    throw std::runtime_error("unknown test name");
  }
}

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("expected one test name");
    run(argv[1]);
    std::cout << "PASS " << argv[1] << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
