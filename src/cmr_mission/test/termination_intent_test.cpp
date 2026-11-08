// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_runtime.hpp"
#include <filesystem>
#include <iostream>
#include <atomic>
#include <sqlite3.h>
using namespace cmr::mission;

namespace {
void check(bool value, const char *message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

MissionRequest request() {
  return {1, "m", "a", "b", "map", "stations", 1};
}

Event event(const Record &r, Kind kind, std::string id) {
  Event e;
  e.kind = kind;
  e.id = std::move(id);
  e.mission_id = r.request.mission_id;
  e.epoch = r.epoch;
  e.operation = r.operation;
  e.attempt = r.attempt;
  e.uuid = r.uuid;
  e.now = r.last_now + 2;
  e.observed_from = e.now - 1;
  e.observed_to = e.now;
  e.valid_until = e.now + 10000;
  if (kind == Kind::Accepted) {
    e.uuid = "goal-1";
  }
  if (kind == Kind::Tick) {
    e.now = r.deadline;
  }
  return e;
}

Record active() {
  Record r;
  r.request = request();
  r.epoch = "first";
  r = reduce(r, event(r, Kind::Prepare, "prepare")).next;
  r = reduce(r, event(r, Kind::Accepted, "accepted")).next;
  return r;
}

struct Sink : Port {
  std::atomic<int> sends{0};

  void execute(const Effect &e) override {
    if (e.kind == EffectKind::Send) {
      ++sends;
    }
  }
};

void logic() {
  for (Kind result : {Kind::Aborted, Kind::Rejected, Kind::Succeeded}) {
    auto r = active();
    r = reduce(r, event(r, Kind::Cancel, "cancel")).next;
    auto d = reduce(r, event(r, result, "result"));
    check(d.changed, "result fact must be accepted");
    r = d.next;
    check(r.pending == "CANCELED" && r.reason == "CANCELED", "later result overwrote cancellation");
    check(r.goal_terminal, "navigation terminal fact lost");
    for (Kind later : {Kind::Fatal, Kind::Tick}) {
      check(!reduce(r, event(r, later, "later")).changed,
            "later termination replaced first intent");
    }
    for (Kind k : {Kind::RevokeAck, Kind::Stop}) {
      r = reduce(r, event(r, k, "cleanup" + std::to_string(static_cast<int>(k)))).next;
    }
    check(!reduce(r, event(r, Kind::Recover, "recover")).changed, "canceled mission may recover");
    check(!reduce(r, event(r, Kind::Retry, "retry")).changed, "canceled mission may retry");
    r = reduce(r, event(r, Kind::Resolve, "resolve")).next;
    check(r.state == State::Canceled, "final state is not canceled");
  }
  auto r = active();
  r = reduce(r, event(r, Kind::Aborted, "abort-first")).next;
  check(!reduce(r, event(r, Kind::Cancel, "cancel-later")).changed && r.pending == "FAILED",
        "failure-first semantics changed");
  for (Kind k : {Kind::RevokeAck, Kind::Stop}) {
    r = reduce(r, event(r, k, "cleanup" + std::to_string(static_cast<int>(k)))).next;
  }
  check(reduce(r, event(r, Kind::Recover, "recover")).changed,
        "legitimate failure recovery disabled");
  auto restart = active();
  restart = reduce(restart, event(restart, Kind::Cancel, "c")).next;
  restart = reduce(restart, event(restart, Kind::Restart, "restart")).next;
  check(restart.pending == "CANCELED" && restart.state == State::Paused,
        "restart lost cancel intent");
  std::cout << "PASS pure first-intent/result/fatal/tick/restart matrix\n";
}

void store(const std::string &dir) {
  for (Kind result : {Kind::Aborted, Kind::Rejected}) {
    const auto path = dir + "/store-" + std::to_string(static_cast<int>(result)) + ".db";
    Sink sink;
    {
      Runtime rt(path, "first", 16, &sink);
      rt.call([](Store &s) { s.submit_legacy(request()); }).get();
      auto apply = [&](Kind k, std::string id) {
        return rt.call([=](Store &s) { return s.apply(event(*s.get_record_from_id("m"), k, id)); }).get();
      };
      apply(Kind::Prepare, "prepare");
      rt.call([](Store &s) { s.dispatch(); }).get();
      apply(Kind::Accepted, "accepted");
      apply(Kind::Cancel, "cancel");
      apply(result, "result");
      apply(Kind::RevokeAck, "revoke");
      apply(Kind::Stop, "stop");
      check(!apply(Kind::Recover, "recover").changed && !apply(Kind::Retry, "retry").changed,
            "store allowed cancel recovery");
      rt.call([](Store &s) { s.dispatch(); }).get();
      check(sink.sends == 1, "second navigation sent");
    }
    {
      Runtime rt(path, "restart", 16, &sink);
      auto record = *rt.call([](Store &s) { return s.get_record_from_id("m"); }).get();
      check(record.pending == "CANCELED", "persisted cancellation lost");
      auto apply = [&](Kind k, std::string id) {
        return rt.call([=](Store &s) { return s.apply(event(*s.get_record_from_id("m"), k, id)); }).get();
      };
      check(!apply(Kind::Recover, "post-restart-recover").changed, "restart allowed recovery");
      apply(Kind::RevokeAck, "restart-revoke");
      apply(Kind::Stop, "restart-stop");
      apply(Kind::Resolve, "resolve");
      check(rt.call([](Store &s) { return s.get_record_from_id("m")->state; }).get() == State::Canceled,
            "persisted final state");
      rt.call([](Store &s) { s.dispatch(); }).get();
      check(sink.sends == 1, "restart replayed send");
    }
    sqlite3 *db = nullptr;
    check(sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) == SQLITE_OK,
          "oracle open");
    sqlite3_stmt *q = nullptr;
    sqlite3_prepare_v2(db, "SELECT count(*) FROM events WHERE event_id='result' AND kind=?", -1, &q,
                       nullptr);
    sqlite3_bind_int(q, 1, static_cast<int>(result));
    check(sqlite3_step(q) == SQLITE_ROW && sqlite3_column_int(q, 0) == 1,
          "raw action result not persisted");
    sqlite3_finalize(q);
    sqlite3_close(db);
  }
  std::cout << "PASS Runtime/Store persistence, original result audit, restart and send count\n";
}

void races(const std::string &dir) {
  for (Kind result : {Kind::Aborted, Kind::Rejected}) {
    for (int cancel_first : {0, 1}) {
      Sink sink;
      Runtime rt(dir + "/race-" + std::to_string(static_cast<int>(result)) + "-"
                     + std::to_string(cancel_first) + ".db",
                 "owner", 16, &sink);
      rt.call([](Store &s) {
          s.submit_legacy(request());
          s.apply(event(*s.get_record_from_id("m"), Kind::Prepare, "prepare"));
          s.dispatch();
          s.apply(event(*s.get_record_from_id("m"), Kind::Accepted, "accepted"));
        }).get();
      const auto snapshot = *rt.call([](Store &s) { return s.get_record_from_id("m"); }).get();
      std::mutex mutex;
      std::condition_variable cv;
      int ready = 0;
      bool go = false;
      bool first_enqueued = false;
      std::future<Decision> first, second;
      auto producer = [&](bool is_first) {
        {
          std::unique_lock<std::mutex> lock(mutex);
          ++ready;
          cv.notify_all();
          cv.wait(lock, [&] { return go && (is_first || first_enqueued); });
        }
        const auto kind = (is_first == static_cast<bool>(cancel_first)) ? Kind::Cancel : result;
        auto f = rt.call([e = event(snapshot, kind, is_first ? "first" : "second")](Store &s) {
          return s.apply(e);
        });
        if (is_first) {
          first = std::move(f);
          std::lock_guard<std::mutex> lock(mutex);
          first_enqueued = true;
          cv.notify_all();
        } else {
          second = std::move(f);
        }
      };
      std::thread a(producer, true), b(producer, false);
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return ready == 2; });
        go = true;
        cv.notify_all();
      }
      a.join();
      b.join();
      first.get();
      second.get();
      const auto expected = cancel_first ? "CANCELED" : "FAILED";
      auto record = *rt.call([](Store &s) { return s.get_record_from_id("m"); }).get();
      check(record.pending == expected, "ordered concurrent first intent overwritten");
      rt.call([](Store &s) {
          s.apply(event(*s.get_record_from_id("m"), Kind::RevokeAck, "revoke"));
          s.apply(event(*s.get_record_from_id("m"), Kind::Stop, "stop"));
        }).get();
      const auto recovery = rt.call([](Store &s) {
                                return s.apply(event(*s.get_record_from_id("m"), Kind::Recover, "recover"));
                              }).get();
      check(recovery.changed == !static_cast<bool>(cancel_first),
            "ordered recovery eligibility incorrect");
      if (cancel_first) {
        rt.call([](Store &s) {
            s.apply(event(*s.get_record_from_id("m"), Kind::Retry, "retry"));
            s.dispatch();
          }).get();
        check(sink.sends == 1, "race caused second send");
      }
    }
  }
  std::cout << "PASS 4 barrier-controlled cancel/abort/reject transaction orders\n";
}
} // namespace

int main(int argc, char **argv) {
  try {
    check(argc == 3, "MODE ARTIFACT_DIR required");
    std::filesystem::create_directories(argv[2]);
    const std::string mode = argv[1];
    if (mode == "logic") {
      logic();
    } else if (mode == "store") {
      store(argv[2]);
    } else if (mode == "races") {
      races(argv[2]);
    } else {
      throw std::runtime_error("unknown mode");
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
