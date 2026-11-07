// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_runtime.hpp"
#include <sqlite3.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <atomic>
#include <sys/wait.h>
#include <unistd.h>
using namespace cmr::mission;
namespace fs = std::filesystem;

void ck(bool b, const std::string &s) {
  if (!b) {
    throw std::runtime_error(s);
  }
}

struct Fixture {
  fs::path root;
  std::int64_t time{1000};
  bool eligible{true};
  Json snapshot, catalog;

  explicit Fixture(fs::path p) : root(std::move(p)) {
    fs::create_directories(root);
    for (auto f : {"map.yaml", "map.pgm", "bt.xml"}) {
      std::ofstream(root / f) << "synthetic fixture resource " << f;
    }
    auto pose = [](double x) {
      return Json{{"frame", "map"},
                  {"position", {{"x", x}, {"y", 0}, {"z", 0}}},
                  {"orientation", {{"x", 0}, {"y", 0}, {"z", 0}, {"w", 1}}}};
    };
    catalog = {{"schema_version", 1},
               {"map_version", "map"},
               {"station_version", "stations"},
               {"map",
                {{"yaml_path", (root / "map.yaml").string()},
                 {"yaml_sha256", file_sha256((root / "map.yaml").string())},
                 {"image_path", (root / "map.pgm").string()},
                 {"image_sha256", file_sha256((root / "map.pgm").string())},
                 {"metadata",
                  {{"width", 100},
                   {"height", 100},
                   {"resolution", .05},
                   {"origin_x", 0.},
                   {"origin_y", 0.},
                   {"origin_yaw", 0.},
                   {"grid_sha256", bytes_sha256(std::string(10000, 0))}}}}},
               {"bt",
                {{"path", (root / "bt.xml").string()},
                 {"sha256", file_sha256((root / "bt.xml").string())}}},
               {"bounds", {{"min_x", -3}, {"max_x", 3}, {"min_y", -3}, {"max_y", 3}}},
               {"stations", Json::array({{{"station_id", "a"},
                                          {"map_version", "map"},
                                          {"station_version", "stations"},
                                          {"pose", pose(1)}},
                                         {{"station_id", "b"},
                                          {"map_version", "map"},
                                          {"station_version", "stations"},
                                          {"pose", pose(0)}}})}};
    std::ofstream(root / "catalog.json") << catalog.dump();
    snapshot = StationCatalog((root / "catalog.json").string()).freeze(request("active"));
  }

  MissionRequest request(std::string mission_id) {
    return {1, std::move(mission_id), "a", "b", "map", "stations", 1};
  }

  StoreOptions options(std::size_t cap = 4) {
    StoreOptions o;
    o.delivery = true;
    o.queue_capacity = cap;
    o.now = [this] { return time; };
    o.eligible = [this](const Json &s) { return eligible && s == snapshot; };
    return o;
  }

  std::string db() const {
    return (root / "mission.db").string();
  }

  Event event(const Record &r, Kind kind) {
    Event e;
    e.kind = kind;
    e.id = "test-" + std::to_string(++time);
    e.now = time;
    e.mission_id = r.request.mission_id;
    e.epoch = r.epoch;
    e.operation = r.operation;
    e.attempt = r.attempt;
    e.uuid = r.uuid;
    e.expected_version = r.version;
    e.gate_epoch = r.gate_epoch;
    e.clock_epoch = r.clock_epoch;
    e.observed_from = time - 1;
    e.observed_to = time;
    e.valid_until = time + 200;
    e.synthetic = true;
    e.source_from_ns = 2000000000LL;
    e.source_to_ns = 3000000000LL;
    e.arrival_verified = true;
    e.source_fresh = true;
    return e;
  }

  void apply(Store &s, const std::string &mission_id, Kind kind) {
    auto r = *s.get(mission_id);
    auto e = event(r, kind);
    if (kind == Kind::Accepted) {
      e.uuid = "uuid-" + r.attempt;
    }
    if (kind == Kind::Succeeded) {
      e.source_to_ns = 1000000000LL;
    }
    auto d = s.apply(e);
    ck(d.changed, "transition failed " + std::to_string(static_cast<int>(kind)) + " " + d.reason);
    s.dispatch();
  }

  void leg(Store &s, const std::string &mission_id, bool first = true) {
    if (first) {
      apply(s, mission_id, Kind::Prepare);
    }
    for (auto k : {Kind::Accepted, Kind::Succeeded, Kind::RevokeAck, Kind::Stop, Kind::Arrival,
                   Kind::Resolve}) {
      apply(s, mission_id, k);
    }
  }

  void complete(Store &s, const std::string &mission_id) {
    leg(s, mission_id);
    auto r = *s.get(mission_id);
    auto q =
        s.confirm_handover(mission_id, "handover1", r.handover_id, r.handover_version, "synthetic", true);
    ck(q.code == "CONFIRMED", "handover");
    s.dispatch();
    leg(s, mission_id, false);
    ck(s.get(mission_id)->state == State::Completed, "completed");
  }
};

long count(const std::string &path, const char *sql) {
  sqlite3 *db = nullptr;
  ck(sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) == SQLITE_OK, "reader");
  sqlite3_stmt *q = nullptr;
  ck(sqlite3_prepare_v2(db, sql, -1, &q, nullptr) == SQLITE_OK, "query");
  ck(sqlite3_step(q) == SQLITE_ROW, "row");
  auto n = sqlite3_column_int64(q, 0);
  sqlite3_finalize(q);
  sqlite3_close(db);
  return n;
}

void capacity(Fixture &f) {
  Store s(f.db(), "e", nullptr, {}, f.options());
  ck(s.register_delivery(f.request("active"), f.snapshot).code == "ACCEPTED", "active");
  for (int n = 0; n < 4; ++n) {
    ck(s.register_delivery(f.request("q" + std::to_string(n)), f.snapshot).code == "QUEUED",
       "queue4");
  }
  ck(s.register_delivery(f.request("fifth"), f.snapshot).code == "QUEUE_FULL", "fifth");
  ck(s.register_delivery(f.request("q0"), f.snapshot).code == "EXISTING", "duplicate before full");
  auto bad = f.snapshot;
  bad["outbound"]["pose"]["position"]["x"] = 2;
  ck(s.register_delivery(f.request("q0"), bad).code == "CONFLICT", "snapshot conflict");
  ck(!s.registration("q0")->record, "waiting is not mission");
  ck(count(f.db(), "SELECT count(*) FROM missions WHERE terminal=0") == 1, "one active");
}

void fairness(Fixture &f) {
  Store s(f.db(), "e", nullptr, {}, f.options());
  s.register_delivery(f.request("active"), f.snapshot);
  s.register_delivery(f.request("q0"), f.snapshot);
  f.complete(s, "active");
  ck(s.register_delivery(f.request("new"), f.snapshot).code == "QUEUED",
     "new cannot jump empty slot");
  auto p = s.promote();
  ck(p.code == "PROMOTED" && p.record->request.mission_id == "q0", "FIFO head");
  ck(s.cancel_delivery("new", "cancel_new").code == "QUEUE_CANCELED", "waiting cancel");
  ck(s.get("q0")->pending.empty(), "queue cancel cannot touch active");
  ck(s.register_delivery(f.request("new"), f.snapshot).code == "EXISTING",
     "canceled registration permanent");
}

void zero(Fixture &f) {
  Store s(f.db(), "e", nullptr, {}, f.options(0));
  s.register_delivery(f.request("a1"), f.snapshot);
  ck(s.register_delivery(f.request("a2"), f.snapshot).code == "BUSY", "capacity0 BUSY");
  ck(s.register_delivery(f.request("a1"), f.snapshot).code == "EXISTING", "capacity0 duplicate");
}

void handover(Fixture &f) {
  Store s(f.db(), "e", nullptr, {}, f.options());
  s.register_delivery(f.request("m"), f.snapshot);
  f.leg(s, "m");
  auto r = *s.get("m");
  ck(s.confirm_handover("m", "wrong", r.handover_id, r.handover_version + 1, "human", true).code
         == "HANDOVER_IDENTITY",
     "wrong entry version");
  ck(s.confirm_handover("m", "noD", r.handover_id, r.handover_version, "human", false).code
         == "EVIDENCE_UNQUALIFIED",
     "live evidence");
  auto a = s.confirm_handover("m", "ok", r.handover_id, r.handover_version, "synthetic", true);
  ck(a.code == "CONFIRMED" && s.get("m")->handover_source == "synthetic", "confirmed source enum");
  auto b = s.confirm_handover("m", "ok", r.handover_id, r.handover_version, "synthetic", false);
  ck(b.code == "CONFIRMED" && b.duplicate, "lost response idempotent");
  ck(s.confirm_handover("m", "ok", r.handover_id, r.handover_version, "human", true).code
         == "COMMAND_CONFLICT",
     "command content conflict");
  f.leg(s, "m", false);
  ck(s.confirm_handover("m", "ok", r.handover_id, r.handover_version, "synthetic", false).code
         == "CONFIRMED",
     "stable original decision after terminal");
  ck(count(f.db(), "SELECT count(*) FROM operations WHERE kind=0") == 2, "only two send intents");
}

void expire(Fixture &f) {
  auto o = f.options();
  Store s(
      f.db(), "e", nullptr,
      [&](const std::string &hook) {
        if (hook == "handover-before-commit") {
          f.time = s.get("m")->handover_until;
        }
      },
      o);
  s.register_delivery(f.request("m"), f.snapshot);
  f.leg(s, "m");
  auto r = *s.get("m");
  auto a = s.confirm_handover("m", "expire", r.handover_id, r.handover_version, "synthetic", true);
  ck(a.code == "HANDOVER_EXPIRED_BEFORE_COMMIT", "exact commit deadline");
  ck(!s.get("m")->handed_over && count(f.db(), "SELECT count(*) FROM operations WHERE kind=0") == 1,
     "no return before commit");
  auto tick = f.event(*s.get("m"), Kind::Tick);
  ck(s.apply(tick).changed, "handover timeout");
  ck(s.cancel_delivery("m", "late_cancel").code == "CANCEL_PENDING",
     "first writer timeout stays failed");
  ck(s.get("m")->pending == "FAILED", "timeout priority");
}

void races(Fixture &f) {
  Runtime runtime(f.db(), "e", 32, nullptr, {}, f.options());
  auto call = [&](auto fn) { return runtime.call(fn).get(); };
  call([&](Store &s) {
    s.schedule(false);
    return s.register_delivery(f.request("q0"), f.snapshot);
  });
  // Owner barrier fixes receive order before release: cancel wins promotion.
  std::promise<void> release;
  auto gate = release.get_future().share();
  auto held = runtime.call([gate](Store &) { gate.wait(); });
  auto cancel = runtime.call([](Store &s) { return s.cancel_delivery("q0", "cancel1"); });
  auto resume = runtime.call([](Store &s) { return s.schedule(true); });
  auto promote = runtime.call([](Store &s) { return s.promote(); });
  release.set_value();
  held.get();
  ck(cancel.get().code == "QUEUE_CANCELED" && resume.get().code == "RESUMED"
         && promote.get().code == "EMPTY",
     "cancel first");
  call([&](Store &s) {
    s.schedule(false);
    return s.register_delivery(f.request("q1"), f.snapshot);
  });
  call([](Store &s) { return s.schedule(true); });
  auto p = runtime.call([](Store &s) { return s.promote(); });
  auto c = runtime.call([](Store &s) { return s.cancel_delivery("q1", "cancel2"); });
  ck(p.get().code == "PROMOTED" && c.get().code == "CANCEL_PENDING",
     "promote first uses active cancel");
  auto r = call([](Store &s) { return s.active(); });
  ck(r && r->pending == "CANCELED", "active retained until resolve");
}

void restart(Fixture &f) {
  std::uint64_t order = 0;
  {
    Store s(f.db(), "e", nullptr, {}, f.options());
    s.register_delivery(f.request("m"), f.snapshot);
    s.register_delivery(f.request("q"), f.snapshot);
    order = s.registration("q")->queue_seq;
    f.apply(s, "m", Kind::Prepare);
  }
  Store s(f.db(), "new", nullptr, {}, f.options());
  ck(s.get("m")->state == State::Paused && s.get("m")->phase == Phase::Uncertain
         && s.get("m")->restart_suspended,
     "restart uncertain and cannot resume old deadlines");
  ck(s.registration("q")->queue_seq == order && s.queue_snapshot()["schedule_paused"] == true,
     "order and pause");
  ck(s.promote().code == "BUSY" && s.schedule(true).code == "ACTIVE_UNRESOLVED",
     "no automatic restart motion");
  ck(count(f.db(), "SELECT count(*) FROM operations WHERE status='INTENT'") == 0,
     "no replay intents");
}

void fault(Fixture &f) {
  try {
    Store s(
        f.db(), "e", nullptr,
        [](const std::string &h) {
          if (h == "delivery-submit-before-commit") {
            throw std::runtime_error("injected commit failure");
          }
        },
        f.options());
    s.register_delivery(f.request("m"), f.snapshot);
    throw std::logic_error("not injected");
  } catch (const std::logic_error &) {
    throw;
  } catch (const std::runtime_error &) {
  }
  Store s(f.db(), "recover", nullptr, {}, f.options());
  ck(!s.registration("m") && !s.active(), "failed commit not accepted");
}

void mismatch(Fixture &f) {
  Store s(f.db(), "e", nullptr, {}, f.options());
  s.schedule(false);
  s.register_delivery(f.request("q"), f.snapshot);
  s.schedule(true);
  f.eligible = false;
  ck(s.promote().code == "HEAD_INPUT_MISMATCH" && s.queue_snapshot()["schedule_paused"] == true
         && !s.get("q"),
     "head freeze guard");
}

void schema(Fixture &f) {
  {
    Store old(f.db(), "e");
    old.submit(f.request("old"));
  }
  auto before = file_sha256(f.db());
  bool refused = false;
  try {
    Store s(f.db(), "new", nullptr, {}, f.options());
  } catch (const std::exception &e) {
    refused = std::string(e.what()).find("MIGRATION_REQUIRED") != std::string::npos;
  }
  ck(refused && file_sha256(f.db()) == before, "no implicit historical DB migration");
}

void catalog_cases(Fixture &f) {
  StationCatalog good((f.root / "catalog.json").string());
  bool rejected = false;
  auto request = f.request("m");
  request.delivery_station = "unknown";
  try {
    good.freeze(request);
  } catch (...) {
    rejected = true;
  }
  ck(rejected, "unknown station");
  request = f.request("m");
  request.map_version = "other";
  rejected = false;
  try {
    good.freeze(request);
  } catch (...) {
    rejected = true;
  }
  ck(rejected, "wrong map version");
  for (int n = 0; n < 9; ++n) {
    auto data = f.catalog;
    if (n == 0) {
      data["stations"][0]["pose"]["frame"] = "odom";
    }
    if (n == 1) {
      data["stations"][0]["pose"]["orientation"]["w"] = 0;
    }
    if (n == 2) {
      data["stations"][0]["pose"]["position"]["x"] = 4;
    }
    if (n == 3) {
      data["stations"][0]["map_version"] = "other";
    }
    if (n == 4) {
      data["stations"][1]["station_id"] = "a";
    }
    if (n == 5) {
      data["map"]["metadata"]["width"] = 4294967297ULL;
    }
    if (n == 6) {
      data["bt"]["sha256"] = std::string(64, '0');
    }
    if (n == 7) {
      data["unexpected"] = "field";
    }
    if (n == 8) {
      data["stations"][0]["pose"]["position"]["x"] = nullptr;
    }
    auto path = f.root / ("bad" + std::to_string(n) + ".json");
    std::ofstream(path) << data.dump();
    rejected = false;
    try {
      StationCatalog bad(path.string());
    } catch (...) {
      rejected = true;
    }
    ck(rejected, "malformed catalog must fail closed");
  }
  std::ofstream(f.root / "map.yaml") << "controlled map hash mismatch";
  ck(!good.resources_current(), "loaded map resource cannot silently change");
}

void long_identity(Fixture &f) {
  Store s(f.db(), "e", nullptr, {}, f.options());
  const auto mission_id = std::string(64, 'm');
  ck(s.register_delivery(f.request(mission_id), f.snapshot).code == "ACCEPTED",
     "maximum Mission ID accepted");
  f.leg(s, mission_id);
  auto r = *s.get(mission_id);
  ck(valid_identifier(r.handover_id), "handover token stays within API identifier bounds");
  ck(s.confirm_handover(mission_id, "confirm", r.handover_id, r.handover_version, "synthetic", true).code
         == "CONFIRMED",
     "64 byte mission can confirm handover");
}

void handover_cancel(Fixture &parent) {
  for (bool confirm_first : {false, true}) {
    Fixture f(parent.root / (confirm_first ? "confirm_first" : "cancel_first"));
    Runtime owner(f.db(), "race", 32, nullptr, {}, f.options());
    auto setup = owner
                     .call([&](Store &s) {
                       s.register_delivery(f.request("m"), f.snapshot);
                       f.leg(s, "m");
                       return *s.get("m");
                     })
                     .get();
    std::promise<void> release;
    auto barrier = release.get_future().share();
    auto blocked = owner.call([barrier](Store &) { barrier.wait(); });
    std::future<DeliveryReply> cancel, confirm;
    auto confirm_fn = [&](Store &s) {
      return s.confirm_handover("m", "confirm", setup.handover_id, setup.handover_version,
                                "synthetic", true);
    };
    if (confirm_first) {
      confirm = owner.call(confirm_fn);
      cancel = owner.call([](Store &s) { return s.cancel_delivery("m", "cancel"); });
    } else {
      cancel = owner.call([](Store &s) { return s.cancel_delivery("m", "cancel"); });
      confirm = owner.call(confirm_fn);
    }
    release.set_value();
    blocked.get();
    ck(cancel.get().code == "CANCEL_PENDING", "cancel decided");
    ck(confirm.get().code == (confirm_first ? "CONFIRMED" : "REJECTED"),
       "handover/cancel transaction order");
    auto current = owner.call([](Store &s) { return *s.get("m"); }).get();
    ck(current.handed_over == confirm_first && current.pending == "CANCELED",
       "committed delivery fact survives cancel");
    ck(count(f.db(), "SELECT count(*) FROM operations WHERE kind=0") == (confirm_first ? 2 : 1),
       "at most one return intent");
  }
}

void crash_windows(Fixture &parent) {
  for (const std::string action : {"submit", "promote", "handover"}) {
    for (bool after : {false, true}) {
      Fixture f(parent.root / (action + (after ? "_after" : "_before")));
      std::string token;
      std::uint64_t version = 0;
      const pid_t pid = fork();
      ck(pid >= 0, "fork");
      if (pid == 0) {
        try {
          bool armed = false;
          const auto hook = action == "submit" ? "delivery-submit" : action;
          Store store(
              f.db(), "crash_owner", nullptr,
              [&](const std::string &point) {
                if (armed
                    && point == hook + std::string(after ? "-after-commit" : "-before-commit")) {
                  _exit(86);
                }
              },
              f.options());
          if (action == "promote") {
            store.schedule(false);
            store.register_delivery(f.request("m"), f.snapshot);
            store.schedule(true);
          }
          if (action == "handover") {
            store.register_delivery(f.request("m"), f.snapshot);
            f.leg(store, "m");
            auto r = *store.get("m");
            token = r.handover_id;
            version = r.handover_version;
            std::ofstream(f.root / "confirmation.json")
                << Json{{"id", token}, {"version", version}}.dump();
          }
          armed = true;
          if (action == "submit") {
            store.register_delivery(f.request("m"), f.snapshot);
          } else if (action == "promote") {
            store.promote();
          } else {
            store.confirm_handover("m", "lost_reply", token, version, "synthetic", true);
          }
          _exit(88);
        } catch (...) {
          _exit(89);
        }
      }
      int status = 0;
      ck(waitpid(pid, &status, 0) == pid && WIFEXITED(status) && WEXITSTATUS(status) == 86,
         "crash point actually hit");
      const long intents = count(f.db(), "SELECT count(*) FROM operations WHERE kind=0");
      Store recovered(f.db(), "new_owner", nullptr, {}, f.options());
      ck(recovered.queue_snapshot()["schedule_paused"] == true, "restart pauses scheduling");
      if (action == "submit") {
        ck(bool(recovered.registration("m")) == after, "submit transaction atomic");
      } else if (action == "promote") {
        ck(bool(recovered.get("m")) == after, "promotion atomic");
        ck(recovered.registration("m")->queue_state == (after ? "ACTIVE" : "WAITING"),
           "queue promotion durable");
      } else {
        auto r = *recovered.get("m");
        ck(r.handed_over == after && intents == (after ? 2 : 1),
           "handover fact and return intent commit together");
        if (after) {
          std::ifstream in(f.root / "confirmation.json");
          Json q;
          in >> q;
          auto reply = recovered.confirm_handover("m", "lost_reply", q["id"], q["version"],
                                                  "synthetic", false);
          ck(reply.code == "CONFIRMED" && reply.duplicate,
             "lost response replay survives process exit");
        }
      }
      if (auto r = recovered.active()) {
        ck(r->restart_suspended && r->state == State::Paused, "no automatic restarted motion");
      }
      ck(count(f.db(), "SELECT count(*) FROM operations WHERE status='INTENT'") == 0,
         "recovery cannot replay external effects");
      std::ofstream(f.root / "crash-proof.json") << Json{
          {"action", action},
          {"after_commit", after},
          {"child_exit", 86},
          {"intent_count_before_reopen", intents},
          {"queue", recovered.queue_snapshot()}}.dump(2);
    }
  }
}

void handover_seeds(const fs::path &root) {
  for (unsigned seed = 1; seed <= 20; ++seed) {
    Fixture f(root / std::to_string(seed));
    Store s(f.db(), "seed_owner", nullptr, {}, f.options());
    s.register_delivery(f.request("m"), f.snapshot);
    f.leg(s, "m");
    const auto baseline = *s.get("m");
    Record r = baseline;
    std::mt19937 random(seed);
    int return_intents = 0;
    bool cancellation_won = false;
    std::ofstream trace(f.root / "handover-replay.jsonl");
    for (int n = 0; n < 200; ++n) {
      if (n % 10 == 0) {
        r = baseline;
        r.deadline = f.time + 40;
        r.handover_until = f.time + 20;
        r.stop_until = f.time + 40;
        r.arrival_until = f.time + 40;
        return_intents = 0;
        cancellation_won = false;
      }
      f.time += random() % 5;
      auto e = f.event(r, Kind::Handover);
      const auto op = random() % 6;
      if (op == 1) {
        e.kind = Kind::Cancel;
      } else if (op == 2) {
        e.kind = Kind::Tick;
      } else if (op == 3) {
        e.expected_version = r.version + 1;
      } else if (op == 4) {
        e.epoch = "stale_owner";
      } else if (op == 5) {
        e.kind = Kind::Feedback;
      }
      e.detail = "synthetic pure FSM interleaving";
      const auto before = r;
      auto d = reduce(r, e);
      r = d.next;
      for (const auto &effect : d.effects) {
        if (effect.kind == EffectKind::Send) {
          ++return_intents;
        }
      }
      ck(return_intents <= 1, "one return intent per handover episode");
      ck(!before.handed_over || r.handed_over, "handover fact monotonic");
      if (e.kind == Kind::Cancel && d.changed && !before.handed_over) {
        cancellation_won = true;
      }
      ck(!cancellation_won || !r.handed_over, "cancel winner prevents later handover");
      if (e.kind == Kind::Handover && d.changed) {
        ck(e.now < before.deadline && e.now < before.handover_until && e.now < before.stop_until
               && e.now < before.arrival_until,
           "no expired handover");
      }
      if (op == 4) {
        ck(!d.changed, "stale owner ignored");
      }
      trace
          << Json{{"seed", seed},       {"receive_order", n},   {"episode", n / 10},
                  {"operation", op},    {"now", e.now},         {"before", encode(before)},
                  {"after", encode(r)}, {"decision", d.reason}, {"return_intents", return_intents},
                  {"synthetic", true}}
                 .dump()
          << '\n';
    }
  }
}

void seeds(const fs::path &root, unsigned first = 1, unsigned last = 20) {
  for (unsigned seed = first; seed <= last; ++seed) {
    Fixture f(root / std::to_string(seed));
    Store s(f.db(), "seed", nullptr, {}, f.options());
    std::mt19937 gen(seed);
    std::ofstream trace(f.root / "replay.jsonl");
    for (int i = 0; i < 200; ++i) {
      auto mission_id = "id" + std::to_string(gen() % 8);
      auto op = gen() % 5;
      std::string code;
      if (op < 2) {
        code = s.register_delivery(f.request(mission_id), f.snapshot).code;
      } else if (op == 2) {
        code = s.cancel_delivery(mission_id, "cmd" + std::to_string(i)).code;
      } else if (op == 3) {
        code = s.promote().code;
      } else {
        code = s.schedule(false).code;
      }
      auto a = s.active();
      if (a && a->pending == "CANCELED" && !a->ever_sent) {
        f.apply(s, a->request.mission_id, Kind::Resolve);
      }
      auto q = s.queue_snapshot();
      std::uint64_t previous = 0;
      int waiting = 0;
      for (const auto &x : q["orders"]) {
        auto seq = x["queue_seq"].get<std::uint64_t>();
        ck(seq > previous, "strict order");
        previous = seq;
        if (x["queue_state"] == "WAITING") {
          ++waiting;
        }
      }
      ck(waiting <= 4 && count(f.db(), "SELECT count(*) FROM missions WHERE terminal=0") <= 1,
         "capacity and slot invariant");
      trace << Json{{"seed", seed},     {"receive_order", i}, {"operation", op},
                    {"mission_id", mission_id}, {"decision", code},   {"queue", q}}
                   .dump()
            << '\n';
    }
  }
}

int main(int argc, char **argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("mode and independent output root required");
    }
    fs::path root = fs::absolute(argv[2]) / std::to_string(getpid());
    const std::string mode = argv[1];
    if (mode == "handover_seeds") {
      handover_seeds(root);
    } else if (mode == "seeds") {
      seeds(root);
    } else if (mode.rfind("seed_", 0) == 0) {
      auto seed = std::stoul(mode.substr(5));
      ck(seed >= 1 && seed <= 20, "fixed seed range");
      seeds(root, seed, seed);
    } else {
      Fixture f(root);
      if (mode == "capacity") {
        capacity(f);
      } else if (mode == "fairness") {
        fairness(f);
      } else if (mode == "zero") {
        zero(f);
      } else if (mode == "handover") {
        handover(f);
      } else if (mode == "expire") {
        expire(f);
      } else if (mode == "races") {
        races(f);
      } else if (mode == "restart") {
        restart(f);
      } else if (mode == "fault") {
        fault(f);
      } else if (mode == "mismatch") {
        mismatch(f);
      } else if (mode == "schema") {
        schema(f);
      } else if (mode == "handover_cancel") {
        handover_cancel(f);
      } else if (mode == "long_identity") {
        long_identity(f);
      } else if (mode == "catalog") {
        catalog_cases(f);
      } else if (mode == "crash_windows") {
        crash_windows(f);
      } else {
        throw std::runtime_error("unknown mode");
      }
    }
    std::cout << "PASS delivery store " << mode << " artifacts=" << root << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
