// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_store.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <sqlite3.h>
using namespace cmr::mission;

void ck(bool b, const char *why) {
  if (!b) {
    throw std::runtime_error(why);
  }
}

Event ev(const Record &r, Kind k, int n) {
  Event e;
  e.id = "e" + std::to_string(n);
  e.mission_id = r.request.mission_id;
  e.epoch = r.epoch;
  e.operation = r.operation;
  e.attempt = r.attempt;
  e.uuid = r.uuid;
  e.kind = k;
  e.now = n;
  e.gate_epoch = r.gate_epoch;
  e.clock_epoch = r.clock_epoch;
  return e;
}

struct Capture : Port {
  std::vector<Effect> received;

  void execute(const Effect &e) override {
    received.push_back(e);
  }
};

int main(int argc, char **argv) {
  try {
    ck(argc == 3, "mode dir");
    std::string mode = argv[1];
    std::filesystem::create_directories(argv[2]);
    Capture port;
    std::unique_ptr<Store> store;
    Record r;
    r.request = {1, "m", "a", "b", "map", "stations", 1};
    r.epoch = "owner";
    if (mode == "store") {
      store = std::make_unique<Store>(std::string(argv[2]) + "/mission.db", "owner", &port);
      ck(store->submit(r.request) == Admission::CreateIntent, "submit");
      r = *store->get_record_from_id("m");
    }
    auto apply = [&](Event e) {
      auto d = store ? store->apply(e) : reduce(r, e);
      if (d.changed) {
        r = d.next;
      }
      if (store) {
        store->dispatch();
      }
      return d;
    };
    auto e = ev(r, Kind::Prepare, 1);
    ck(apply(e).changed, "Prepare");
    e = ev(r, Kind::ExecutionContext, 2);
    e.gate_epoch = "real-gate";
    e.clock_epoch = "real-clock";
    ck(!apply(e).changed, "unverified context rejected");
    e.id = "verified-context";
    e.execution_verified = true;
    auto d = apply(e);
    ck(d.changed && d.effects.size() == 1 && d.effects[0].kind == EffectKind::BeginLease,
       "persist context before Begin");
    e = ev(r, Kind::ExecutionBind, 3);
    e.execution_verified = true;
    d = apply(e);
    ck(d.changed && d.effects[0].kind == EffectKind::SendAuthorized,
       "grant persisted before native Send");
    e = ev(r, Kind::Accepted, 4);
    e.uuid = "uuid";
    ck(apply(e).changed, "native accept");
    e = ev(r, Kind::ExecutionRenew, 5);
    e.execution_verified = true;
    d = apply(e);
    ck(d.changed && d.effects[0].kind == EffectKind::RenewLease,
       "owner transaction generates renew");
    e = ev(r, Kind::Cancel, 6);
    ck(apply(e).changed, "Cancel");
    e = ev(r, Kind::ExecutionRenew, 7);
    e.execution_verified = true;
    ck(!apply(e).changed, "paused cannot renew");
    e = ev(r, Kind::RevokeAck, 8);
    ck(!apply(e).changed, "fake ACK rejected for real binding");
    e.id = "verified-ack";
    e.execution_verified = true;
    ck(apply(e).changed, "real ACK");
    e = ev(r, Kind::Stop, 9);
    e.synthetic = false;
    e.valid_until = 100;
    ck(!apply(e).changed, "bare STOPPED bool not evidence");
    e = ev(r, Kind::Stop, 10);
    e.execution_verified = true;
    e.source_fresh = true;
    e.continuous_zero = true;
    e.source_from_ns = 1000000000;
    e.source_to_ns = 4000000000;
    e.revoke_ros_ns = 1000000000;
    e.source_samples = 31;
    e.max_source_gap_ns = 100000000;
    e.evidence_ref = "independent-window";
    e.valid_until = 100;
    auto bad = e;
    bad.id = "old-clock";
    bad.clock_epoch = "old";
    ck(!apply(bad).changed, "old clock");
    bad = e;
    bad.id = "short";
    bad.source_to_ns = 3999999999;
    ck(!apply(bad).changed, "3s exact requirement");
    bad = e;
    bad.id = "pre-revoke";
    bad.revoke_ros_ns = 1000000001;
    ck(!apply(bad).changed, "window must follow revoke");
    bad = e;
    bad.id = "gap";
    bad.max_source_gap_ns = 500000001;
    ck(!apply(bad).changed, "source gap");
    bad = e;
    bad.id = "drift";
    bad.drift_m = .02001;
    ck(!apply(bad).changed, "drift");
    ck(apply(e).changed, "full correlated D accepted");
    e = ev(r, Kind::Resolve, 11);
    ck(!apply(e).changed, "D without Goal terminal keeps slot");
    e = ev(r, Kind::CanceledResult, 12);
    ck(apply(e).changed, "raw Goal terminal");
    ck(r.pending == "CANCELED", "first intent preserved");
    e = ev(r, Kind::Resolve, 13);
    ck(apply(e).changed && r.state == State::Canceled, "only complete proof releases");
    if (store) {
      ck(!store->get_active_record(), "persisted slot released");
      int counts[6] = {};
      sqlite3 *db{};
      ck(sqlite3_open_v2((std::string(argv[2]) + "/mission.db").c_str(), &db, SQLITE_OPEN_READONLY,
                         nullptr)
             == SQLITE_OK,
         "independent DB");
      sqlite3_stmt *q{};
      sqlite3_prepare_v2(db, "SELECT kind,count(*) FROM operations GROUP BY kind", -1, &q, nullptr);
      while (sqlite3_step(q) == SQLITE_ROW) {
        int k = sqlite3_column_int(q, 0);
        if (k < 6) {
          counts[k] = sqlite3_column_int(q, 1);
        }
      }
      sqlite3_finalize(q);
      sqlite3_close(db);
      ck(counts[0] == 1 && counts[3] == 1 && counts[4] == 1 && counts[5] == 1,
         "independent outbox counts");
    }
    // Deterministic early-cancel ordering: logical Send, cancel, definitive
    // NOT_SENT, then late Gate context. It must bind only to revoke, never Begin.
    if (store) {
      MissionRequest early = {1, "early", "a", "b", "map", "stations", 1};
      ck(store->submit(early) == Admission::CreateIntent, "early submit");
      r = *store->get_record_from_id("early");
    } else {
      r = Record{};
      r.request = {1, "early", "a", "b", "map", "stations", 1};
      r.epoch = "owner";
    }
    ck(apply(ev(r, Kind::Prepare, 1)).changed, "early Prepare");
    ck(apply(ev(r, Kind::Cancel, 2)).changed, "early Cancel");
    ck(apply(ev(r, Kind::Rejected, 3)).changed, "definitive native NOT_SENT");
    e = ev(r, Kind::ExecutionContext, 4);
    e.execution_verified = true;
    e.gate_epoch = "early-gate";
    e.clock_epoch = "early-clock";
    d = apply(e);
    ck(d.changed && d.effects.size() == 1 && d.effects[0].kind == EffectKind::Revoke,
       "late context only revokes");
    e = ev(r, Kind::RevokeAck, 5);
    e.execution_verified = true;
    ck(apply(e).changed, "early revoke");
    e = ev(r, Kind::Stop, 6);
    e.execution_verified = true;
    e.source_fresh = true;
    e.continuous_zero = true;
    e.source_from_ns = 1000000000;
    e.source_to_ns = 4000000000;
    e.revoke_ros_ns = 1000000000;
    e.source_samples = 31;
    e.max_source_gap_ns = 100000000;
    e.evidence_ref = "early-independent-D";
    e.valid_until = 100;
    ck(apply(e).changed, "early D bound to actual Gate");
    ck(apply(ev(r, Kind::Resolve, 7)).changed && r.state == State::Canceled,
       "early cancel resolves after proof");
    std::cout << "PASS execution " << mode
              << " context/grant/renew/revoke/D/Goal transaction boundaries\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
