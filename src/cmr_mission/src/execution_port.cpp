// SPDX-License-Identifier: MIT
#include "cmr_mission/execution_port.hpp"
#include "cmr_mission/effect_sequence.hpp"
#include <cmr_execution/ros_convert.hpp>
#include <cmr_interfaces/msg/gate_state.hpp>
#include <cmr_interfaces/msg/stop_report.hpp>
#include <cmr_interfaces/srv/execution_lease.hpp>
#include <atomic>
#include <deque>
#include <map>
#include <set>
#include <mutex>
#include <iostream>

namespace cmr::mission {
namespace x = cmr::execution;
using Lease = cmr_interfaces::srv::ExecutionLease;
using Status = cmr_interfaces::msg::GateState;
using Report = cmr_interfaces::msg::StopReport;
using Clock = std::chrono::steady_clock;

struct ExecutionPort::Impl : std::enable_shared_from_this<Impl> {
  struct Context {
    Effect effect;
    x::Identity who;
    std::string clock;
    std::string lease;
    std::string stage{"INSPECT"};
    bool rpc{false};
    bool cancelled{false};
    bool has_grant{false};
    bool nav_sent{false};
    bool rejected{false};
    bool revoked{false};
    bool fault_reported{false};
    std::uint64_t generation{};
    std::uint64_t last_renew_version{};
    x::Ns revoke_ros{}, last_report_end{};
    Clock::time_point created{Clock::now()};
    Clock::time_point retry{};
    Clock::time_point valid_until{};
    Clock::time_point last_stop{};
    std::optional<Effect> authorized;
  };

  struct Pending {
    std::int64_t request_id;
    Clock::time_point until;
    std::shared_ptr<Context> context;
    std::string operation;
  };

  ExecutionConfig config;
  rclcpp::Node::WeakPtr node;
  NavigationAdapter *navigation;
  NavigationAdapter::EventSink sink;
  rclcpp::Client<Lease>::SharedPtr client;
  rclcpp::Subscription<Status>::SharedPtr status_sub;
  rclcpp::Subscription<Report>::SharedPtr report_sub;
  std::mutex mutex;
  std::deque<Effect> commands;
  std::deque<Event> events;
  std::map<std::string, std::shared_ptr<Context>> contexts;
  std::map<std::uint64_t, Pending> pending;
  std::set<std::string> seen;
  std::atomic<bool> alive{true};
  std::atomic<bool> fault{false};
  bool closing{false};
  bool reset_pending{false};
  std::uint64_t serial{0};
  std::uint64_t token{0};
  std::uint64_t event_seq{0};
  std::optional<Status> latest;
  Clock::time_point latest_at{};

  struct Authority {
    Status state;
    Clock::time_point received;
  };

  std::shared_ptr<const Authority> authority;

  struct ReceivedReport {
    Report report;
    Clock::time_point received;
  };

  std::optional<ReceivedReport> deferred_report;

  Event event(const std::shared_ptr<Context> &c, Kind kind, std::string detail) {
    Event e;
    e.id = "execution-" + config.owner + "-" + std::to_string(++event_seq);
    e.mission = c->effect.mission;
    e.epoch = config.owner;
    e.operation = c->effect.operation;
    e.attempt = c->effect.attempt;
    e.uuid = c->effect.uuid;
    e.kind = kind;
    e.gate_epoch = c->who.gate;
    e.clock_epoch = c->clock;
    e.synthetic = config.synthetic_observations;
    e.execution_verified = true;
    e.detail = std::move(detail);
    return e;
  }

  void emit(Event e) {
    if (!alive) {
      return;
    }
    if (events.size() >= 64) {
      fault = true;
      std::cerr << "EXECUTION_EVENT_OVERFLOW; no new authorization\n";
      return;
    }
    events.push_back(std::move(e));
  }

  void fail(const std::shared_ptr<Context> &c, const std::string &why) {
    if (!c->fault_reported) {
      c->fault_reported = true;
      emit(event(c, Kind::Fatal, "EXECUTION " + why));
    }
    c->cancelled = true;
    c->stage = "REVOKE";
  }

  void not_sent(const std::shared_ptr<Context> &c) {
    if (!c->nav_sent && !c->rejected) {
      c->rejected = true;
      emit(event(c, Kind::Rejected, "NOT_SENT EXECUTION_CANCEL_OR_FAILURE"));
    }
  }

  Lease::Request::SharedPtr request(const std::shared_ptr<Context> &c, const std::string &op) {
    auto q = std::make_shared<Lease::Request>();
    q->operation = op;
    q->identity = x::to(c->who);
    q->lease_id = c->lease;
    q->request_id = "exec." + config.owner + "." + std::to_string(++serial);
    q->request_seq = serial;
    q->old_goal_resolved = true;
    return q;
  }

  void send(const std::shared_ptr<Context> &c, Lease::Request::SharedPtr q) {
    if (!alive || c->rpc || pending.size() >= 8 || !client->service_is_ready()) {
      return;
    }
    const auto t = ++token;
    const auto sent_at = Clock::now();
    auto weak = weak_from_this();
    c->rpc = true;
    try {
      auto f = client->async_send_request(
          q, [weak, c, q, t, sent_at](rclcpp::Client<Lease>::SharedFuture f) {
            if (auto self = weak.lock(); self && self->alive) {
              self->pending.erase(t);
              c->rpc = false;
              try {
                self->response(c, q, *f.get(), sent_at);
              } catch (const std::exception &e) {
                self->fail(c, std::string("RPC_RESPONSE ") + e.what());
              }
            }
          });
      pending.emplace(
          t, Pending{f.request_id, Clock::now() + std::chrono::milliseconds(250), c, q->operation});
    } catch (const std::exception &e) {
      c->rpc = false;
      fail(c, std::string("RPC_UNKNOWN ") + e.what());
    }
  }

  void response(const std::shared_ptr<Context> &c, Lease::Request::SharedPtr q,
                const Lease::Response &r, Clock::time_point sent_at) {
    if (q->operation == "inspect") {
      c->who.gate = r.gate_epoch;
      c->clock = r.clock_epoch;
      c->generation = r.generation;
      c->stage = "WAIT_CONTEXT";
      auto e = event(c, Kind::ExecutionContext,
                     "GATE_CONTEXT epoch=" + r.gate_epoch + " clock=" + r.clock_epoch);
      e.id = "execution-context-" + config.owner + "-" + c->effect.attempt;
      emit(e);
      return;
    }
    if (r.gate_epoch != c->who.gate) {
      fail(c, "GATE_RESTART");
      return;
    }
    if (q->operation == "reset") {
      if (r.code == "RESET_DISARMED" && r.state == "DISARMED") {
        c->stage = "RESET_DONE";
        reset_pending = false;
        emit(event(c, Kind::Feedback, "GATE_RESET_DISARMED; no new authority"));
      } else {
        fail(c, "RESET_DENIED " + r.code);
        reset_pending = true;
      }
      return;
    }
    if (q->operation == "challenge_begin" || q->operation == "challenge_renew") {
      if (r.code == "CHALLENGE") {
        if (c->cancelled || closing) {
          c->stage = "REVOKE";
          return;
        }
        q->operation = q->operation == "challenge_begin" ? "begin" : "renew";
        q->challenge = r.challenge;
        send(c, q);
      } else if (q->operation == "challenge_begin" && r.state == "DISARMED") {
        c->retry = Clock::now() + std::chrono::milliseconds(50);
      } else {
        fail(c, "CHALLENGE_DENIED " + r.code);
      }
      return;
    }
    if (q->operation == "begin") {
      if (r.code == "GRANTED") {
        c->has_grant = true;
        c->lease = r.lease_id;
        c->generation = r.generation;
        c->valid_until = sent_at + std::chrono::milliseconds(r.remaining_ms);
        if (c->cancelled || closing) {
          c->stage = "REVOKE";
          not_sent(c);
          return;
        }
        c->stage = "WAIT_BIND";
        auto e = event(c, Kind::ExecutionBind,
                       "GATE_GRANTED lease=" + c->lease + " seq=" + std::to_string(q->request_seq));
        e.id = "execution-bind-" + config.owner + "-" + c->effect.attempt;
        emit(e);
      } else if (r.state == "DISARMED") {
        c->retry = Clock::now() + std::chrono::milliseconds(50);
      } else {
        fail(c, "BEGIN_DENIED " + r.code);
      }
      return;
    }
    if (q->operation == "renew") {
      if (r.code != "RENEWED" || r.state != "ARMED") {
        fail(c, "RENEW_DENIED " + r.code);
        return;
      }
      c->valid_until = sent_at + std::chrono::milliseconds(r.remaining_ms);
      emit(event(c, Kind::Feedback,
                 "GATE_RENEWED lease=" + c->lease + " seq=" + std::to_string(q->request_seq)
                     + " deadline_gate_ms=" + std::to_string(r.deadline_gate_ms)));
      return;
    }
    if (q->operation == "revoke") {
      if (r.code != "REVOKED" || r.state == "ARMED") {
        c->retry = Clock::now() + std::chrono::milliseconds(100);
        return;
      }
      c->revoked = true;
      c->generation = r.generation;
      c->clock = r.clock_epoch;
      c->revoke_ros = r.ros_ns;
      c->stage = "REVOKED";
      emit(event(c, Kind::RevokeAck,
                 "GATE_REVOKED lease=" + c->lease + " generation=" + std::to_string(r.generation)));
      not_sent(c);
      return;
    }
  }

  void state(const Status &s, const rclcpp::MessageInfo &info) {
    auto owner = node.lock();
    if (!owner
        || !x::publisher_matches(*owner, config.ns + "/state", "execution_gate", config.ns, info)) {
      return;
    }
    latest = s;
    latest_at = Clock::now();
    std::atomic_store(&authority, std::make_shared<const Authority>(Authority{s, latest_at}));
    for (auto &pair : contexts) {
      auto c = pair.second;
      if (x::from(s.identity) != c->who || (!c->lease.empty() && s.lease_id != c->lease)) {
        continue;
      }
      if (c->lease.empty() && !s.lease_id.empty() && s.allowed && s.state == "ARMED") {
        c->lease = s.lease_id;
        c->has_grant = true;
      }
      if (!c->clock.empty() && s.clock_epoch != c->clock) {
        c->clock = s.clock_epoch;
        c->revoked = false;
        auto e = event(c, Kind::ExecutionClock, "GATE_CLOCK_CHANGED");
        emit(e);
        c->cancelled = true;
        c->stage = "REVOKE";
      }
      if (c->has_grant && !s.allowed && !c->cancelled) {
        fail(c, "GATE_INVALIDATED " + s.reason);
      }
    }
    consume_report();
  }

  void report(const Report &r, const rclcpp::MessageInfo &info) {
    auto owner = node.lock();
    if (!owner
        || !x::publisher_matches(*owner, config.ns + "/stop_report", "execution_observer",
                                 config.ns, info)) {
      return;
    }
    deferred_report = ReceivedReport{r, Clock::now()};
    consume_report();
  }

  void consume_report() {
    if (!deferred_report) {
      return;
    }
    auto now = Clock::now();
    if (now - deferred_report->received >= std::chrono::milliseconds(200)) {
      deferred_report.reset();
      return;
    }

    // Match authority/identity, then source time and freshness, before forwarding Stop.
    const auto &r = deferred_report->report;
    for (auto &pair : contexts) {
      auto c = pair.second;
      if ((!config.synthetic_observations && r.reason != "WINDOW_PROVEN_BODY_POSE") || !c->revoked
          || r.verdict != "STOPPED" || r.synthetic != config.synthetic_observations
          || x::from(r.identity) != c->who || r.lease_id != c->lease || r.clock_epoch != c->clock
          || r.generation != c->generation || r.from_ros_ns < c->revoke_ros
          || r.to_ros_ns <= c->last_report_end || !latest
          || now - latest_at >= std::chrono::milliseconds(100) || latest->ros_ns < r.to_ros_ns
          || latest->ros_ns - r.to_ros_ns >= 200000000
          || now - c->last_stop < std::chrono::milliseconds(50)) {
        continue;
      }
      Event e = event(c, Kind::Stop,
                      "D_REPORT " + r.evidence_ref + " synthetic=" + std::to_string(r.synthetic)
                          + " generation=" + std::to_string(r.generation) + " from="
                          + std::to_string(r.from_ros_ns) + " to=" + std::to_string(r.to_ros_ns));
      e.source_generation = r.generation;
      e.source_received_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        deferred_report->received.time_since_epoch())
                                        .count();
      e.continuous_zero = true;
      e.source_fresh = true;
      e.source_from_ns = r.from_ros_ns;
      e.source_to_ns = r.to_ros_ns;
      e.revoke_ros_ns = c->revoke_ros;
      e.max_source_gap_ns = r.max_gap_ns;
      e.source_samples = r.samples;
      e.drift_m = r.drift_m;
      e.yaw_drift_rad = r.yaw_drift_rad;
      e.evidence_ref = r.evidence_ref;
      // Owner converts receipt-relative lifetime; no cross-process steady subtraction.
      e.valid_until = 200;
      c->last_report_end = r.to_ros_ns;
      c->last_stop = now;
      emit(std::move(e));
      deferred_report.reset();
      break;
    }
  }
};

ExecutionPort::ExecutionPort(rclcpp::Node::SharedPtr node, ExecutionConfig cfg,
                             NavigationAdapter &nav, NavigationAdapter::EventSink sink)
    : p(std::make_shared<Impl>()) {
  p->config = std::move(cfg);
  p->node = node;
  p->navigation = &nav;
  p->sink = std::move(sink);
  p->client = node->create_client<Lease>(p->config.ns + "/lease");
  auto weak = std::weak_ptr<Impl>(p);
  auto qos = rclcpp::QoS(1).reliable().durability_volatile();
  p->status_sub = node->create_subscription<Status>(
      p->config.ns + "/state", qos,
      [weak](Status::ConstSharedPtr s, const rclcpp::MessageInfo &info) {
        if (auto p = weak.lock(); p && p->alive) {
          p->state(*s, info);
        }
      });
  p->report_sub = node->create_subscription<Report>(
      p->config.ns + "/stop_report", qos,
      [weak](Report::ConstSharedPtr s, const rclcpp::MessageInfo &info) {
        if (auto p = weak.lock(); p && p->alive) {
          p->report(*s, info);
        }
      });
}

ExecutionPort::~ExecutionPort() {
  stop();
}

bool ExecutionPort::faulted() const {
  return p->fault.load();
}

bool ExecutionPort::evidence_current(const Record &r, std::uint64_t generation) const {
  auto a = std::atomic_load(&p->authority);
  return a && Clock::now() - a->received < std::chrono::milliseconds(100) && !a->state.allowed
         && a->state.identity.gate_epoch == r.gate_epoch && a->state.clock_epoch == r.clock_epoch
         && a->state.identity.owner_epoch == r.epoch
         && a->state.identity.mission_id == r.request.mission_id
         && a->state.identity.operation_id == r.operation
         && a->state.identity.attempt_id == r.attempt && a->state.generation == generation;
}

void ExecutionPort::execute(const Effect &e) {
  std::lock_guard<std::mutex> l(p->mutex);
  if (!p->alive || p->commands.size() >= 32
      || (p->fault && e.kind != EffectKind::Revoke && e.kind != EffectKind::CancelExact)) {
    p->fault = true;
    throw std::runtime_error("execution port closed/overloaded/faulted");
  }
  p->commands.push_back(e);
}

void ExecutionPort::tick() {
  if (!p->alive) {
    return;
  }

  // Deliver queued evidence before expiring RPCs or consuming new owner intentions.
  while (!p->events.empty()) {
    if (!p->sink(p->events.front())) {
      break;
    }
    p->events.pop_front();
  }
  for (auto it = p->pending.begin(); it != p->pending.end();) {
    if (Clock::now() >= it->second.until) {
      p->client->remove_pending_request(it->second.request_id);
      auto c = it->second.context;
      c->rpc = false;
      p->fail(c, "RPC_TIMEOUT " + it->second.operation);
      it = p->pending.erase(it);
    } else {
      ++it;
    }
  }

  // Transfer commands under the mailbox lock; executor work remains outside that scope.
  std::deque<Effect> batch;
  {
    std::lock_guard<std::mutex> l(p->mutex);
    batch.swap(p->commands);
  }
  // Revoke queued in the same batch wins over still-unissued navigation sends.
  for (const auto &e : batch) {
    if (e.kind == EffectKind::Revoke) {
      auto it = p->contexts.find(e.attempt);
      if (it != p->contexts.end()) {
        it->second->cancelled = true;
      }
    }
  }
  for (const auto &e : batch) {
    if (e.kind == EffectKind::RenewLease) {
      const auto context = p->contexts.find(e.attempt);
      if (context == p->contexts.end() || context->second->cancelled) {
        continue;
      }
      auto c = context->second;
      if (e.mission != c->effect.mission || e.operation != c->effect.operation) {
        p->fault = true;
        throw std::runtime_error("renew effect scope");
      }
      const auto version = renew_effect_sequence(e);
      if (version <= c->last_renew_version) {
        continue;
      }
      c->last_renew_version = version;
    } else if (!p->seen.insert(e.id).second) {
      continue;
    }
    if (p->seen.size() > 4096) {
      p->fault = true;
      throw std::runtime_error("execution effect capacity");
    }
    if (e.kind == EffectKind::Send) {
      if (p->closing || p->fault) {
        continue;
      }
      if (p->contexts.size() >= 64) {
        p->fault = true;
        throw std::runtime_error("execution context capacity; restart under recovery policy");
      }
      auto c = std::make_shared<Impl::Context>();
      c->effect = e;
      c->who = {p->config.run, "", p->config.owner, e.mission, e.operation, e.attempt};
      p->contexts[e.attempt] = c;
      continue;
    }
    auto it = p->contexts.find(e.attempt);
    if (it == p->contexts.end()) {
      continue;
    }
    auto c = it->second;
    if (!e.uuid.empty()) {
      c->effect.uuid = e.uuid;
    }
    if (e.kind == EffectKind::ResetLease) {
      c->cancelled = true;
      c->stage = "RESET";
      p->reset_pending = true;
    } else if (e.kind == EffectKind::BeginLease) {
      c->stage = c->cancelled ? "REVOKE" : "BEGIN";
    } else if (e.kind == EffectKind::SendAuthorized) {
      if (c->cancelled || p->closing) {
        p->not_sent(c);
        continue;
      }
      if (!c->nav_sent) {
        c->authorized = e;
        c->stage = "READY_TO_SEND";
      }
    } else if (e.kind == EffectKind::RenewLease) {
      if (!p->fault && !p->closing && !c->cancelled && c->has_grant && !c->rpc) {
        p->send(c, p->request(c, "challenge_renew"));
      }
    } else if (e.kind == EffectKind::Revoke) {
      c->cancelled = true;
      p->navigation->execute(e);
      c->stage = "REVOKE";
      if (!c->nav_sent) {
        p->not_sent(c);
      }
    } else if (e.kind == EffectKind::CancelExact) {
      p->navigation->execute(e);
    }
  }

  // Authorize navigation only from a fresh grant, then advance the lease RPC stage.
  for (auto &pair : p->contexts) {
    auto c = pair.second;
    auto now = Clock::now();
    if (p->fault && !c->cancelled) {
      p->fail(c, "MAILBOX_FAULT");
    }
    if (c->authorized && !c->nav_sent && !c->cancelled && !p->closing) {
      if (now + std::chrono::milliseconds(20) >= c->valid_until) {
        p->fail(c, "SEND_LEASE_STALE");
        p->not_sent(c);
        c->authorized.reset();
      } else if (p->latest && p->latest->allowed && p->latest->lease_id == c->lease
                 && x::from(p->latest->identity) == c->who
                 && now - p->latest_at < std::chrono::milliseconds(100)) {
        auto send = *c->authorized;
        send.kind = EffectKind::Send;
        p->navigation->execute(send);
        c->nav_sent = true;
        c->authorized.reset();
        c->stage = "ACTIVE";
      }
    }
    if (c->rpc || now < c->retry) {
      continue;
    }
    if ((c->stage == "INSPECT" && !p->reset_pending)
        || (c->stage == "REVOKE" && c->who.gate.empty())) {
      p->send(c, p->request(c, "inspect"));
    } else if (c->stage == "RESET") {
      p->send(c, p->request(c, "reset"));
    } else if (c->stage == "BEGIN") {
      p->send(c, p->request(c, "challenge_begin"));
    } else if (c->stage == "REVOKE" && !c->who.gate.empty()) {
      p->send(c, p->request(c, "revoke"));
    }
    if (!c->has_grant && !c->cancelled && now - c->created > std::chrono::seconds(6)) {
      p->fail(c, "BEGIN_BUDGET");
    }
  }
}

void ExecutionPort::begin_shutdown() {
  p->closing = true;
}

void ExecutionPort::stop() {
  if (!p || !p->alive.exchange(false)) {
    return;
  }
  for (const auto &x : p->pending) {
    p->client->remove_pending_request(x.second.request_id);
  }
  p->pending.clear();
  p->events.clear();
  p->status_sub.reset();
  p->report_sub.reset();
}
} // namespace cmr::mission
