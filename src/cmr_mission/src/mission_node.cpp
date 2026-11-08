// SPDX-License-Identifier: MIT
#include "cmr_mission/nav2_adapter.hpp"
#include "cmr_mission/execution_port.hpp"
#include "cmr_mission/delivery_monitor.hpp"
#include <cmr_interfaces/srv/delivery_command.hpp>
#include "cmr_mission/mission_runtime.hpp"
#include "cmr_mission/action_correlation.hpp"
#include <cmr_interfaces/srv/mission_command.hpp>
#include <cmr_interfaces/srv/synthetic_event.hpp>
#include <csignal>
#include <chrono>
#include <atomic>
#include <iostream>
#include <thread>
#include <set>
#include <map>

namespace cmr::mission {
using DeliveryApi = cmr_interfaces::srv::DeliveryCommand;
using Api = cmr_interfaces::srv::MissionCommand;
using Fixture = cmr_interfaces::srv::SyntheticEvent;
using Clock = std::chrono::steady_clock;
static std::atomic<bool> interrupted{false};

static void signal_handler(int) {
  interrupted = true;
}

class MissionNode : public rclcpp::Node {
  Clock::time_point start = Clock::now();
  std::string epoch;
  bool delivery_mode = false;
  bool delivery_synthetic = false;
  std::unique_ptr<StationCatalog> catalog;
  std::unique_ptr<DeliveryMonitor> monitor;

  struct DeliveryWindow {
    ArrivalWindow arrival;
    std::optional<Event> stop;
  };

  std::map<std::string, DeliveryWindow> delivery_windows;
  rclcpp::Service<DeliveryApi>::SharedPtr delivery_api;

  struct PendingDelivery {
    std::shared_ptr<rmw_request_id_t> header;
    std::uint64_t receipt;
    std::future<DeliveryApi::Response> future;
  };

  std::vector<PendingDelivery> delivery_requests;
  std::unique_ptr<Runtime> runtime;
  std::unique_ptr<NavigationAdapter> adapter;
  std::unique_ptr<ExecutionPort> execution;
  bool gate_mode = false;
  std::string execution_run;
  std::string execution_ns;
  std::optional<std::future<Decision>> health;
  Clock::time_point health_at{};
  std::atomic<bool> health_fault{false};
  std::map<std::string, std::uint64_t> stop_generations;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr frozen_parameters;
  rclcpp::Service<Api>::SharedPtr api;
  rclcpp::Service<Fixture>::SharedPtr fixture;
  rclcpp::TimerBase::SharedPtr timer;

  struct PendingApi {
    std::shared_ptr<rmw_request_id_t> header;
    std::future<Api::Response> future;
  };

  struct PendingFixture {
    std::shared_ptr<rmw_request_id_t> header;
    std::future<Fixture::Response> future;
  };

  std::vector<PendingApi> requests;
  std::vector<PendingFixture> fixtures;
  std::vector<std::future<Decision>> events;
  std::atomic<bool> closing{false};
  bool shutdown_queued = false;
  std::size_t mailbox = 16;
  std::uint64_t seq = 0;
  std::int64_t last_ros_clock = -1;
  std::int64_t last_health_log = -1000;

public:
  MissionNode() : Node("mission_node", rclcpp::NodeOptions().use_clock_thread(false)) {
  }

  ~MissionNode() override {
    if (runtime) {
      runtime->close();
    }
  }

  std::int64_t now_ms() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
  }

  Event ev(const Record &r, Kind kind, std::string id) {
    Event e;
    e.kind = kind;
    e.id = std::move(id);
    e.mission_id = r.request.mission_id;
    e.epoch = r.epoch;
    e.operation = r.operation;
    e.attempt = r.attempt;
    e.uuid = r.uuid;
    e.now = now_ms();
    return e;
  }

  static void fill(Api::Response &response, const Record &r) {
    response.state = state_name(r.state);
    response.state_version = r.version;
    response.pending = r.pending;
    response.operation_id = r.operation;
    response.attempt_id = r.attempt;
    response.goal_uuid = r.uuid;
    response.detail = encode(r);
  }

  bool delivery_eligible(const Json &snapshot) const {
    if (closing || health_fault || !monitor || !catalog || (execution && execution->faulted())
        || (adapter && adapter->faulted())) {
      return false;
    }
    auto h = monitor->snapshot();
    return h && h->healthy && now_ms() - h->now_ms < 100
           && snapshot.at("catalog_sha256") == catalog->sha256() && catalog->resources_current();
  }

  bool delivery_evidence(const Record &r) const {
    auto it = delivery_windows.find(r.attempt);
    return it != delivery_windows.end() && it->second.stop && r.goal_terminal && r.revoked
           && r.stopped && now_ms() < r.stop_until && r.arrived && now_ms() < r.arrival_until
           && execution->evidence_current(r, it->second.stop->source_generation);
  }

  Decision delivery_drive(Store &s) {
    auto active = s.get_active_record();
    if (!active) {
      if (!closing) {
        s.promote();
      }
      return {};
    }
    auto r = *active;
    if (!r.delivery_managed || r.restart_suspended) {
      return {};
    }
    auto registration = s.get_registration_from_id(r.request.mission_id);
    if (!registration) {
      throw std::runtime_error("active delivery registration absent");
    }

    // Business deadline and input-health decisions precede evidence-driven progression.
    const auto time = now_ms();
    const bool healthy = delivery_eligible(registration->snapshot);
    auto apply = [&](Kind kind, const std::string &why) {
      auto e = ev(r, kind, "delivery-" + epoch + "-" + std::to_string(r.version) + "-" + why);
      e.detail = why;
      auto d = s.apply(e);
      s.dispatch();
      if (d.changed) {
        r = d.next;
      }
      return d;
    };
    if (!r.pending.empty() && r.state == State::Paused && !r.goal_terminal) {
      return {};
    }
    if (!terminal(r.state) && r.pending.empty()
        && time >= (r.state == State::WaitHandover && !r.handed_over
                        ? std::min(r.deadline, r.handover_until)
                        : r.deadline)) {
      return apply(Kind::Tick, "deadline");
    }
    if (!closing && healthy && r.state == State::Accepted && r.phase == Phase::None) {
      return apply(Kind::Prepare, "prepare");
    }
    if (!closing && !healthy && r.ever_sent && r.state != State::Paused && r.pending.empty()) {
      return apply(Kind::Fatal, "input_health_failed");
    }
    if (r.state == State::Paused && !r.pending.empty() && !r.ever_sent && r.goal_terminal
        && r.revoked && r.stopped) {
      return apply(Kind::Resolve, "audited_no_motion_termination");
    }

    // Arrival, retry, and resolution require the current stop report as well as Goal terminal.
    if (!r.goal_terminal || !r.revoked || !r.stopped || time >= r.stop_until) {
      return {};
    }
    auto &window = delivery_windows[r.attempt];
    if (!window.stop || !execution->evidence_current(r, window.stop->source_generation)) {
      return {};
    }
    if (healthy && r.goal_succeeded) {
      auto h = monitor->snapshot();
      auto observation = h->pose;
      observation.identity = r.attempt + "/" + r.uuid;
      window.arrival.add(observation);
      const auto target =
          checked_pose(registration->snapshot.at(r.returning ? "return" : "outbound").at("pose"));
      const auto a = window.arrival.evaluate(target, observation.identity, r.clock_epoch,
                                             r.terminal_ros_ns, window.stop->source_from_ns,
                                             window.stop->source_to_ns, h->source_now_ns, time);
      if (window.arrival.jumped() && r.pending.empty()) {
        return apply(Kind::Fatal, "localization_discontinuity");
      }
      if (a.arrived) {
        auto e = ev(r, Kind::Arrival, "arrival-" + epoch + "-" + std::to_string(r.version));
        e.arrival_verified = true;
        e.source_fresh = true;
        e.synthetic = delivery_synthetic;
        e.gate_epoch = r.gate_epoch;
        e.clock_epoch = r.clock_epoch;
        e.source_from_ns = a.from_ns;
        e.source_to_ns = a.to_ns;
        e.valid_until = std::min(r.stop_until, h->now_ms + 200);
        e.detail = Json{{"type", "ARRIVED"},       {"synthetic", delivery_synthetic},
                        {"from_ns", a.from_ns},    {"to_ns", a.to_ns},
                        {"samples", a.samples},    {"position_error", a.position_error},
                        {"yaw_error", a.yaw_error}}
                       .dump();
        auto d = s.apply(e);
        if (d.changed) {
          r = d.next;
        }
      }
    }

    // Retry retains the shared task deadline; otherwise resolve the pending termination.
    if (r.state == State::Paused && !r.pending.empty()) {
      if (!closing && healthy && r.pending == "FAILED" && r.retry_eligible && r.retries < 1
          && now_ms() + 10000 < r.deadline) {
        auto d = apply(Kind::Recover, "retry_whitelist_NAV_ABORTED");
        if (d.changed) {
          return apply(Kind::Retry, "retry_shared_task_budget");
        }
      }
      return apply(Kind::Resolve, "termination_A_D_goal_terminal");
    }
    if ((r.state == State::Navigating || r.state == State::Returning) && r.arrived
        && now_ms() < r.arrival_until) {
      return apply(Kind::Resolve, "goal_arrival_A_D");
    }
    return {};
  }

  DeliveryApi::Response delivery_response(Store &s, const std::string &mission_id, DeliveryReply reply) {
    DeliveryApi::Response out;
    out.code = reply.result_code;
    out.mission_id = mission_id;
    auto reg = reply.registration ? reply.registration : s.get_registration_from_id(mission_id);
    if (reg) {
      out.queue_state = reg->queue_state;
      out.queue_seq = reg->queue_seq;
    }
    out.schedule_paused = s.queue_snapshot().at("schedule_paused");
    auto r = reply.record ? reply.record : (reg ? reg->record : std::optional<Record>{});
    if (r) {
      out.state = state_name(r->state);
      out.state_version = r->version;
      out.operation_id = r->operation;
      out.attempt_id = r->attempt;
      out.goal_uuid = r->uuid;
      out.handover_id = r->handover_id;
      out.handover_entry_version = r->handover_version;
      out.handed_over = r->handed_over;
      out.handover_source = r->handover_source;
      out.task_remaining_ms =
          r->restart_suspended ? 0 : std::max<std::int64_t>(0, r->deadline - now_ms());
      out.handover_remaining_ms = std::max<std::int64_t>(0, r->handover_until - now_ms());
    }
    out.detail = Json{{"queue", s.queue_snapshot()},
                      {"duplicate", reply.duplicate},
                      {"record", r ? Json(encode(*r)) : Json(nullptr)},
                      {"battery", "unavailable"},
                      {"synthetic_observations", delivery_synthetic}}
                     .dump();
    return out;
  }

  void start_delivery_api() {
    auto delivery_qos = rmw_qos_profile_services_default;
    delivery_qos.depth = 64;
    delivery_api = create_service<DeliveryApi>(
        "delivery",
        [this](std::shared_ptr<rmw_request_id_t> h, DeliveryApi::Request::SharedPtr q) {
          const auto receive = ++seq;
          std::cout
              << "DELIVERY_RX "
              << Json{{"receive_seq", receive},        {"operation", q->operation},
                      {"mission_id", q->mission_id},   {"command_id", q->command_id},
                      {"handover_id", q->handover_id}, {"source", q->source},
                      {"map_version", q->map_version}, {"station_version", q->station_version}}
                     .dump()
              << std::endl;
          if (closing || delivery_requests.size() >= 64) {
            DeliveryApi::Response out;
            out.code = closing ? "CLOSED" : "OVERLOADED";
            std::cout << "DELIVERY_REPLY "
                      << Json{{"receive_seq", receive}, {"code", out.code}}.dump() << std::endl;
            delivery_api->send_response(*h, out);
            return;
          }
          try {
            delivery_requests.push_back(
                {h, receive, runtime->call([this, q](Store &s) {
                   DeliveryApi::Response out;
                   try {
                     if (closing) {
                       out.code = "CLOSED";
                       return out;
                     }
                     DeliveryReply reply;
                     if (q->operation == "submit") {
                       MissionRequest request{q->schema_version,    q->mission_id,
                                              q->delivery_station,  q->return_station,
                                              q->map_version,       q->station_version,
                                              q->created_at_unix_ms};
                       Json frozen;
                       try {
                         frozen = catalog->freeze(request);
                       } catch (const std::exception &e) {
                         out.code = "INVALID_STATION_VERSION";
                         out.detail = e.what();
                         return out;
                       }
                       reply = s.submit_delivery(request, frozen);
                     } else if (q->operation == "query") {
                       reply.result_code = s.get_registration_from_id(q->mission_id) ? "FOUND" : "NOT_FOUND";
                     } else if (q->operation == "queue") {
                       reply.result_code = "QUEUE";
                     } else if (q->operation == "cancel") {
                       reply = s.cancel_delivery(q->mission_id, q->command_id);
                     } else if (q->operation == "pause_schedule") {
                       reply = s.schedule(false);
                     } else if (q->operation == "resume_schedule") {
                       reply = s.schedule(true);
                     } else if (q->operation == "handover") {
                       auto record = s.get_record_from_id(q->mission_id);
                       reply = s.confirm_handover(q->mission_id, q->command_id, q->handover_id,
                                                  q->handover_entry_version, q->source,
                                                  record && delivery_evidence(*record));
                     } else {
                       out.code = "INVALID_OPERATION";
                       return out;
                     }
                     s.dispatch();
                     auto response = delivery_response(s, q->mission_id, reply);
                     std::cout << "DELIVERY_DECISION "
                               << Json{{"operation", q->operation},
                                       {"mission_id", q->mission_id},
                                       {"command_id", q->command_id},
                                       {"code", response.code},
                                       {"state", response.state},
                                       {"version", response.state_version},
                                       {"queue_state", response.queue_state},
                                       {"handover_source", response.handover_source}}
                                      .dump()
                               << std::endl;
                     return response;
                   } catch (const std::exception &error) {
                     out.code = "STORAGE_ERROR";
                     out.detail = error.what();
                     return out;
                   }
                 })});
          } catch (const std::exception &error) {
            DeliveryApi::Response out;
            out.code = "OVERLOADED";
            out.detail = error.what();
            delivery_api->send_response(*h, out);
          }
        },
        delivery_qos);
  }

private:
  // Runs inside Runtime's owner task, never in the ROS callback thread.
  Decision apply_callback(Store &s, Event &e) {
    // Correlate identity and consume the original receipt-relative evidence lifetime.
    e.now = now_ms();
    if (auto current = s.get_record_from_id(e.mission_id)) {
      e = scope_callback(std::move(e), *current);
      if (gate_mode && e.uuid.empty()) {
        e.uuid = current->uuid;
      }
    }
    if (e.kind == Kind::Stop && e.execution_verified) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               Clock::now().time_since_epoch()
                               - std::chrono::nanoseconds(e.source_received_steady_ns))
                               .count();
      e.valid_until = e.now + 200 - elapsed;
      auto record = s.get_record_from_id(e.mission_id);
      if (e.source_received_steady_ns <= 0 || elapsed < 0 || elapsed >= 200 || !record
          || !execution->evidence_current(*record, e.source_generation)) {
        e.source_fresh = false;
      }
    }

    // Persist the decision before dispatching effects and advancing the workflow.
    auto d = s.apply(e);
    if (gate_mode && d.changed && e.kind == Kind::Stop) {
      stop_generations[e.attempt] = e.source_generation;
      if (delivery_mode) {
        delivery_windows[e.attempt].stop = e;
      }
    }
    if (gate_mode && d.changed && e.kind == Kind::ExecutionClock) {
      stop_generations.erase(e.attempt);
    }

    s.dispatch();
    if (delivery_mode) {
      delivery_drive(s);
    } else if (gate_mode) {
      auto r = s.get_active_record();
      if (r && r->state == State::Paused && !r->pending.empty() && r->goal_terminal && r->revoked
          && r->stopped && e.now < r->stop_until && stop_generations.count(r->attempt)
          && execution->evidence_current(*r, stop_generations[r->attempt])) {
        auto resolve =
            ev(*r, Kind::Resolve, "execution-resolve-" + epoch + "-" + std::to_string(r->version));
        resolve.detail = "verified A+D and Goal terminal";
        s.apply(resolve);
        s.dispatch();
      }
    }
    return d;
  }

  void start_command_api() {
    api = create_service<Api>("command", [this](std::shared_ptr<rmw_request_id_t> h,
                                                Api::Request::SharedPtr q) {
      std::cout << "BUSINESS_RX seq=" << ++seq << " operation=" << q->operation
                << " command=" << q->command_id << " mission=" << q->mission_id << std::endl;
      if (closing || requests.size() >= 64) {
        Api::Response r;
        r.code = closing ? "CLOSED" : "OVERLOADED";
        api->send_response(*h, r);
        return;
      }
      try {
        requests.push_back(
            {h, runtime->call([this, q](Store &s) {
               Api::Response response;
               try {
                 if (closing) {
                   response.code = "CLOSED";
                   return response;
                 }
                 if (delivery_mode && q->operation != "query") {
                   response.code = "DELIVERY_API_REQUIRED";
                   return response;
                 }
                 if (q->operation == "submit") {
                   if (!s.get_record_from_id(q->mission_id)
                       && (q->delivery_station != "a" || q->return_station != "b"
                           || q->map_version != "map" || q->station_version != "stations")) {
                     response.code = "INVALID_STATION_VERSION";
                     return response;
                   }
                   MissionRequest request{q->schema_version,    q->mission_id,  q->delivery_station,
                                          q->return_station,    q->map_version, q->station_version,
                                          q->created_at_unix_ms};
                   const auto decision = s.submit_legacy(request);
                   static const char *codes[] = {
                       "INVALID", "INVALID_CONTEXT", "EXISTING", "CONFLICT", "BUSY", "ACCEPTED"};
                   response.code = codes[static_cast<int>(decision)];
                   if (decision == Admission::CreateIntent) {
                     if (gate_mode) {
                       stop_generations.clear();
                     }
                     auto r = *s.get_record_from_id(q->mission_id);
                     auto e = ev(r, Kind::Prepare, "prepare-" + epoch + "-" + q->mission_id);
                     s.apply(e);
                     s.dispatch();
                   }
                 } else if (q->operation == "query") {
                   response.code = "FOUND";
                 } else if (q->operation == "cancel" || q->operation == "handover") {
                   if (!valid_identifier(q->command_id)) {
                     response.code = "INVALID_COMMAND_ID";
                     return response;
                   }
                   auto record = s.get_record_from_id(q->mission_id);
                   if (!record) {
                     response.code = "NOT_FOUND";
                     return response;
                   }
                   auto e = ev(*record, q->operation == "cancel" ? Kind::Cancel : Kind::Handover,
                               "api-" + q->command_id);
                   e.expected_version = q->expected_state_version;
                   e.detail = "API " + q->operation
                              + " expected=" + std::to_string(q->expected_state_version);
                   auto d = s.apply(e);
                   response.code = d.reason == "COMMAND_CONFLICT" ? "COMMAND_CONFLICT"
                                   : terminal(d.next.state)       ? "ALREADY_TERMINAL"
                                   : (d.changed || d.reason == "DUPLICATE")
                                       ? (q->operation == "cancel" ? "CANCEL_PENDING" : "CONFIRMED")
                                       : "REJECTED";
                   s.dispatch();
                 } else {
                   response.code = "INVALID_OPERATION";
                   return response;
                 }
                 if (auto record = s.get_record_from_id(q->mission_id)) {
                   fill(response, *record);
                 } else if (q->operation == "query") {
                   response.code = "NOT_FOUND";
                 }
               } catch (const std::exception &e) {
                 response.code = "STORAGE_ERROR";
                 response.detail = e.what();
               }
               return response;
             })});
      } catch (const std::exception &e) {
        Api::Response response;
        response.code = "OVERLOADED";
        response.detail = e.what();
        api->send_response(*h, response);
      }
    });
  }

  void start_fixture_api() {
    fixture = create_service<Fixture>("synthetic_event", [this](std::shared_ptr<rmw_request_id_t> h,
                                                                Fixture::Request::SharedPtr q) {
      if (closing || fixtures.size() >= 32) {
        Fixture::Response r;
        r.code = "OVERLOADED";
        fixture->send_response(*h, r);
        return;
      }
      try {
        fixtures.push_back({h, runtime->call([this, q](Store &s) {
                              Fixture::Response response;
                              try {
                                if (delivery_mode) {
                                  response.code = "FIXTURE_PROHIBITED";
                                  return response;
                                }
                                if (!q->synthetic) {
                                  response.code = "SYNTHETIC_REQUIRED";
                                  return response;
                                }
                                if (q->kind == "owner_pause") {
                                  if (q->valid_for_ms <= 0 || q->valid_for_ms > 1000) {
                                    response.code = "INVALID";
                                    return response;
                                  }
                                  std::cout << "OWNER_PAUSE_BEGIN " << q->valid_for_ms << std::endl;
                                  std::this_thread::sleep_for(
                                      std::chrono::milliseconds(q->valid_for_ms));
                                  std::cout << "OWNER_PAUSE_END" << std::endl;
                                  response.code = "APPLIED";
                                  return response;
                                }
                                if (gate_mode) {
                                  response.code = "FIXTURE_STOP_PROHIBITED";
                                  return response;
                                }
                                auto record = s.get_record_from_id(q->mission_id);
                                if (!record) {
                                  response.code = "NOT_FOUND";
                                  return response;
                                }
                                Kind kind;
                                if (q->kind == "stop") {
                                  kind = Kind::Stop;
                                } else if (q->kind == "arrival") {
                                  kind = Kind::Arrival;
                                } else if (q->kind == "resolve") {
                                  kind = Kind::Resolve;
                                } else if (q->kind == "revoke") {
                                  kind = Kind::RevokeAck;
                                } else if (q->kind == "fatal") {
                                  kind = Kind::Fatal;
                                } else {
                                  response.code = "INVALID";
                                  return response;
                                }
                                auto e = ev(*record, kind, q->event_id);
                                e.epoch = q->epoch;
                                e.operation = q->operation_id;
                                e.attempt = q->attempt_id;
                                e.uuid = q->goal_uuid;
                                e.gate_epoch = q->gate_epoch;
                                e.clock_epoch = q->clock_epoch;
                                e.synthetic = true;
                                e.observed_from = e.now - 1;
                                e.observed_to = e.now;
                                e.valid_until = e.now + q->valid_for_ms;
                                e.detail = "synthetic=true fixture";
                                if (e.epoch != record->epoch) {
                                  response.code = "IDENTITY";
                                  return response;
                                }
                                if (e.operation != record->operation || e.attempt != record->attempt
                                    || e.uuid != record->uuid) {
                                  response.code = "OPERATION";
                                  return response;
                                }
                                auto d = s.apply(e);
                                response.code = d.changed ? "APPLIED" : d.reason;
                                response.state = state_name(d.next.state);
                                response.detail = d.reason;
                                s.dispatch();
                              } catch (const std::exception &error) {
                                response.code = "STORAGE_ERROR";
                                response.detail = error.what();
                              }
                              return response;
                            })});
      } catch (const std::exception &e) {
        Fixture::Response r;
        r.code = "OVERLOADED";
        r.detail = e.what();
        fixture->send_response(*h, r);
      }
    });
  }

public:
  void start_node() {
    // Declare and validate the frozen configuration before constructing runtime owners.
    const bool test_mode = declare_parameter("test_mode", false);
    delivery_synthetic = test_mode;
    const auto workflow = declare_parameter<std::string>("workflow_mode", "legacy");
    delivery_mode = workflow == "delivery";
    if (workflow != "legacy" && workflow != "delivery") {
      throw std::runtime_error("invalid workflow_mode");
    }
    const auto catalog_path = declare_parameter<std::string>("station_catalog", "");
    const auto queue_capacity = declare_parameter("queue_capacity", 4);
    const auto task_ms = declare_parameter("task_ms", 240000);
    const auto handover_ms = declare_parameter("handover_ms", 120000);
    const auto action = declare_parameter<std::string>("action_name", "");
    const auto db = declare_parameter<std::string>("database", "");
    epoch = declare_parameter<std::string>("process_epoch", "");
    mailbox = static_cast<std::size_t>(declare_parameter("mailbox_capacity", 16));
    const auto execution_mode = declare_parameter<std::string>("execution_mode", "fixture");
    gate_mode = execution_mode == "gate";
    execution_run = declare_parameter<std::string>("execution_run_id", "");
    execution_ns = declare_parameter<std::string>("execution_namespace", get_namespace());
    if (execution_mode != "fixture" && execution_mode != "gate") {
      throw std::runtime_error("invalid execution mode");
    }
    if (gate_mode && (!valid_identifier(execution_run) || execution_ns != get_namespace())) {
      throw std::runtime_error("execution run/namespace mismatch");
    }
    AdapterConfig c;
    c.synthetic_revoke = !gate_mode;
    c.action = action;
    c.epoch = epoch;
    c.readiness_ms = declare_parameter("readiness_ms", 1500);
    c.accept_ms = declare_parameter("accept_ms", 1500);
    c.execute_ms = declare_parameter("execute_ms", 10000);
    c.cancel_ms = declare_parameter("cancel_ms", 1000);
    c.collect_ms = declare_parameter("collect_ms", 3000);
    const std::set<std::string> allowed = {
        "workflow_mode",  "station_catalog",  "queue_capacity",     "task_ms",
        "handover_ms",    "test_mode",        "action_name",        "database",
        "process_epoch",  "mailbox_capacity", "readiness_ms",       "accept_ms",
        "execute_ms",     "cancel_ms",        "collect_ms",         "use_sim_time",
        "execution_mode", "execution_run_id", "execution_namespace"};
    for (const auto &value : get_node_parameters_interface()->get_parameter_overrides()) {
      if (!allowed.count(value.first)) {
        throw std::runtime_error("unknown parameter: " + value.first);
      }
    }
    const char *domain = getenv("ROS_DOMAIN_ID");
    if (!test_mode && !delivery_mode) {
      throw std::runtime_error(
          "PRODUCTION_REFUSED: physical station/navigation policy not enabled; synthetic fixture prohibited");
    }
    if (db.empty() || !valid_identifier(epoch) || mailbox == 0 || mailbox > 256) {
      throw std::runtime_error("invalid owner configuration");
    }
    if (delivery_mode) {
      if (!gate_mode || !domain || std::string(domain) != (test_mode ? "72" : "71")
          || (test_mode ? action.rfind("/cmr_test/", 0) != 0 : action != "/navigate_to_pose")
          || catalog_path.empty() || queue_capacity < 0 || queue_capacity > 64
          || get_parameter("use_sim_time").as_bool()) {
        throw std::runtime_error("invalid explicit delivery domain/catalog/clock configuration");
      }
      catalog = std::make_unique<StationCatalog>(catalog_path);
      monitor = std::make_unique<DeliveryMonitor>(
          shared_from_this(), *catalog,
          DeliveryMonitorConfig{execution_run, epoch, execution_ns,
                                test_mode ? execution_ns : std::string(""), action, test_mode});
      c.simulation = !test_mode;
      c.goal_payload_required = true;
      c.source_now_ns = [this] { return monitor->source_time(); };
    } else if (!domain || std::string(domain) != "72" || action.rfind("/cmr_test/", 0) != 0) {
      throw std::runtime_error("invalid isolated test configuration");
    }

    // ROS callbacks enqueue owner work; service replies are collected later by poll().
    auto sink = [this](Event e) {
      if (events.size() >= 32) {
        return false;
      }
      try {
        const auto received = now_ms();
        events.push_back(
            runtime->call([this, e, received](Store &s) mutable { return apply_callback(s, e); }));
        return true;
      } catch (const std::exception &error) {
        std::cerr << "EVENT_MAILBOX " << error.what() << std::endl;
        return false;
      }
    };
    adapter = std::make_unique<NavigationAdapter>(shared_from_this(), c, sink);
    if (gate_mode) {
      execution = std::make_unique<ExecutionPort>(
          shared_from_this(),
          ExecutionConfig{execution_run, epoch, execution_ns, delivery_synthetic}, *adapter, sink);
    }
    StoreOptions store_options;
    if (delivery_mode) {
      store_options.delivery = true;
      store_options.queue_capacity = static_cast<std::size_t>(queue_capacity);
      store_options.task_ms = task_ms;
      store_options.handover_ms = handover_ms;
      store_options.now = [this] { return now_ms(); };
      store_options.eligible = [this](const Json &snapshot) { return delivery_eligible(snapshot); };
    }
    runtime = std::make_unique<Runtime>(db, epoch, mailbox,
                                        gate_mode ? static_cast<Port *>(execution.get())
                                                  : static_cast<Port *>(adapter.get()),
                                        Store::Hook{}, store_options);
    if (delivery_mode) {
      start_delivery_api();
    }
    start_command_api();
    start_fixture_api();
    timer = create_wall_timer(std::chrono::milliseconds(5), [this] { poll(); });
    if (delivery_mode) {
      frozen_parameters =
          add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &values) {
            rcl_interfaces::msg::SetParametersResult r;
            r.successful = values.empty();
            r.reason = "delivery configuration frozen; use transactional business API";
            return r;
          });
    }
    std::cout << "MISSION_READY epoch=" << epoch << " synthetic=" << delivery_synthetic
              << " delivery=" << delivery_mode << std::endl;
  }

  void poll() {
    if (monitor && adapter) {
      monitor->tick(now_ms(), adapter->ready());
      if (now_ms() - last_health_log >= 1000) {
        last_health_log = now_ms();
        auto h = monitor->snapshot();
        if (h) {
          std::cout << "DELIVERY_HEALTH "
                    << Json{{"qualified", h->healthy},
                            {"clock_epoch", h->clock_epoch},
                            {"source_ns", h->source_now_ns},
                            {"reason", h->reason}}
                           .dump()
                    << std::endl;
        }
      }
    }
    if (get_clock()->ros_time_is_active()) {
      const auto stamp = get_clock()->now().nanoseconds();
      if (stamp != last_ros_clock) {
        last_ros_clock = stamp;
        std::cout << "ROS_CLOCK_NS " << stamp << std::endl;
      }
    }

    // Preserve executor ordering: shutdown intent, ExecutionPort, owner health, then Nav2.
    if (closing) {
      queue_shutdown();
    }
    if (execution) {
      execution->tick();
    }
    if (gate_mode && !closing && !health_fault && !execution->faulted()) {
      if (health && health->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          health->get();
        } catch (const std::exception &e) {
          health_fault = true;
          std::cerr << "EXECUTION_OWNER_HEALTH_FAULT " << e.what() << std::endl;
        }
        health.reset();
      }
      if (!health && Clock::now() - health_at >= std::chrono::milliseconds(80)) {
        try {
          health = runtime->call([this](Store &s) {
            if (delivery_mode) {
              delivery_drive(s);
            }
            auto r = s.get_active_record();
            if (!r) {
              return Decision{};
            }
            auto e = ev(*r, Kind::ExecutionRenew,
                        "lease-health-" + epoch + "-" + std::to_string(r->version));
            e.execution_verified = true;
            e.detail = "owner transaction completed; renewal intent";
            auto d = s.apply(e);
            s.dispatch();
            return d;
          });
          health_at = Clock::now();
        } catch (const std::exception &e) {
          std::cerr << "EXECUTION_OWNER_HEALTH_DEFERRED " << e.what() << std::endl;
        }
      }
    }
    adapter->tick();

    // Only completed owner tasks produce service responses.
    for (auto it = requests.begin(); it != requests.end();) {
      if (it->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto r = it->future.get();
        api->send_response(*it->header, r);
        it = requests.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = delivery_requests.begin(); it != delivery_requests.end();) {
      if (it->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto r = it->future.get();
        std::cout << "DELIVERY_REPLY "
                  << Json{{"receive_seq", it->receipt},
                          {"code", r.code},
                          {"mission_id", r.mission_id},
                          {"state", r.state}}
                         .dump()
                  << std::endl;
        delivery_api->send_response(*it->header, r);
        it = delivery_requests.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = fixtures.begin(); it != fixtures.end();) {
      if (it->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto r = it->future.get();
        fixture->send_response(*it->header, r);
        it = fixtures.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = events.begin(); it != events.end();) {
      if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          auto d = it->get();
          std::cout << "OWNER_EVENT " << d.reason << " state=" << state_name(d.next.state)
                    << std::endl;
        } catch (const std::exception &e) {
          std::cerr << "OWNER_EVENT_ERROR " << e.what() << std::endl;
        }
        it = events.erase(it);
      } else {
        ++it;
      }
    }
  }

  void queue_shutdown() {
    if (shutdown_queued) {
      return;
    }
    try {
      events.push_back(runtime->call([this](Store &s) {
        auto r = s.get_active_record();
        if (!r) {
          return Decision{};
        }
        auto e = ev(*r, Kind::Cancel, "shutdown-" + epoch);
        e.detail = "shutdown";
        auto d = s.apply(e);
        s.dispatch();
        return d;
      }));
      shutdown_queued = true;
    } catch (const std::exception &e) {
      std::cerr << "SHUTDOWN_REQUEST_RETRY " << e.what() << std::endl;
    }
  }

  void begin_shutdown() {
    closing = true;
    if (execution) {
      execution->begin_shutdown();
    }
    adapter->begin_shutdown();
    queue_shutdown();
  }

  void finish() {
    if (execution) {
      execution->stop();
    }
    adapter->stop();
    runtime->close();
    poll();
    std::cout << "MISSION_CLOSED requests=" << requests.size() << " events=" << events.size()
              << std::endl;
  }
};
} // namespace cmr::mission

int main(int argc, char **argv) {
  using namespace cmr::mission;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try {
    auto node = std::make_shared<MissionNode>();
    node->start_node();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !interrupted) {
      executor.spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    node->begin_shutdown();
    const auto deadline = Clock::now() + std::chrono::milliseconds(1600);
    while (rclcpp::ok() && Clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    node->finish();
    executor.cancel();
    executor.remove_node(node);
    std::weak_ptr<MissionNode> lifetime = node;
    node.reset();
    if (!lifetime.expired()) {
      throw std::runtime_error("Mission Node retained after owned shutdown");
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "MISSION_FATAL " << e.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
}
