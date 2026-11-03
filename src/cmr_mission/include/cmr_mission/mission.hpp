// SPDX-License-Identifier: MIT
#pragma once
#include "admission.hpp"
#include <vector>
#include <cstdint>
namespace cmr::mission {
enum class State { Accepted, Navigating, WaitHandover, Returning, Paused, Recovering, Completed, Failed, Canceled };
enum class Phase { None, SendPending, Active, StopPending, Uncertain };
enum class Kind { Prepare, Accepted, Succeeded, Aborted, Rejected, Cancel, CancelAck, RevokeAck, Stop, Arrival, Handover, Tick, Fatal, Recover, Retry, Resolve, Restart, Feedback, CanceledResult, ExecutionBind, ExecutionRenew, ExecutionClock, ExecutionContext };
enum class EffectKind { Send, CancelExact, Revoke, SendAuthorized, RenewLease, BeginLease, ResetLease };
struct Effect { std::string id, mission, operation, attempt, uuid; EffectKind kind; std::string navigation_payload{}; };
struct Record {
 MissionRequest request;
 State state{State::Accepted}; Phase phase{Phase::None};
 std::string epoch, operation, attempt, uuid, pending, reason, gate_epoch{"synthetic-gate"}, clock_epoch{"synthetic-clock"};
 std::uint64_t version{0}, seq{0}; int retries{0};
 bool returning{false}, handed_over{false}, goal_terminal{true}, revoked{true}, stopped{true}, arrived{false}, ever_sent{false};
 std::int64_t stop_until{0}, deadline{100000}, last_now{0};
 // Format2 belongs to explicit delivery mode only; format1 behavior stays compatible.
 bool delivery_managed{false}, goal_succeeded{false}, retry_eligible{false},restart_suspended{false};
 std::int64_t arrival_until{0}, terminal_ros_ns{0}, handover_until{0}, handover_budget{120000};
 std::uint64_t handover_version{0};std::string handover_id,handover_source;
};
struct Event {
 std::string id, mission, epoch, operation, attempt, uuid, detail;
 Kind kind{Kind::Feedback}; std::int64_t now{0};
 std::uint64_t expected_version{0}; bool synthetic{true};
 std::string gate_epoch{"synthetic-gate"}, clock_epoch{"synthetic-clock"};
 std::int64_t observed_from{0}, observed_to{0}, valid_until{0};
 // Internal verified ExecutionPort boundary; not supplied by MissionCommand or fixture.
 bool execution_verified{false}, continuous_zero{false}, source_fresh{false}, arrival_verified{false};
 std::int64_t source_from_ns{0},source_to_ns{0},revoke_ros_ns{0},max_source_gap_ns{0},source_received_steady_ns{0};
 std::uint64_t source_samples{0},source_generation{0};double drift_m{0},yaw_drift_rad{0};std::string evidence_ref;
};
struct Decision { Record next; std::vector<Effect> effects; bool changed{false}; std::string reason; };
bool terminal(State s);
const char* state_name(State s);
Decision reduce(const Record&, const Event&);
std::string encode(const Record&);
Record decode(const std::string&);
}
