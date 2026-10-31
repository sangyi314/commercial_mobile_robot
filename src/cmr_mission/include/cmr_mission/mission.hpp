// SPDX-License-Identifier: MIT
#pragma once
#include "admission.hpp"
#include <vector>
#include <cstdint>
namespace cmr::mission {
enum class State { Accepted, Navigating, WaitHandover, Returning, Paused, Recovering, Completed, Failed, Canceled };
enum class Phase { None, SendPending, Active, StopPending, Uncertain };
enum class Kind { Prepare, Accepted, Succeeded, Aborted, Rejected, Cancel, CancelAck, RevokeAck, Stop, Arrival, Handover, Tick, Fatal, Recover, Retry, Resolve, Restart, Feedback };
enum class EffectKind { Send, CancelExact, Revoke };
struct Effect { std::string id, mission, operation, attempt, uuid; EffectKind kind; };
struct Record {
 MissionRequest request;
 State state{State::Accepted}; Phase phase{Phase::None};
 std::string epoch, operation, attempt, uuid, pending, reason, gate_epoch{"synthetic-gate"}, clock_epoch{"synthetic-clock"};
 std::uint64_t version{0}, seq{0}; int retries{0};
 bool returning{false}, handed_over{false}, goal_terminal{true}, revoked{true}, stopped{true}, arrived{false}, ever_sent{false};
 std::int64_t stop_until{0}, deadline{100000}, last_now{0};
};
struct Event {
 std::string id, mission, epoch, operation, attempt, uuid;
 Kind kind{Kind::Feedback}; std::int64_t now{0};
 std::uint64_t expected_version{0}; bool synthetic{true};
 std::string gate_epoch{"synthetic-gate"}, clock_epoch{"synthetic-clock"};
 std::int64_t observed_from{0}, observed_to{0}, valid_until{0};
};
struct Decision { Record next; std::vector<Effect> effects; bool changed{false}; std::string reason; };
bool terminal(State s);
const char* state_name(State s);
Decision reduce(const Record&, const Event&);
std::string encode(const Record&);
Record decode(const std::string&);
}
