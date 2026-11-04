// SPDX-License-Identifier: MIT
#pragma once
#include "admission.hpp"
#include <vector>
#include <cstdint>

namespace cmr::mission {
// Business lifecycle; Phase separately tracks the navigation execution boundary.
enum class State {
  Accepted,
  Navigating,
  WaitHandover,
  Returning,
  Paused,
  Recovering,
  Completed,
  Failed,
  Canceled
};
// Dispatch, active Goal, and stop/uncertainty bookkeeping within a business state.
enum class Phase { None, SendPending, Active, StopPending, Uncertain };
enum class Kind {
  Prepare,
  Accepted,
  Succeeded,
  Aborted,
  Rejected,
  Cancel,
  CancelAck,
  RevokeAck,
  Stop,
  Arrival,
  Handover,
  Tick,
  Fatal,
  Recover,
  Retry,
  Resolve,
  Restart,
  Feedback,
  CanceledResult,
  ExecutionBind,
  ExecutionRenew,
  ExecutionClock,
  ExecutionContext
};
enum class EffectKind {
  Send,
  CancelExact,
  Revoke,
  SendAuthorized,
  RenewLease,
  BeginLease,
  ResetLease
};

struct Effect {
  std::string id;
  std::string mission;
  std::string operation;
  std::string attempt;
  std::string uuid;
  EffectKind kind;
  std::string navigation_payload{};
};

struct Record {
  // Business state and current execution identity. Keep declaration order stable.
  MissionRequest request;
  State state{State::Accepted};
  Phase phase{Phase::None};
  std::string epoch;
  std::string operation;
  std::string attempt;
  std::string uuid;

  // Outstanding termination intent; pause() preserves the first committed intent.
  std::string pending;
  // Retained termination diagnosis; Recover clears pending without clearing reason.
  std::string reason;

  // Execution authority and record sequencing.
  std::string gate_epoch{"synthetic-gate"};
  std::string clock_epoch{"synthetic-clock"};
  std::uint64_t version{0};
  std::uint64_t seq{0};
  int retries{0};

  // Navigation and handover facts remain independent.
  bool returning{false};
  bool handed_over{false};
  // No unresolved Goal, including before the first send; does not prove arrival or stop.
  bool goal_terminal{true};
  // After a send, revocation acknowledgment and validated stop evidence are separate.
  // Both default to true before any Goal has been sent.
  bool revoked{true};
  bool stopped{true};
  // Arrival evidence is separate from Action success (verified in delivery mode).
  bool arrived{false};
  bool ever_sent{false};

  // Owner-clock validity, task deadline, and rollback guard.
  std::int64_t stop_until{0};
  std::int64_t deadline{100000};
  std::int64_t last_now{0};

  // Explicit delivery workflow; encode uses format3, while legacy mode retains format1.
  bool delivery_managed{false};
  // Action result was Succeeded; arrival and stop still require their own evidence.
  bool goal_succeeded{false};
  bool retry_eligible{false};
  // Delivery restart blocks progression through Prepare, Handover, and Recover.
  bool restart_suspended{false};

  // Delivery evidence windows and handover identity.
  std::int64_t arrival_until{0};
  std::int64_t terminal_ros_ns{0};
  std::int64_t handover_until{0};
  std::int64_t handover_budget{120000};
  std::uint64_t handover_version{0};
  std::string handover_id;
  std::string handover_source;
};

struct Event {
  // Mission/operation identity and event metadata.
  std::string id;
  std::string mission;
  std::string epoch;
  std::string operation;
  std::string attempt;
  std::string uuid;
  std::string detail;
  Kind kind{Kind::Feedback};
  std::int64_t now{0};

  // Business command version and synthetic-fixture marker.
  std::uint64_t expected_version{0};
  bool synthetic{true};

  // Evidence identity and owner-clock validity.
  std::string gate_epoch{"synthetic-gate"};
  std::string clock_epoch{"synthetic-clock"};
  std::int64_t observed_from{0};
  std::int64_t observed_to{0};
  std::int64_t valid_until{0};

  // Internal verified ExecutionPort boundary; not supplied by MissionCommand or fixture.
  bool execution_verified{false};
  bool continuous_zero{false};
  bool source_fresh{false};
  bool arrival_verified{false};

  // Source-clock sample span, freshness, and motion evidence.
  std::int64_t source_from_ns{0};
  std::int64_t source_to_ns{0};
  std::int64_t revoke_ros_ns{0};
  std::int64_t max_source_gap_ns{0};
  std::int64_t source_received_steady_ns{0};
  std::uint64_t source_samples{0};
  std::uint64_t source_generation{0};
  double drift_m{0};
  double yaw_drift_rad{0};
  std::string evidence_ref;
};

struct Decision {
  Record next;
  std::vector<Effect> effects;
  bool changed{false};
  std::string reason;
};

bool terminal(State s);
const char *state_name(State s);
Decision reduce(const Record &, const Event &);
std::string encode(const Record &);
Record decode(const std::string &);
} // namespace cmr::mission
