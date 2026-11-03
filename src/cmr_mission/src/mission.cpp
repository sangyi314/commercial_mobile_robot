// SPDX-License-Identifier: MIT
#include "cmr_mission/mission.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <algorithm>
namespace cmr::mission {
bool terminal(State s) { return s==State::Completed || s==State::Failed || s==State::Canceled; }
const char* state_name(State s) {
 static const char* names[]={"ACCEPTED","NAVIGATING","WAIT_HANDOVER","RETURNING","PAUSED","RECOVERING","COMPLETED","FAILED","CANCELED"};
 return names[static_cast<int>(s)];
}
Decision reduce(const Record& old, const Event& e) {
 Decision d{old,{},false,"ILLEGAL_EVENT"}; auto& n=d.next;
 if(e.id.empty() || e.mission!=old.request.mission_id || e.epoch!=old.epoch) {d.reason="IDENTITY"; return d;}
 if(terminal(old.state)) {d.reason="ALREADY_TERMINAL";return d;}
 if(e.kind!=Kind::Restart && e.now<old.last_now) {d.reason="TIME_ROLLBACK";return d;}
 auto effect=[&](EffectKind k) {for(const auto& x:d.effects)if(x.kind==k)return;d.effects.push_back({old.request.mission_id+":"+std::to_string(old.version+1)+":"+std::to_string(static_cast<int>(k)),old.request.mission_id,n.operation,n.attempt,n.uuid,k});};
 auto pause=[&](const std::string& why) {
  n.state=State::Paused;
  // Business termination is first-writer-wins. A later navigation result is
  // still audited by Store::apply, but cannot replace a committed cancel.
  if(n.pending.empty()) { n.pending=why; n.reason=why; }
  effect(EffectKind::Revoke);
  if(!n.goal_terminal && !n.uuid.empty()) effect(EffectKind::CancelExact);
  n.phase=n.goal_terminal?Phase::StopPending:Phase::Uncertain;
 };
 auto clean=[&] {return n.goal_terminal && n.revoked && n.stopped && (!n.ever_sent || e.now<n.stop_until);};
 const bool nav=e.kind==Kind::ExecutionContext || e.kind==Kind::ExecutionBind || e.kind==Kind::ExecutionRenew || e.kind==Kind::ExecutionClock || e.kind==Kind::CanceledResult || e.kind==Kind::Accepted || e.kind==Kind::Succeeded || e.kind==Kind::Aborted || e.kind==Kind::Rejected || e.kind==Kind::Stop || e.kind==Kind::RevokeAck || e.kind==Kind::Arrival || e.kind==Kind::CancelAck || e.kind==Kind::Feedback;
 if(nav && (e.operation!=old.operation || e.attempt!=old.attempt || (e.kind!=Kind::Accepted && !old.uuid.empty() && e.uuid!=old.uuid))) {d.reason="OPERATION";return d;}
 switch(e.kind) {
 case Kind::Prepare:
 case Kind::Retry:
  if(old.delivery_managed&&old.restart_suspended)return d;
  if(!((e.kind==Kind::Prepare && old.state==State::Accepted) || (e.kind==Kind::Retry && old.state==State::Recovering)) || !clean() || old.phase==Phase::SendPending || e.now>=old.deadline) return d;
  n.operation=old.request.mission_id+(n.returning?".return":".out"); n.attempt=n.operation+"."+std::to_string(n.retries); n.uuid.clear();
  n.goal_terminal=false;n.goal_succeeded=false;n.retry_eligible=false;n.arrival_until=0;n.terminal_ros_ns=0;n.revoked=false;n.stopped=false;n.arrived=false;n.ever_sent=true;n.phase=Phase::SendPending;effect(EffectKind::Send);break;
 case Kind::Accepted:
  if(old.goal_terminal || e.uuid.empty() || (!old.uuid.empty() && old.uuid!=e.uuid)) return d;
  if(old.uuid==e.uuid) {d.reason="DUPLICATE_ACCEPT";return d;}
  n.uuid=e.uuid;
  if(old.state==State::Paused) {effect(EffectKind::CancelExact);n.phase=Phase::Uncertain;}
  else {n.state=n.returning?State::Returning:State::Navigating;n.phase=Phase::Active;}break;
 case Kind::Succeeded:
 case Kind::CanceledResult:
 case Kind::Aborted:
 case Kind::Rejected:
  if(old.goal_terminal) return d;
  if(e.kind!=Kind::Rejected && old.uuid.empty()) return d;
  n.goal_terminal=true;n.retry_eligible=e.kind==Kind::Aborted&&old.pending.empty();n.goal_succeeded=e.kind==Kind::Succeeded;n.terminal_ros_ns=e.source_to_ns;n.phase=Phase::StopPending;effect(EffectKind::Revoke);
  if(e.kind!=Kind::Succeeded) {pause("FAILED");}
  break;
 case Kind::Cancel:
 case Kind::Fatal:
  if(!old.pending.empty()) {d.reason="TERMINATION_PENDING";return d;}
  n.retry_eligible=false;pause(e.kind==Kind::Cancel?"CANCELED":"FAILED");break;
 case Kind::Tick:
  if(e.now<(old.delivery_managed&&old.state==State::WaitHandover&&!old.handed_over?std::min(old.deadline,old.handover_until):old.deadline) || !old.pending.empty()) return d;
  n.retry_eligible=false;pause("FAILED");break;
 case Kind::CancelAck: d.reason="ACK_ONLY";return d;
 case Kind::RevokeAck: if(old.revoked) return d; if(old.gate_epoch!="synthetic-gate"&&(!e.execution_verified||e.gate_epoch!=old.gate_epoch||e.clock_epoch!=old.clock_epoch))return d; n.revoked=true;break;
 case Kind::Stop: {
  const bool offline=old.gate_epoch=="synthetic-gate"&&old.clock_epoch=="synthetic-clock";
  const bool verified=e.execution_verified&&e.continuous_zero&&e.source_fresh&&e.source_from_ns>0&&e.source_from_ns>=e.revoke_ros_ns&&e.source_to_ns-e.source_from_ns>=3000000000LL&&e.source_samples>=5&&e.max_source_gap_ns>=0&&e.max_source_gap_ns<=500000000&&std::isfinite(e.drift_m)&&std::isfinite(e.yaw_drift_rad)&&e.drift_m>=0&&e.drift_m<=.02&&e.yaw_drift_rad>=0&&e.yaw_drift_rad<=.03&&!e.evidence_ref.empty();
  if((offline?(!e.synthetic||e.observed_from>=e.observed_to||e.observed_to>e.now||e.observed_from<old.last_now):!verified)||e.gate_epoch!=old.gate_epoch||e.clock_epoch!=old.clock_epoch||e.valid_until<=e.now) {d.reason="STOP_EVIDENCE";return d;}
  n.stopped=true;n.stop_until=e.valid_until;break;
 }
 case Kind::ExecutionContext:
  if(!e.execution_verified||e.gate_epoch.empty()||e.clock_epoch.empty()||!old.uuid.empty()||(old.goal_terminal&&!(old.state==State::Paused&&!old.pending.empty()))||!(old.phase==Phase::SendPending||old.state==State::Paused))return d;
  n.gate_epoch=e.gate_epoch;n.clock_epoch=e.clock_epoch;n.stopped=false;n.stop_until=0;
  effect(old.pending.empty()?EffectKind::BeginLease:EffectKind::Revoke);break;
 case Kind::ExecutionBind:
  if(!e.execution_verified||!old.pending.empty()||old.phase!=Phase::SendPending||old.goal_terminal||!old.uuid.empty()||old.gate_epoch=="synthetic-gate"||e.gate_epoch!=old.gate_epoch||e.clock_epoch!=old.clock_epoch)return d;
  effect(EffectKind::SendAuthorized);break;
 case Kind::ExecutionRenew:
  if(!e.execution_verified||!old.pending.empty()||old.state==State::Paused||old.goal_terminal||old.gate_epoch=="synthetic-gate"||!(old.phase==Phase::SendPending||old.phase==Phase::Active))return d;
  effect(EffectKind::RenewLease);break;
 case Kind::ExecutionClock:
  if(!e.execution_verified||e.gate_epoch!=old.gate_epoch||e.clock_epoch.empty()||e.clock_epoch==old.clock_epoch)return d;
  n.clock_epoch=e.clock_epoch;n.stopped=false;n.stop_until=0;n.revoked=false;n.retry_eligible=false;pause("FAILED");break;
 case Kind::Arrival:
  if(!old.goal_terminal || old.state==State::Paused) return d;
  if(old.delivery_managed&&(!old.goal_succeeded||!e.arrival_verified||!e.source_fresh||e.gate_epoch!=old.gate_epoch||e.clock_epoch!=old.clock_epoch||e.source_from_ns<old.terminal_ros_ns||e.valid_until<=e.now)) {d.reason="ARRIVAL_EVIDENCE";return d;}
  n.arrived=true;n.arrival_until=e.valid_until;break;
 case Kind::Resolve:
  if(!clean()) {d.reason="STOP_OR_GOAL_UNKNOWN";return d;}
  if(old.state==State::Paused && !old.pending.empty()) {n.state=old.pending=="CANCELED"?State::Canceled:State::Failed;n.phase=Phase::None;}
  else if((old.state==State::Navigating || old.state==State::Returning) && old.arrived && (!old.delivery_managed||(old.goal_succeeded&&e.now<old.arrival_until))) {n.state=n.returning?State::Completed:State::WaitHandover;n.phase=Phase::None;
   if(old.delivery_managed&&!n.returning){n.handover_version=old.version+1;n.handover_id=old.request.mission_id.substr(0,32)+".h."+std::to_string(n.handover_version);n.handover_until=std::min(old.deadline,e.now+old.handover_budget);}}

  else return d;
  if(old.delivery_managed&&terminal(n.state))effect(EffectKind::ResetLease);
  break;
 case Kind::Handover:
  if(old.delivery_managed&&old.restart_suspended)return d;
  if(old.state!=State::WaitHandover || old.handed_over || e.expected_version!=old.version || e.now>=old.deadline || !clean() || (old.delivery_managed&&(e.now>=old.handover_until||e.now>=old.arrival_until))) return d;
  if(old.delivery_managed)d.effects.push_back({old.request.mission_id+":"+std::to_string(old.version+1)+":reset",old.request.mission_id,old.operation,old.attempt,old.uuid,EffectKind::ResetLease});
  n.handed_over=true;n.handover_source=e.detail;n.returning=true;n.operation=old.request.mission_id+".return";n.attempt=n.operation+".0";n.uuid.clear();
  n.goal_terminal=false;n.goal_succeeded=false;n.retry_eligible=false;n.arrival_until=0;n.terminal_ros_ns=0;n.revoked=false;n.stopped=false;n.arrived=false;n.ever_sent=true;n.phase=Phase::SendPending;effect(EffectKind::Send);break;
 case Kind::Recover:
  if(old.state!=State::Paused || old.pending!="FAILED" || old.retries>=1 || (old.delivery_managed&&(!old.retry_eligible||old.restart_suspended)) || !clean() || e.now>=old.deadline) return d;
  n.state=State::Recovering;n.pending.clear();++n.retries;n.phase=Phase::None;if(old.delivery_managed)effect(EffectKind::ResetLease);break;
 case Kind::Restart:
  n.state=State::Paused;n.phase=old.ever_sent?Phase::Uncertain:Phase::None;
  n.stopped=!old.ever_sent;n.stop_until=0;n.last_now=0;n.retry_eligible=false;n.restart_suspended=old.delivery_managed;break;
 case Kind::Feedback: d.reason="FEEDBACK_ONLY";return d;
 }
 n.version++;n.seq++;if(e.kind!=Kind::Restart)n.last_now=e.now;
 d.changed=true;d.reason="APPLIED";return d;
}
std::string encode(const Record& r) {
 std::ostringstream o; o<<(r.delivery_managed?3:1)<<' '<<r.request.schema_version<<' '<<std::quoted(r.request.mission_id)<<' '<<std::quoted(r.request.delivery_station)<<' '<<std::quoted(r.request.return_station)<<' '<<std::quoted(r.request.map_version)<<' '<<std::quoted(r.request.station_version)<<' '<<r.request.created_at_unix_ms<<' '<<static_cast<int>(r.state)<<' '<<static_cast<int>(r.phase)<<' ';
 for(const auto* s:{&r.epoch,&r.operation,&r.attempt,&r.uuid,&r.pending,&r.reason,&r.gate_epoch,&r.clock_epoch})o<<std::quoted(*s)<<' ';
 o<<r.version<<' '<<r.seq<<' '<<r.retries<<' '<<r.returning<<' '<<r.handed_over<<' '<<r.goal_terminal<<' '<<r.revoked<<' '<<r.stopped<<' '<<r.arrived<<' '<<r.ever_sent<<' '<<r.stop_until<<' '<<r.deadline<<' '<<r.last_now;
 if(r.delivery_managed)o<<' '<<r.retry_eligible<<' '<<r.goal_succeeded<<' '<<r.arrival_until<<' '<<r.terminal_ros_ns<<' '<<r.handover_until<<' '<<r.handover_budget<<' '<<r.handover_version<<' '<<std::quoted(r.handover_id)<<' '<<std::quoted(r.handover_source)<<' '<<r.restart_suspended;
 return o.str();
}
Record decode(const std::string& text) {
 Record r;std::istringstream i(text);int format,s,p;
 i>>format>>r.request.schema_version>>std::quoted(r.request.mission_id)>>std::quoted(r.request.delivery_station)>>std::quoted(r.request.return_station)>>std::quoted(r.request.map_version)>>std::quoted(r.request.station_version)>>r.request.created_at_unix_ms>>s>>p;
 for(auto* x:{&r.epoch,&r.operation,&r.attempt,&r.uuid,&r.pending,&r.reason,&r.gate_epoch,&r.clock_epoch})i>>std::quoted(*x);
 i>>r.version>>r.seq>>r.retries>>r.returning>>r.handed_over>>r.goal_terminal>>r.revoked>>r.stopped>>r.arrived>>r.ever_sent>>r.stop_until>>r.deadline>>r.last_now;
 if(format==2||format==3){r.delivery_managed=true;const auto pos=i.tellg();i>>r.retry_eligible>>r.goal_succeeded>>r.arrival_until>>r.terminal_ros_ns>>r.handover_until>>r.handover_budget>>r.handover_version>>std::quoted(r.handover_id)>>std::quoted(r.handover_source);
  if(!i&&format==2){i.clear();i.seekg(pos);r.retry_eligible=false;i>>r.goal_succeeded>>r.arrival_until>>r.terminal_ros_ns>>r.handover_until>>r.handover_budget>>r.handover_version>>std::quoted(r.handover_id)>>std::quoted(r.handover_source);}
  if(format==3)i>>r.restart_suspended;
 }
 if(!i || (format!=1&&format!=2&&format!=3) || s<0 || s>8 || p<0 || p>4 || validate(r.request)!=Validation::Valid)throw std::runtime_error("corrupt record");
 i>>std::ws;if(!i.eof())throw std::runtime_error("trailing record data");r.state=static_cast<State>(s);r.phase=static_cast<Phase>(p);return r;
}
}
