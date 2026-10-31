// SPDX-License-Identifier: MIT
#include "cmr_mission/mission.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>
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
  n.state=State::Paused; n.pending=why; n.reason=why;
  effect(EffectKind::Revoke);
  if(!n.goal_terminal && !n.uuid.empty()) effect(EffectKind::CancelExact);
  n.phase=n.goal_terminal?Phase::StopPending:Phase::Uncertain;
 };
 auto clean=[&] {return n.goal_terminal && n.revoked && n.stopped && (!n.ever_sent || e.now<n.stop_until);};
 const bool nav=e.kind==Kind::Accepted || e.kind==Kind::Succeeded || e.kind==Kind::Aborted || e.kind==Kind::Rejected || e.kind==Kind::Stop || e.kind==Kind::RevokeAck || e.kind==Kind::Arrival || e.kind==Kind::CancelAck || e.kind==Kind::Feedback;
 if(nav && (e.operation!=old.operation || e.attempt!=old.attempt || (e.kind!=Kind::Accepted && !old.uuid.empty() && e.uuid!=old.uuid))) {d.reason="OPERATION";return d;}
 switch(e.kind) {
 case Kind::Prepare:
 case Kind::Retry:
  if(!((e.kind==Kind::Prepare && old.state==State::Accepted) || (e.kind==Kind::Retry && old.state==State::Recovering)) || !clean() || old.phase==Phase::SendPending || e.now>=old.deadline) return d;
  n.operation=old.request.mission_id+(n.returning?".return":".out"); n.attempt=n.operation+"."+std::to_string(n.retries); n.uuid.clear();
  n.goal_terminal=false;n.revoked=false;n.stopped=false;n.arrived=false;n.ever_sent=true;n.phase=Phase::SendPending;effect(EffectKind::Send);break;
 case Kind::Accepted:
  if(old.goal_terminal || e.uuid.empty() || (!old.uuid.empty() && old.uuid!=e.uuid)) return d;
  if(old.uuid==e.uuid) {d.reason="DUPLICATE_ACCEPT";return d;}
  n.uuid=e.uuid;
  if(old.state==State::Paused) {effect(EffectKind::CancelExact);n.phase=Phase::Uncertain;}
  else {n.state=n.returning?State::Returning:State::Navigating;n.phase=Phase::Active;}break;
 case Kind::Succeeded:
 case Kind::Aborted:
 case Kind::Rejected:
  if(old.goal_terminal) return d;
  if(e.kind!=Kind::Rejected && old.uuid.empty()) return d;
  n.goal_terminal=true;n.phase=Phase::StopPending;effect(EffectKind::Revoke);
  if(e.kind!=Kind::Succeeded) pause("FAILED");break;
 case Kind::Cancel:
 case Kind::Fatal:
  if(!old.pending.empty()) {d.reason="TERMINATION_PENDING";return d;}
  pause(e.kind==Kind::Cancel?"CANCELED":"FAILED");break;
 case Kind::Tick:
  if(e.now<old.deadline || !old.pending.empty()) return d;
  pause("FAILED");break;
 case Kind::CancelAck: d.reason="ACK_ONLY";return d;
 case Kind::RevokeAck: if(old.revoked) return d; n.revoked=true;break;
 case Kind::Stop:
  if(!e.synthetic || e.gate_epoch!=old.gate_epoch || e.clock_epoch!=old.clock_epoch || e.observed_from>=e.observed_to || e.observed_to>e.now || e.valid_until<=e.now || e.observed_from<old.last_now) {d.reason="STOP_EVIDENCE";return d;}
  n.stopped=true;n.stop_until=e.valid_until;break;
 case Kind::Arrival: if(!old.goal_terminal || old.state==State::Paused) return d; n.arrived=true;break;
 case Kind::Resolve:
  if(!clean()) {d.reason="STOP_OR_GOAL_UNKNOWN";return d;}
  if(old.state==State::Paused && !old.pending.empty()) {n.state=old.pending=="CANCELED"?State::Canceled:State::Failed;n.phase=Phase::None;}
  else if((old.state==State::Navigating || old.state==State::Returning) && old.arrived) {n.state=n.returning?State::Completed:State::WaitHandover;n.phase=Phase::None;}
  else return d;
  break;
 case Kind::Handover:
  if(old.state!=State::WaitHandover || old.handed_over || e.expected_version!=old.version || e.now>=old.deadline || !clean()) return d;
  n.handed_over=true;n.returning=true;n.operation=old.request.mission_id+".return";n.attempt=n.operation+".0";n.uuid.clear();
  n.goal_terminal=false;n.revoked=false;n.stopped=false;n.arrived=false;n.ever_sent=true;n.phase=Phase::SendPending;effect(EffectKind::Send);break;
 case Kind::Recover:
  if(old.state!=State::Paused || old.pending!="FAILED" || old.retries>=1 || !clean() || e.now>=old.deadline) return d;
  n.state=State::Recovering;n.pending.clear();++n.retries;n.phase=Phase::None;break;
 case Kind::Restart:
  n.state=State::Paused;n.phase=old.ever_sent?Phase::Uncertain:Phase::None;
  n.stopped=!old.ever_sent;n.stop_until=0;n.last_now=0;break;
 case Kind::Feedback: d.reason="FEEDBACK_ONLY";return d;
 }
 n.version++;n.seq++;if(e.kind!=Kind::Restart)n.last_now=e.now;
 d.changed=true;d.reason="APPLIED";return d;
}
std::string encode(const Record& r) {
 std::ostringstream o; o<<1<<' '<<r.request.schema_version<<' '<<std::quoted(r.request.mission_id)<<' '<<std::quoted(r.request.delivery_station)<<' '<<std::quoted(r.request.return_station)<<' '<<std::quoted(r.request.map_version)<<' '<<std::quoted(r.request.station_version)<<' '<<r.request.created_at_unix_ms<<' '<<static_cast<int>(r.state)<<' '<<static_cast<int>(r.phase)<<' ';
 for(const auto* s:{&r.epoch,&r.operation,&r.attempt,&r.uuid,&r.pending,&r.reason,&r.gate_epoch,&r.clock_epoch})o<<std::quoted(*s)<<' ';
 o<<r.version<<' '<<r.seq<<' '<<r.retries<<' '<<r.returning<<' '<<r.handed_over<<' '<<r.goal_terminal<<' '<<r.revoked<<' '<<r.stopped<<' '<<r.arrived<<' '<<r.ever_sent<<' '<<r.stop_until<<' '<<r.deadline<<' '<<r.last_now;return o.str();
}
Record decode(const std::string& text) {
 Record r;std::istringstream i(text);int format,s,p;
 i>>format>>r.request.schema_version>>std::quoted(r.request.mission_id)>>std::quoted(r.request.delivery_station)>>std::quoted(r.request.return_station)>>std::quoted(r.request.map_version)>>std::quoted(r.request.station_version)>>r.request.created_at_unix_ms>>s>>p;
 for(auto* x:{&r.epoch,&r.operation,&r.attempt,&r.uuid,&r.pending,&r.reason,&r.gate_epoch,&r.clock_epoch})i>>std::quoted(*x);
 i>>r.version>>r.seq>>r.retries>>r.returning>>r.handed_over>>r.goal_terminal>>r.revoked>>r.stopped>>r.arrived>>r.ever_sent>>r.stop_until>>r.deadline>>r.last_now;
 if(!i || format!=1 || s<0 || s>8 || p<0 || p>4 || validate(r.request)!=Validation::Valid)throw std::runtime_error("corrupt record");
 i>>std::ws;if(!i.eof())throw std::runtime_error("trailing record data");r.state=static_cast<State>(s);r.phase=static_cast<Phase>(p);return r;
}
}
