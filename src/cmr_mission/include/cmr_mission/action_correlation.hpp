// SPDX-License-Identifier: MIT
#pragma once
#include "mission.hpp"
#include <deque>
namespace cmr::mission {
inline Event scope_callback(Event e,const Record& current){
 if(e.kind==Kind::Fatal && (e.operation!=current.operation||e.attempt!=current.attempt)){
  e.detail="STALE_OPERATION_DIAGNOSTIC "+e.detail;e.kind=Kind::Feedback;
 }
 return e;
}
inline bool deadline_elapsed(std::int64_t elapsed_ms,std::int64_t budget_ms){return elapsed_ms>=budget_ms;}
// Pure correlation boundary shared with the real adapter; no ROS or alternate FSM.
class ActionCorrelation {
 std::string epoch,mission,operation,attempt,id;bool accepted=false,finished=false;Kind terminal_kind{Kind::Feedback};
 std::deque<Event> early;
 public:
 bool fault{false};std::size_t discarded{0},conflicts{0};
 ActionCorrelation(std::string owner,const Effect& e):epoch(std::move(owner)),mission(e.mission),operation(e.operation),attempt(e.attempt){}
 std::vector<Event> push(Event e){
  if(e.epoch!=epoch||e.mission!=mission||e.operation!=operation||e.attempt!=attempt){++discarded;return {};}
  if(e.kind==Kind::Accepted){
   if(e.uuid.empty()){fault=true;return {};}
   if(accepted){if(e.uuid!=id)fault=true;else ++discarded;return {};}
   accepted=true;id=e.uuid;std::vector<Event> out{e};while(!early.empty()){auto next=early.front();early.pop_front();auto ready=push(next);out.insert(out.end(),ready.begin(),ready.end());}return out;
  }
  if(!accepted){if(early.size()==2){fault=true;return {};}early.push_back(std::move(e));return {};}
  if(e.uuid!=id){fault=true;return {};}
  const bool terminal=e.kind==Kind::Succeeded||e.kind==Kind::Aborted||e.kind==Kind::CanceledResult;
  if(finished){
   if(terminal&&e.kind!=terminal_kind){++conflicts;e.detail="CONFLICTING_TERMINAL observed="+std::to_string(static_cast<int>(e.kind))+" "+e.detail;e.kind=Kind::Feedback;return {std::move(e)};}
   ++discarded;return {};
  }
  if(terminal){finished=true;terminal_kind=e.kind;}
  return {std::move(e)};
 }
};
}
