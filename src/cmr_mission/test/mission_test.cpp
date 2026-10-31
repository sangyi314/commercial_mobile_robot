// SPDX-License-Identifier: MIT
#include "cmr_mission/mission.hpp"
#include <iostream>
#include <stdexcept>
using namespace cmr::mission;
void ck(bool b,const char* s){if(!b)throw std::runtime_error(s);}
Record initial(){Record r;r.request={1,"m","a","b","map","stations",1};r.epoch="e";return r;}
Event event(const Record& r,Kind k){Event e;e.id=std::to_string(r.version)+"e";e.mission=r.request.mission_id;e.epoch=r.epoch;e.operation=r.operation;e.attempt=r.attempt;e.uuid=r.uuid;e.kind=k;e.now=r.last_now+2;e.expected_version=r.version;e.observed_from=e.now-1;e.observed_to=e.now;e.valid_until=e.now+10000;return e;}
void apply(Record& r,Kind k){auto e=event(r,k);if(k==Kind::Accepted)e.uuid="uuid-"+r.attempt;auto d=reduce(r,e);ck(d.changed,"expected transition");r=d.next;}
void leg(Record& r){apply(r,Kind::Accepted);apply(r,Kind::Succeeded);apply(r,Kind::RevokeAck);apply(r,Kind::Stop);apply(r,Kind::Arrival);apply(r,Kind::Resolve);}
int main(){try{
 auto r=initial();apply(r,Kind::Prepare);ck(r.phase==Phase::SendPending,"send reserves slot");leg(r);ck(r.state==State::WaitHandover,"wait handover");apply(r,Kind::Handover);leg(r);ck(r.state==State::Completed && r.handed_over,"complete");ck(!reduce(r,event(r,Kind::Cancel)).changed,"terminal stable");
 r=initial();ck(!reduce(r,event(r,Kind::Handover)).changed,"illegal");apply(r,Kind::Prepare);auto old=event(r,Kind::Accepted);old.uuid="u";old.epoch="old";ck(!reduce(r,old).changed,"old epoch");old=event(r,Kind::Accepted);old.uuid="u";old.attempt="wrong";ck(!reduce(r,old).changed,"old attempt");
 apply(r,Kind::Cancel);ck(!reduce(r,event(r,Kind::CancelAck)).changed,"ack only");apply(r,Kind::Accepted);ck(r.state==State::Paused,"late accept stays paused");ck(!reduce(r,event(r,Kind::Resolve)).changed,"goal unknown");apply(r,Kind::Succeeded);apply(r,Kind::RevokeAck);ck(!reduce(r,event(r,Kind::Resolve)).changed,"stop missing");auto bad=event(r,Kind::Stop);bad.gate_epoch="wrong";ck(!reduce(r,bad).changed,"stop epoch");apply(r,Kind::Stop);apply(r,Kind::Resolve);ck(r.state==State::Canceled,"cancel before success");
 r=initial();apply(r,Kind::Prepare);apply(r,Kind::Accepted);apply(r,Kind::Succeeded);apply(r,Kind::Cancel);apply(r,Kind::RevokeAck);apply(r,Kind::Stop);apply(r,Kind::Resolve);ck(r.state==State::Canceled,"success before cancel but not completed");
 r=initial();auto t=event(r,Kind::Tick);t.now=r.deadline-1;ck(!reduce(r,t).changed,"before deadline");t.now=r.deadline;ck(reduce(r,t).changed,"equal deadline expires");t.now++;ck(reduce(r,t).changed,"after deadline");
 r=initial();apply(r,Kind::Prepare);apply(r,Kind::Accepted);apply(r,Kind::Aborted);apply(r,Kind::RevokeAck);apply(r,Kind::Stop);apply(r,Kind::Recover);apply(r,Kind::Retry);apply(r,Kind::Accepted);apply(r,Kind::Aborted);apply(r,Kind::RevokeAck);apply(r,Kind::Stop);ck(!reduce(r,event(r,Kind::Recover)).changed,"retry exhausted");
 r=initial();apply(r,Kind::Prepare);apply(r,Kind::Accepted);auto wrong=event(r,Kind::Succeeded);wrong.uuid="old-uuid";ck(!reduce(r,wrong).changed,"old UUID");apply(r,Kind::Succeeded);apply(r,Kind::RevokeAck);auto expired=event(r,Kind::Stop);expired.valid_until=expired.now;ck(!reduce(r,expired).changed,"expired stop");expired=event(r,Kind::Stop);expired.observed_from=expired.observed_to;ck(!reduce(r,expired).changed,"nonadvancing stop");apply(r,Kind::Stop);apply(r,Kind::Arrival);apply(r,Kind::Resolve);wrong=event(r,Kind::Handover);wrong.expected_version++;ck(!reduce(r,wrong).changed,"handover state version");apply(r,Kind::Handover);ck(!reduce(r,event(r,Kind::Handover)).changed,"duplicate handover");apply(r,Kind::Accepted);apply(r,Kind::Aborted);apply(r,Kind::RevokeAck);apply(r,Kind::Stop);apply(r,Kind::Resolve);ck(r.state==State::Failed&&r.handed_over,"delivered but return failed");
 r=initial();apply(r,Kind::Prepare);apply(r,Kind::Accepted);apply(r,Kind::Stop);ck(!reduce(r,event(r,Kind::Resolve)).changed,"stop before result does not release");
 r=initial();apply(r,Kind::Prepare);leg(r);t=event(r,Kind::Tick);t.now=r.deadline;auto timed=reduce(r,t);ck(timed.changed&&timed.next.state==State::Paused&&timed.next.pending=="FAILED","handover timeout");r=timed.next;ck(!reduce(r,event(r,Kind::Handover)).changed,"late confirmation");
 r=initial();apply(r,Kind::Cancel);apply(r,Kind::Resolve);ck(r.state==State::Canceled,"no-motion cancel");
 ck(encode(decode(encode(r)))==encode(r),"serialization");std::cout<<"PASS state timelines and counterexamples\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
