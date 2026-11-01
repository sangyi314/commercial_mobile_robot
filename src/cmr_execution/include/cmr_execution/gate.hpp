// SPDX-License-Identifier: MIT
#pragma once
#include <cmr_core/core.hpp>
#include <array>
#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
namespace cmr::execution {
using Ms=std::int64_t;using Ns=std::int64_t;
enum class State{Disarmed,Armed,Stopping,FaultLatched};
inline const char* name(State s){switch(s){case State::Disarmed:return "DISARMED";case State::Armed:return "ARMED";case State::Stopping:return "STOPPING";case State::FaultLatched:return "FAULT_LATCHED";}return "INVALID";}
struct Identity{
 std::string run,gate,owner,mission,operation,attempt;
 auto key()const{return std::tie(run,gate,owner,mission,operation,attempt);}
 bool operator==(const Identity& b)const{return key()==b.key();}bool operator!=(const Identity& b)const{return !(*this==b);}
};
inline bool identifier(const std::string& s){if(s.empty()||s.size()>128)return false;for(unsigned char c:s)if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='.'||c=='_'||c=='-'))return false;return true;}
inline bool valid(const Identity& i){return identifier(i.run)&&identifier(i.gate)&&identifier(i.owner)&&identifier(i.mission)&&identifier(i.operation)&&identifier(i.attempt);}
struct Config{Ms tick{20},lease{300},challenge{100},receive_age{200},clock_stall{300};Ns source_age{300000000};double max_v{.2},max_w{1.0},radius{.033},track{.287},max_wheel{12};std::string frame{"base_footprint"};};
struct Request{std::string verb,id,lease,challenge;Identity who;std::uint64_t seq{0};bool old_resolved{false};
 auto key()const{return std::tie(verb,id,lease,challenge,who.run,who.gate,who.owner,who.mission,who.operation,who.attempt,seq,old_resolved);}
 bool operator==(const Request& b)const{return key()==b.key();}};
struct Reply{std::string code,challenge,lease;Ms deadline{0};std::uint64_t sequence{0};};
struct Candidate{std::array<double,6> u{};Ns stamp{0};std::string frame;};
struct Decision{std::array<double,6> u{};State state;std::string reason;std::uint64_t output_seq,control_seq,generation;};
class Gate{
 Config c;std::string run,epoch,owner;State state_{State::Disarmed};std::string reason_{"START_DISARMED"},clock_epoch_;Identity identity_;std::string lease_;
 Ns ros_{0},boundary_{0};Ms clock_progress_{0},deadline_{0},still_until_{0},last_time_{0};bool clock_seen_{false},route_{false},active_{false};
 std::optional<Candidate> candidate_;Ms candidate_rx_{0};std::uint64_t output_{0},control_{0},generation_{0},challenge_serial_{0},lease_serial_{0},last_seq_{0},mutations_{0};
 struct Challenge{Request q;std::string token;Ms until;};std::optional<Challenge> challenge_;
 std::map<std::string,std::pair<Request,Reply>> begins_;std::optional<std::pair<Request,Reply>> last_renew_,last_revoke_;
 bool time(Ms now){if(now<last_time_){trip("STEADY_ROLLBACK",true);return false;}last_time_=now;return true;}
 bool who(const Identity& i)const{return valid(i)&&i.run==run&&i.gate==epoch&&i.owner==owner;}
 bool clock_ok(Ms now)const{return clock_seen_&&now>=clock_progress_&&now-clock_progress_<c.clock_stall;}
 void clear(){candidate_.reset();challenge_.reset();active_=false;still_until_=0;++control_;++generation_;}
 void expire(Ms now){if(active_&&now>=deadline_)trip("LEASE_EXPIRED");if(active_&&!clock_ok(now))trip("CLOCK_STALLED");}
 Reply fail(const std::string& code)const{return {code,"",lease_,deadline_,control_};}
 public:
 Gate(Config cfg,std::string run_id,std::string gate_epoch,std::string owner_epoch):c(std::move(cfg)),run(std::move(run_id)),epoch(std::move(gate_epoch)),owner(std::move(owner_epoch)),clock_epoch_(epoch+".clock.0"){
  if(!identifier(run)||!identifier(epoch)||!identifier(owner)||c.tick<=0||c.lease<=0||c.challenge<=0||c.receive_age<=0||c.clock_stall<=0||c.source_age<=0||c.lease>60000||c.challenge>c.lease||!std::isfinite(c.max_v)||!std::isfinite(c.max_w)||c.max_v<=0||c.max_w<=0||!cmr::wheel_speed_ok({0,0},c.radius,c.track,c.max_wheel)||c.frame.empty())throw std::invalid_argument("invalid Gate config");
 }
 void trip(const std::string& why,bool latch=false){if(state_==State::FaultLatched)return;clear();state_=latch?State::FaultLatched:State::Stopping;reason_=why;}
 void route(bool good){route_=good;if(!good&&active_)trip("ROUTE_UNQUALIFIED",true);}
 void clock(Ns stamp,Ms now){if(!time(now))return;if(stamp<=0){if(stamp<0||clock_seen_){trip("CLOCK_INVALID");clock_epoch_=epoch+".clock."+std::to_string(generation_);clock_seen_=false;ros_=0;}return;}if(clock_seen_&&stamp<ros_){clear();state_=State::Stopping;reason_="CLOCK_ROLLBACK";clock_epoch_=epoch+".clock."+std::to_string(generation_);}
  if(!clock_seen_||stamp!=ros_){clock_progress_=now;ros_=stamp;clock_seen_=true;}}
 void stationary(const Identity& id,const std::string& clock,std::uint64_t generation,Ms now,Ms freshness){if(id==identity_&&clock==clock_epoch_&&generation==generation_&&freshness>0&&freshness<=200)still_until_=now+freshness;}
 Reply issue(const Request& q,Ms now){if(!time(now))return fail("BAD_TIME");expire(now);if(!who(q.who))return fail("IDENTITY");if(q.verb!="begin"&&q.verb!="renew")return fail("INVALID");
  if(q.verb=="begin"){
   if(state_==State::FaultLatched)return fail("FAULT_LATCHED");
   if(q.who!=identity_){if(active_||state_==State::Stopping)return fail("OLD_MOTION_UNRESOLVED");identity_=q.who;candidate_.reset();still_until_=0;++generation_;}
  }else if(!active_||q.who!=identity_||q.lease!=lease_)return fail("NOT_ARMED");
  if(!identifier(q.id)||q.seq==0)return fail("REQUEST_ID");
  const auto token=epoch+".c"+std::to_string(++challenge_serial_);challenge_=Challenge{q,token,now+c.challenge};return {"CHALLENGE",token,lease_,now+c.challenge,control_};
 }
 Reply control(const Request& q,Ms now){if(!time(now))return fail("BAD_TIME");expire(now);if(!who(q.who))return fail("IDENTITY");
  if(q.verb=="inspect")return fail("STATUS");
  if(q.verb=="revoke"){
   if(!identifier(q.id)||q.seq==0)return fail("REQUEST_ID");
   if(last_revoke_&&last_revoke_->first.id==q.id)return last_revoke_->first==q?last_revoke_->second:fail("REQUEST_CONFLICT");
   if(lease_.empty()&&!active_&&q.lease.empty()&&(!valid(identity_)||q.who==identity_)){identity_=q.who;trip("REVOKED_UNISSUED");}
   else if(q.who!=identity_||q.lease!=lease_||lease_.empty())return fail("IDENTITY");
   if(active_)trip("REVOKED");
   auto r=fail("REVOKED");last_revoke_=std::make_pair(q,r);return r;
  }
  if(q.verb=="reset"){
   if(active_||!q.old_resolved||!route_||!clock_ok(now)||now>=still_until_||q.who!=identity_)return fail("RESET_UNQUALIFIED");
   clear();state_=State::Disarmed;reason_="RESET_DISARMED";return fail("RESET_DISARMED");
  }
  if(q.verb!="begin"&&q.verb!="renew")return fail("INVALID");
  if(q.verb=="begin"){
   auto memo=begins_.find(q.id);if(memo!=begins_.end())return memo->second.first==q?memo->second.second:fail("REQUEST_CONFLICT");
  }else if(last_renew_&&last_renew_->first.id==q.id)return last_renew_->first==q?last_renew_->second:fail("REQUEST_CONFLICT");
  if(!challenge_||q.challenge!=challenge_->token||now>=challenge_->until)return fail("CHALLENGE_EXPIRED");
  auto expected=challenge_->q;expected.challenge=q.challenge;if(!(expected==q))return fail("CHALLENGE_IDENTITY");challenge_.reset();
  if(!route_||!clock_ok(now))return fail("INPUT_UNQUALIFIED");
  if(q.verb=="begin"){
   if(active_||state_!=State::Disarmed||!q.old_resolved||q.who!=identity_||now>=still_until_)return fail("BEGIN_UNQUALIFIED");
   if(begins_.size()>=64){trip("REQUEST_CAPACITY",true);return fail("CAPACITY");}
   clear();identity_=q.who;lease_=epoch+".l"+std::to_string(++lease_serial_);state_=State::Armed;reason_="ARMED";active_=true;last_seq_=q.seq;deadline_=now+c.lease;boundary_=ros_;
   Reply r{"GRANTED","",lease_,deadline_,control_};begins_.emplace(q.id,std::make_pair(q,r));return r;
  }
  if(!active_||state_!=State::Armed||q.who!=identity_||q.lease!=lease_||q.seq<=last_seq_)return fail("STALE_RENEW");
  last_seq_=q.seq;deadline_=now+c.lease;reason_="RENEWED";Reply r{"RENEWED","",lease_,deadline_,control_};last_renew_=std::make_pair(q,r);return r;
 }
 bool candidate(const Candidate& value,Ms received,Ms now){if(!time(now))return false;expire(now);if(!active_)return false;
  bool finite=true;for(double x:value.u)finite=finite&&std::isfinite(x);
  if(!finite||value.u[1]!=0||value.u[2]!=0||value.u[3]!=0||value.u[4]!=0||std::abs(value.u[0])>c.max_v||std::abs(value.u[5])>c.max_w||!cmr::wheel_speed_ok({value.u[0],value.u[5]},c.radius,c.track,c.max_wheel)||value.frame!=c.frame){trip("INVALID_CANDIDATE");return false;}
  if(value.stamp<=0){trip("SOURCE_TIME");return false;}
  if(value.stamp<=boundary_){reason_="PRE_AUTH_CANDIDATE";return false;}
  if(received>now||now-received>=c.receive_age||value.stamp>ros_||ros_-value.stamp>=c.source_age||(candidate_&&value.stamp<candidate_->stamp)){trip("SOURCE_TIME");return false;}
  if(candidate_&&value.stamp==candidate_->stamp){if(value.u!=candidate_->u)++mutations_;candidate_->u=value.u;return true;}
  candidate_=value;candidate_rx_=received;return true;
 }
 Decision tick(Ms now){time(now);expire(now);std::array<double,6> u{};
  if(active_&&candidate_){if(now-candidate_rx_>=c.receive_age)trip("CANDIDATE_EXPIRED");else if(ros_-candidate_->stamp>=c.source_age)trip("SOURCE_EXPIRED");else if(route_&&clock_ok(now)&&state_==State::Armed)u=candidate_->u;}
  return {u,state_,reason_,++output_,control_,generation_};
 }
 State state()const{return state_;}const std::string& reason()const{return reason_;}const Identity& identity()const{return identity_;}const std::string& epoch_id()const{return epoch;}const std::string& lease()const{return lease_;}const std::string& clock_epoch()const{return clock_epoch_;}Ns ros()const{return ros_;}Ms deadline()const{return deadline_;}std::uint64_t generation()const{return generation_;}std::uint64_t mutations()const{return mutations_;}bool active()const{return active_;}
};
}
