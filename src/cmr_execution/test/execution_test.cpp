// SPDX-License-Identifier: MIT
#include "cmr_execution/gate.hpp"
#include "cmr_execution/stop.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <random>
using namespace cmr::execution;
void ck(bool b,const char* why){if(!b)throw std::runtime_error(why);}
Identity id(){return {"run","gate","owner","m","m.out","m.out.0"};}
struct Fixture{
 Gate g{Config{},"run","gate","owner"};Ms now{1};Ns ros{1000000000};std::string lease;Request begin;
 Fixture(){g.route(true);g.clock(ros,now);}
 Reply arm(){begin={"begin","begin1","","",id(),1,true};auto challenge=g.issue(begin,now);ck(challenge.code=="CHALLENGE","challenge");g.stationary(id(),g.clock_epoch(),g.generation(),now,200);begin.challenge=challenge.challenge;auto r=g.control(begin,now);lease=r.lease;ck(r.code=="GRANTED","grant");return r;}
 void advance(Ms dt){now+=dt;ros+=dt*1000000;g.clock(ros,now);}
 bool input(double v=.1,double w=0){Candidate c;c.u={v,0,0,0,0,w};c.stamp=ros;c.frame="base_footprint";return g.candidate(c,now,now);}
 Request renew(std::uint64_t seq=2){Request q{"renew","renew"+std::to_string(seq),lease,"",id(),seq,false};auto r=g.issue(q,now);q.challenge=r.challenge;return q;}
 Request revoke(){return {"revoke","revoke1",lease,"",id(),100,false};}
};
Binding binding(){return {id(),"clock","lease",2};}
StopResult window(int mode){StopWindow w;auto b=binding();for(int i=0;i<=30;++i){Sample s;s.binding=b;s.stamp=1000000000+std::int64_t(i)*100000000;s.received=i*100;s.output_fresh=true;
 if(mode==1&&i==10)s.stamp-=100000000;
 if(mode==2&&i==10)s.binding.who.operation="old";
 if(mode==3)s.x=i*.002;
 if(mode==4&&i==25)s.output_v=.1;
 if(mode==5&&i==25)s.v=.02;
 if(mode==6&&i>10)s.stamp+=1000000000;
 if(mode==7)s.yaw=i*.004;
 if(mode==9&&i>10)s.received+=1000;
 w.add(s);}
 return w.evaluate(mode==8?3300:mode==9?4000:3000,4000000000,b);}
int main(int argc,char** argv){try{ck(argc==2,"case required");std::string c=argv[1];
 if(c=="identity"){
  Fixture f;ck(f.g.tick(1).u[0]==0,"default zero");auto q=Request{"begin","b","","",id(),1,true};q.who.owner="wrong";ck(f.g.issue(q,1).code=="IDENTITY","wrong owner");q.who=id();q.who.gate="old";ck(f.g.issue(q,1).code=="IDENTITY","old gate");q.who=id();q.who.run="other";ck(f.g.issue(q,1).code=="IDENTITY","wrong run");
 }else if(c=="normal"){
  Fixture f;auto r=f.arm();ck(r.deadline==301,"manual deadline");ck(f.g.tick(1).u[0]==0,"Begin alone no motion");f.advance(1);ck(f.input(.12,.15),"candidate");auto d=f.g.tick(f.now);ck(d.u[0]==.12&&d.u[5]==.15,"exact original values");
 }else if(c=="idempotence"){
  Fixture f;auto r=f.arm();f.advance(10);ck(f.g.control(f.begin,f.now).deadline==r.deadline,"duplicate Begin cannot extend");auto altered=f.begin;altered.old_resolved=false;ck(f.g.control(altered,f.now).code=="REQUEST_CONFLICT","same ID different content");auto q=f.renew();auto renew=f.g.control(q,f.now);ck(renew.code=="RENEWED","renew");f.advance(10);ck(f.g.control(q,f.now).deadline==renew.deadline,"duplicate renew no extension");auto revoke=f.revoke();f.g.control(revoke,f.now);f.g.control(q,f.now);ck(!f.g.active(),"old renew cannot revive");ck(f.g.control(revoke,f.now).code=="REVOKED","duplicate Revoke");auto conflict=revoke;conflict.seq++;ck(f.g.control(conflict,f.now).code=="REQUEST_CONFLICT","Revoke ID bound");
 }else if(c=="freshness"){
  Fixture f;f.arm();f.advance(1);ck(f.input(),"first sample");for(int i=0;i<3;++i){f.advance(50);Candidate cnd;cnd.frame="base_footprint";cnd.stamp=1001000000;cnd.u[0]=.1;ck(f.g.candidate(cnd,f.now,f.now),"repeated stamp");auto q=f.renew(i+2);f.g.control(q,f.now);}
  f.advance(50);ck(f.g.tick(f.now).u[0]==0&&!f.g.active(),"repeated stamp does not refresh age");
  for(int mode=0;mode<5;++mode){Fixture x;x.arm();x.advance(1);Candidate bad;bad.frame="base_footprint";bad.stamp=x.ros;bad.u[0]=.1;if(mode==0)bad.u[0]=NAN;if(mode==1)bad.u[1]=.01;if(mode==2)bad.stamp=x.ros+1;if(mode==3)bad.stamp=0;if(mode==4)bad.u[0]=.21;ck(!x.g.candidate(bad,x.now,x.now),"reject unsafe input");ck(x.g.tick(x.now).u[0]==0,"unsafe zero");}
 }else if(c=="clock"){
  Fixture f;f.arm();f.advance(1);f.input();f.g.clock(f.ros,302);ck(f.g.tick(302).u[0]==0,"paused clock wall timeout");f.g.clock(2000000000,303);ck(!f.g.active(),"clock resume no rearm");Config independent;independent.lease=1000;independent.receive_age=1000;Gate isolated(independent,"run","gate","owner");isolated.route(true);isolated.clock(1000000000,1);Request b{"begin","independent","","",id(),1,true};auto challenge=isolated.issue(b,1);isolated.stationary(id(),isolated.clock_epoch(),isolated.generation(),1,200);b.challenge=challenge.challenge;ck(isolated.control(b,1).code=="GRANTED","isolated clock grant");isolated.clock(1001000000,2);Candidate candidate;candidate.frame="base_footprint";candidate.stamp=1001000000;candidate.u[0]=.1;isolated.candidate(candidate,2,2);auto halted=isolated.tick(302);ck(halted.u[0]==0&&halted.reason=="CLOCK_STALLED","clock timeout independent of lease/candidate deadlines");
  Fixture x;x.arm();auto epoch=x.g.clock_epoch();x.g.clock(900000000,2);ck(x.g.clock_epoch()!=epoch&&!x.g.active(),"rollback invalidates epoch");
 }else if(c=="restart"){
  Fixture f;f.arm();Gate restarted(Config{},"run","newgate","owner");restarted.route(true);restarted.clock(f.ros,1);ck(restarted.control(f.begin,1).code=="IDENTITY","old Begin rejected on new Gate");ck(restarted.tick(1).u[0]==0,"restart default zero");
 }else if(c=="boundaries"){
  Fixture f;f.arm();f.advance(299);ck(f.g.active(),"before lease deadline");auto q=f.renew();auto late=f.g.control(q,301);ck(late.code!="RENEWED"&&!f.g.active(),"equality expires lease");Fixture x;Request b{"begin","b","","",id(),1,true};auto ch=x.g.issue(b,1);x.g.stationary(id(),x.g.clock_epoch(),x.g.generation(),1,200);b.challenge=ch.challenge;ck(x.g.control(b,101).code=="CHALLENGE_EXPIRED","challenge equality");Fixture y;y.arm();y.advance(1);y.input();y.advance(199);ck(y.g.tick(y.now).u[0]==.1,"before candidate boundary");y.advance(1);ck(y.g.tick(y.now).u[0]==0,"candidate equality");
 }else if(c=="concurrency"){
  for(int run=0;run<50;++run){Fixture f;f.arm();f.advance(1);f.input();std::mutex m;bool revoked=false;std::thread tick([&]{for(int i=0;i<200;++i){std::lock_guard<std::mutex> l(m);auto d=f.g.tick(2);ck(!revoked||d.u[0]==0,"linearized revoke forbids later nonzero");}});{std::lock_guard<std::mutex> l(m);f.g.control(f.revoke(),2);revoked=true;}tick.join();ck(f.g.tick(2).u[0]==0,"revoke final");}
 }else if(c=="stop_good"){
  auto r=window(0);ck(r.verdict=="STOPPED"&&r.samples==31&&r.from==1000000000&&r.to==4000000000,"hand-calculated 3s stationary window");
 }else if(c=="stop_bad"){
  for(int mode=1;mode<=9;++mode){ck(window(mode).verdict!="STOPPED","reject invalid stopping sequence");}
  StopWindow w;Sample s;s.binding=binding();s.stamp=1;s.output_fresh=true;w.add(s);ck(w.evaluate(0,1,binding()).verdict=="STOP_UNKNOWN","short window unknown");
 }else if(c=="reset"){
  Fixture f;f.arm();f.advance(1);f.input();f.g.control(f.revoke(),2);Request reset{"reset","reset1",f.lease,"",id(),101,true};ck(f.g.control(reset,2).code=="RESET_UNQUALIFIED","no stop proof no reset");f.g.stationary(id(),f.g.clock_epoch(),f.g.generation(),2,200);ck(f.g.control(reset,2).code=="RESET_DISARMED","qualified reset only disarms");ck(f.g.tick(2).u[0]==0,"no cached candidate after Reset");bool rejected=false;try{Config invalid;invalid.lease=0;Gate g(invalid,"run","gate","owner");}catch(const std::invalid_argument&){rejected=true;}ck(rejected,"invalid config");
 }else if(c=="time_format"){
  for(Ns stamp:{Ns(-1),Ns(0),std::numeric_limits<Ns>::min()}){Fixture f;f.arm();f.advance(1);f.input();Candidate bad;bad.u={.1,0,0,0,0,0};bad.frame="base_footprint";bad.stamp=stamp;ck(!f.g.candidate(bad,f.now,f.now)&&!f.g.active()&&f.g.tick(f.now).u[0]==0,"invalid source stamp fail closed");}
  for(Ns stamp:{Ns(-1),Ns(0)}){Fixture f;f.arm();f.advance(1);f.input();auto old=f.g.clock_epoch();f.g.clock(stamp,f.now);ck(!f.g.active()&&f.g.clock_epoch()!=old&&f.g.tick(f.now).u[0]==0,"invalid clock invalidates epoch");f.advance(1);ck(!f.g.active(),"clock recovery cannot authorize");}
  Gate initial(Config{},"run","gate","owner");initial.clock(0,0);ck(initial.state()==State::Disarmed,"initial zero stays disarmed");
 }else if(c=="random"){
  for(int seed=0;seed<20;++seed){Fixture f;f.arm();std::mt19937 rng(seed);for(int i=0;i<200;++i){f.advance(1);if(rng()%4==0)f.g.control(f.revoke(),f.now);else if(f.g.active())f.input();auto d=f.g.tick(f.now);ck(f.g.active()||d.u[0]==0,"inactive never nonzero");}ck(f.g.tick(1000).u[0]==0,"eventual expiry");}
 }else throw std::runtime_error("unknown case");
 std::cout<<"PASS "<<c<<" (explicit checks retained in Release)\n";return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
