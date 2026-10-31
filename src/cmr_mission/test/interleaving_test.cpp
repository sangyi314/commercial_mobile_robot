// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_runtime.hpp"
#include <sqlite3.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <algorithm>
using namespace cmr::mission;
void ck(bool b,const char* s){if(!b)throw std::runtime_error(s);}
MissionRequest request(std::string id="m"){return {1,id,"a","b","map","stations",1};}
struct Sink:Port{std::set<std::string> ids,active_attempts;int sends=0;std::ofstream out;explicit Sink(const std::string& path):out(path){}void execute(const Effect& e)override{ck(ids.insert(e.id).second,"duplicate effect execution");if(e.kind==EffectKind::Send){++sends;active_attempts.insert(e.attempt);ck(active_attempts.size()<=1,"I2 external receiver has multiple active attempts");}out<<e.id<<' '<<static_cast<int>(e.kind)<<' '<<e.operation<<' '<<e.attempt<<std::endl;}};
int scalar(const std::string& path,const char* q){sqlite3* db=nullptr;ck(sqlite3_open_v2(path.c_str(),&db,SQLITE_OPEN_READONLY,nullptr)==0,"oracle open");sqlite3_stmt* s=nullptr;ck(sqlite3_prepare_v2(db,q,-1,&s,nullptr)==0,"oracle prepare");ck(sqlite3_step(s)==SQLITE_ROW,"oracle step");int n=sqlite3_column_int(s,0);sqlite3_finalize(s);sqlite3_close(db);return n;}
void concurrent(const std::string& root){
 for(int scenario=0;scenario<3;++scenario){Runtime rt(root+"/concurrent"+std::to_string(scenario)+".db","owner");std::mutex m;std::condition_variable cv;int ready=0;bool go=false;Admission a,b;
 auto run=[&](bool second,Admission& result){auto q=request(second&&scenario==0?"other":"m");if(second&&scenario==2)q.return_station="x";{std::unique_lock<std::mutex> l(m);++ready;cv.notify_all();cv.wait(l,[&]{return go;});}result=rt.call([q](Store& s){return s.submit(q);}).get();};
 std::thread x(run,false,std::ref(a)),y(run,true,std::ref(b));{std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{return ready==2;});go=true;cv.notify_all();}x.join();y.join();
 const auto other=scenario==0?Admission::Busy:scenario==1?Admission::Existing:Admission::Conflict;ck((a==Admission::CreateIntent&&b==other)||(b==Admission::CreateIntent&&a==other),"concurrent admit");rt.close();}
}
void random_run(const std::string& root,unsigned seed,const std::string& replay=""){
 const auto path=root+"/seed-"+std::to_string(seed)+".db";Sink sink(root+"/seed-"+std::to_string(seed)+".receiver");std::ofstream log(root+"/seed-"+std::to_string(seed)+".events");std::ofstream trace(root+"/seed-"+std::to_string(seed)+".trace");std::mt19937 rng(seed);unsigned epoch=0;auto store=std::make_unique<Store>(path,"e0",&sink);std::ifstream in(replay);int old_sends=0;
 for(int step=0;step<200;++step){unsigned op=static_cast<unsigned>(rng()%24);if(!replay.empty())ck(static_cast<bool>(in>>op),"replay missing event");log<<op<<std::endl;
 if(op==0 || op==1 || op==2 || op==3){auto q=request(op==3?"other":"m");if(op==1)q.return_station="conflict";store->submit(q);}
 else if(op==23){store.reset();store=std::make_unique<Store>(path,"e"+std::to_string(++epoch),&sink);ck(sink.sends==old_sends,"restart emitted send");}
 else if(auto r=store->get("m")){
  Event e;e.id="random-"+std::to_string(step);e.mission="m";e.epoch=r->epoch;e.operation=r->operation;e.attempt=r->attempt;e.uuid=r->uuid;e.expected_version=r->version;e.now=r->last_now+2;e.observed_from=e.now-1;e.observed_to=e.now;e.valid_until=e.now+10000;
  static const Kind kinds[]={Kind::Prepare,Kind::Accepted,Kind::Succeeded,Kind::Aborted,Kind::Rejected,Kind::Cancel,Kind::CancelAck,Kind::RevokeAck,Kind::Stop,Kind::Arrival,Kind::Handover,Kind::Tick,Kind::Fatal,Kind::Recover,Kind::Retry,Kind::Resolve,Kind::Feedback,Kind::Stop,Kind::Accepted};
  e.kind=kinds[(op-4)%19];if(e.kind==Kind::Accepted)e.uuid="uuid-"+r->attempt;
  if(op==21)e.clock_epoch="old-clock";if(op==22)e.epoch="old-owner";if(e.kind==Kind::Tick)e.now=r->deadline;
  const auto before=*r;store->apply(e);const auto after=*store->get("m");
  if(!before.goal_terminal && after.goal_terminal)sink.active_attempts.erase(before.attempt);
  if(op==22)ck(encode(before)==encode(after),"I8 old owner changed current mission");
  ck(after.retries<=1 && after.deadline==before.deadline,"I9 retry budget/deadline reset");if(terminal(before.state))ck(encode(before)==encode(after),"terminal resurrection");
 }
 store->dispatch();old_sends=sink.sends;if(auto record=store->get("m"))trace<<step<<" op="<<op<<" "<<encode(*record)<<std::endl;
 if(auto r=store->get("m")){
  const int held=scalar(path,"SELECT count(*) FROM manager WHERE active='m'");
  if(!r->goal_terminal || !r->revoked || !r->stopped)ck(held==1,"I5 released unresolved motion");
  if(r->state==State::Completed)ck(r->handed_over&&r->arrived&&r->goal_terminal&&r->revoked&&r->stopped,"I10 completion conflated facts");
  if(r->state==State::WaitHandover)ck(r->arrived&&r->goal_terminal&&r->revoked&&r->stopped,"I10 handover without arrival stop");
 }
 ck(scalar(path,"SELECT count(*) FROM missions WHERE terminal=0")<=1,"two active missions");
 ck(scalar(path,"SELECT count(*) FROM manager WHERE active IS NOT NULL")<=1,"two slots");
 ck(scalar(path,"SELECT count(*) FROM operations WHERE kind=0 AND status IN('INTENT','DISPATCHING')")<=1,"multiple pending sends");
 ck(scalar(path,"SELECT count(*) FROM (SELECT id,count(*) n FROM operations GROUP BY id HAVING n>1)")==0,"duplicate intent");
 }
}
int main(int argc,char** argv){try{if(argc<2){std::cout<<"interleaving_test ARTIFACT_DIR [SEED [EVENT_FILE]]\n";return 0;}std::filesystem::create_directories(argv[1]);if(argc>2){random_run(argv[1],static_cast<unsigned>(std::stoul(argv[2])),argc>3?argv[3]:"");}else {concurrent(argv[1]);for(unsigned seed=0;seed<20;++seed)random_run(argv[1],seed);}std::cout<<"PASS deterministic concurrent admission and fixed interleavings\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
