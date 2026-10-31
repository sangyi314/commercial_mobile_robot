// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_runtime.hpp"
#include <sqlite3.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <sys/stat.h>
using namespace cmr::mission;
void ck(bool b,const char* m){if(!b)throw std::runtime_error(m);}
MissionRequest req(){return {1,"m","a","b","map","stations",1};}
struct Sink:Port{int sends=0;void execute(const Effect& e)override{if(e.kind==EffectKind::Send)++sends;}};
void sql(const std::string& path,const char* q){sqlite3* d=nullptr;ck(sqlite3_open(path.c_str(),&d)==0,"open");char* err=nullptr;int rc=sqlite3_exec(d,q,nullptr,nullptr,&err);sqlite3_free(err);sqlite3_close(d);ck(rc==0,"test SQL");}
int main(int argc,char** argv){try{
 if(argc!=2)throw std::runtime_error("need artifact dir");std::filesystem::create_directories(argv[1]);std::string path=std::string(argv[1])+"/store.db";Sink sink;
 {Store s(path,"e",&sink);ck(s.submit(req())==Admission::CreateIntent,"accept");ck(s.submit(req())==Admission::Existing,"duplicate");auto q=req();q.return_station="x";ck(s.submit(q)==Admission::Conflict,"conflict");q=req();q.mission_id="new";ck(s.submit(q)==Admission::Busy,"busy");
 bool locked=false;try{Store other(path,"other");}catch(...){locked=true;}ck(locked,"second owner");
 Event e;e.id="prepare";e.kind=Kind::Prepare;e.mission="m";e.epoch="e";e.now=1;ck(s.apply(e).changed,"prepare");s.dispatch();s.dispatch();ck(sink.sends==1,"dispatch once");ck(!s.apply(e).changed,"duplicate event");}
 {Store s(path,"restart",&sink);auto m=*s.get("m");ck(m.state==State::Paused && m.phase==Phase::Uncertain,"paused restart");s.dispatch();ck(sink.sends==1,"no replay");}
 auto stale=std::string(argv[1])+"/stale.db";
 {Sink receiver;Store s(stale,"e",&receiver);s.submit(req());Event e;e.id="unknown";e.mission="absent";ck(s.apply(e).reason=="UNKNOWN_MISSION","unknown input not storage fault");e.id="p";e.mission="m";e.epoch="e";e.kind=Kind::Prepare;e.now=1;s.apply(e);auto m=*s.get("m");e.id="a";e.operation=m.operation;e.attempt=m.attempt;e.uuid="u";e.kind=Kind::Accepted;e.now=2;s.apply(e);s.dispatch();ck(receiver.sends==0,"accepted goal suppresses stale send intent");}
 auto fail=std::string(argv[1])+"/fail.db";
 {Store s(fail,"e");sql(fail,"CREATE TRIGGER fail BEFORE INSERT ON events BEGIN SELECT RAISE(ABORT,'controlled failure'); END;");bool thrown=false;try{s.submit(req());}catch(...){thrown=true;}ck(thrown,"write abort");}
 {Store s(fail,"restart");ck(!s.get("m"),"rollback task and slot");}
 auto busy=std::string(argv[1])+"/busy.db";
 {Store s(busy,"e");sqlite3* db=nullptr;ck(sqlite3_open(busy.c_str(),&db)==0,"busy connection");ck(sqlite3_exec(db,"BEGIN IMMEDIATE",nullptr,nullptr,nullptr)==0,"lock writer");bool failed=false;try{s.submit(req());}catch(...){failed=true;}sqlite3_exec(db,"ROLLBACK",nullptr,nullptr,nullptr);sqlite3_close(db);ck(failed,"busy bounded failure");}
 {Store s(busy,"afterbusy");ck(!s.get("m"),"busy did not accept");}
 auto ro=std::string(argv[1])+"/readonly.db";{Store s(ro,"e");}chmod(ro.c_str(),0444);bool readonly=false;try{Store s(ro,"e2");s.submit(req());}catch(...){readonly=true;}chmod(ro.c_str(),0600);ck(readonly,"readonly actual SQLite failure");
 auto corrupt=std::string(argv[1])+"/corrupt.db";{std::ofstream f(corrupt);f<<"corrupt database bytes";}bool bad=false;try{Store s(corrupt,"e");}catch(...){bad=true;}ck(bad,"corrupt rejected");
 auto unknown=std::string(argv[1])+"/unknown.db";sql(unknown,"PRAGMA user_version=99;");bad=false;try{Store s(unknown,"e");}catch(...){bad=true;}ck(bad,"schema rejected");
 auto alias=std::string(argv[1])+"/alias.db";std::filesystem::create_symlink(path,alias);bad=false;try{Store s(alias,"e");}catch(...){bad=true;}ck(bad,"symlink rejected");
 auto runtime=std::string(argv[1])+"/runtime.db";
 {Runtime rt(runtime,"e",1);std::promise<void> entered,release;auto gate=release.get_future().share();auto first=rt.call([&](Store&){entered.set_value();gate.wait();return 1;});entered.get_future().wait();auto second=rt.call([](Store&){return 2;});bad=false;try{rt.call([](Store&){return 3;});}catch(...){bad=true;}release.set_value();ck(first.get()==1&&second.get()==2&&bad,"mailbox bounded drains");rt.close();bad=false;try{rt.call([](Store&){return 0;});}catch(...){bad=true;}ck(bad,"closed rejects");}
 std::cout<<"PASS SQLite transactions recovery ownership mailbox\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
