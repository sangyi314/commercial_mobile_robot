// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_runtime.hpp"
#include <filesystem>
#include <iostream>
using namespace cmr::mission;
void ck(bool b,const char* s){if(!b)throw std::runtime_error(s);}
MissionRequest req(){return {1,"m","a","b","map","stations",1};}
Event ev(Record r,Kind k,std::string id){Event e;e.id=id;e.kind=k;e.mission="m";e.epoch=r.epoch;e.operation=r.operation;e.attempt=r.attempt;e.uuid=r.uuid;e.now=r.last_now+2;e.observed_from=e.now-1;e.observed_to=e.now;e.valid_until=e.now+1000;if(k==Kind::Accepted)e.uuid="uuid";return e;}
int main(int argc,char** argv){try{ck(argc==2,"artifact path required");std::filesystem::create_directories(argv[1]);
 for(int order=0;order<2;++order){Runtime rt(std::string(argv[1])+"/order"+std::to_string(order)+".db","e");rt.call([](Store& s){s.submit(req());}).get();
 auto apply=[&](Kind k,std::string id){return rt.call([=](Store& s){return s.apply(ev(*s.get("m"),k,id));}).get();};
 apply(Kind::Prepare,"p");apply(Kind::Accepted,"a");
 auto snapshot=*rt.call([](Store& s){return s.get("m");}).get();
 std::mutex mutex;std::condition_variable cv;int ready=0;bool go=false,first_enqueued=false;std::future<Decision> a,b;
 auto producer=[&](bool first){ {std::unique_lock<std::mutex> l(mutex);++ready;cv.notify_all();cv.wait(l,[&]{return go && (first||first_enqueued);});}
 auto k=(first==(order==0))?Kind::Cancel:Kind::Succeeded;
 auto result=rt.call([e=ev(snapshot,k,first?"first":"second")](Store& s){return s.apply(e);});
 if(first){a=std::move(result);std::lock_guard<std::mutex> l(mutex);first_enqueued=true;cv.notify_all();}else b=std::move(result);
 };
 std::thread x(producer,true),y(producer,false);{std::unique_lock<std::mutex> l(mutex);cv.wait(l,[&]{return ready==2;});go=true;cv.notify_all();}x.join();y.join();ck(a.get().changed&&b.get().changed,"both ordered events applied");
 auto q=req();q.mission_id="new";ck(rt.call([q](Store& s){return s.submit(q);}).get()==Admission::Busy,"pending stop holds slot");
 apply(Kind::RevokeAck,"revoked");apply(Kind::Stop,"stop");apply(Kind::Resolve,"resolve");ck(rt.call([](Store& s){return s.get("m")->state;}).get()==State::Canceled,"cancel wins before final completion");
 std::cout<<"PASS runtime ordered cancel/success "<<order<<'\n';}
 for(int round=0;round<20;++round){const auto path=std::string(argv[1])+"/close"+std::to_string(round)+".db";Runtime rt(path,"e");std::mutex m;std::condition_variable cv;int ready=0;bool go=false;bool accepted=false,rejected=false;
 auto wait=[&]{std::unique_lock<std::mutex> l(m);++ready;cv.notify_all();cv.wait(l,[&]{return go;});};
 std::thread submit([&]{wait();try{accepted=rt.call([](Store& s){return s.submit(req());}).get()==Admission::CreateIntent;}catch(const std::runtime_error& e){rejected=std::string(e.what())=="CLOSED";}});
 std::thread close([&]{wait();rt.close();});{std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{return ready==2;});go=true;cv.notify_all();}submit.join();close.join();ck(accepted!=rejected,"close race resolves future");Store inspect(path,"verify");ck(inspect.get("m").has_value()==accepted,"ack is committed; rejected no mission");}
 std::cout<<"PASS close/submit 20 barrier races\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
