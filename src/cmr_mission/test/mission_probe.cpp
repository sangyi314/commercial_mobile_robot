// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_runtime.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
using namespace cmr::mission;
struct Sink:Port{std::string path;EffectKind last{EffectKind::Revoke};explicit Sink(std::string p):path(std::move(p)){}void execute(const Effect& e)override{last=e.kind;int fd=open(path.c_str(),O_WRONLY|O_APPEND|O_CREAT|O_CLOEXEC,0600);if(fd<0)throw std::runtime_error("sink open");std::string s=e.id+" "+std::to_string(static_cast<int>(e.kind))+" "+e.operation+" "+e.attempt+"\n";auto n=write(fd,s.data(),s.size());int rc=fsync(fd);close(fd);if(n!=static_cast<ssize_t>(s.size())||rc)throw std::runtime_error("sink write");}};
int main(int argc,char** argv){try{
 if(argc==2 && std::string(argv[1])=="--help"){std::cout<<"mission_probe DB inspect|normal|cancel|crash WINDOW SINK\nAll events synthetic=true. DB must be new except inspect.\n";return 0;}
 if(argc<3)throw std::runtime_error("see --help");std::string mode=argv[2],window=argc>3?argv[3]:"",sinkpath=argc>4?argv[4]:std::string(argv[1])+".sink";Sink sink(sinkpath);std::string stage;
 auto hook=[&](const std::string& point){bool hit=(window=="submit-before"&&point=="submit-before-commit")||(window=="submit-after"&&point=="submit-after-commit")||(window=="intent-after"&&stage=="prepare"&&point=="event-after-commit")||(window=="send-after"&&stage=="dispatch"&&point=="dispatch-after-port")||(window=="accept-after"&&stage=="accept"&&point=="event-after-commit")||(window=="cancel-after"&&stage=="cancel-dispatch"&&sink.last==EffectKind::CancelExact&&point=="dispatch-after-port")||(window=="terminal-before"&&point=="terminal-before-commit")||(window=="terminal-after"&&point=="terminal-after-commit");if(mode=="crash"&&hit){std::cout<<"BARRIER "<<window<<std::endl;char c;while(read(STDIN_FILENO,&c,1)<0){};throw std::runtime_error("barrier released without kill");}};
 Runtime runtime(argv[1],mode=="inspect"?"reopened":"probe",16,&sink,hook);
 struct Access {Runtime& runtime;
 std::optional<Record> get(std::string id){return runtime.call([id](Store& s){return s.get(id);}).get();}
 Admission submit(MissionRequest q){return runtime.call([q](Store& s){return s.submit(q);}).get();}
 Decision apply(Event e){return runtime.call([e](Store& s){return s.apply(e);}).get();}
 void dispatch(){runtime.call([](Store& s){s.dispatch();}).get();}
 } s{runtime};
 if(mode=="inspect"){auto r=s.get("m");s.dispatch();std::cout<<(r?state_name(r->state):"ABSENT")<<" "<<(r?static_cast<int>(r->phase):0)<<'\n';return 0;}
 auto a=s.submit({1,"m","a","b","map","stations",1});if(a!=Admission::CreateIntent)throw std::runtime_error("demo requires new database");
 if(mode=="normal") {auto q=MissionRequest{1,"m","a","b","map","stations",2};std::cout<<"duplicate="<<static_cast<int>(s.submit(q))<<" ";q.return_station="other";std::cout<<"conflict="<<static_cast<int>(s.submit(q))<<" ";q.mission_id="other";std::cout<<"busy="<<static_cast<int>(s.submit(q))<<'\n';}
 unsigned count=0;auto apply=[&](Kind k){auto r=*s.get("m");Event e;e.id="event-"+std::to_string(++count);e.mission="m";e.epoch=r.epoch;e.operation=r.operation;e.attempt=r.attempt;e.uuid=r.uuid;e.kind=k;e.now=r.last_now+2;e.expected_version=r.version;e.observed_from=e.now-1;e.observed_to=e.now;e.valid_until=e.now+10000;if(k==Kind::Accepted)e.uuid="u-"+r.attempt;auto d=s.apply(e);if(!d.changed&&k!=Kind::CancelAck)throw std::runtime_error("demo transition: "+d.reason);std::cout<<"synthetic=true event="<<static_cast<int>(k)<<" state="<<state_name(d.next.state)<<" held="<<!terminal(d.next.state)<<'\n';};
 stage="prepare";apply(Kind::Prepare);stage="dispatch";s.dispatch();stage="accept";apply(Kind::Accepted);
 if(mode=="cancel"||window=="cancel-after"){stage="cancel";apply(Kind::Cancel);apply(Kind::CancelAck);stage="cancel-dispatch";s.dispatch();apply(Kind::Succeeded);apply(Kind::RevokeAck);apply(Kind::Stop);apply(Kind::Resolve);}
 else {apply(Kind::Succeeded);apply(Kind::RevokeAck);apply(Kind::Stop);apply(Kind::Arrival);apply(Kind::Resolve);apply(Kind::Handover);s.dispatch();apply(Kind::Accepted);apply(Kind::Succeeded);apply(Kind::RevokeAck);apply(Kind::Stop);apply(Kind::Arrival);stage="terminal";apply(Kind::Resolve);}
 std::cout<<"FINAL "<<state_name(s.get("m")->state)<<'\n';return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
