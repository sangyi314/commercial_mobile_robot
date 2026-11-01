// SPDX-License-Identifier: MIT
#include "cmr_mission/execution_port.hpp"
#include <unistd.h>
#include <iostream>
#include <stdexcept>
using namespace cmr::mission;
static void check(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(int argc,char** argv){
 rclcpp::init(argc,argv);
 try{
  auto ns="/cmr_test/port_capacity_"+std::to_string(getpid());
  auto node=std::make_shared<rclcpp::Node>("port_capacity",ns);
  {
   AdapterConfig cfg;cfg.epoch="owner";cfg.action=ns+"/unused_action";
   NavigationAdapter adapter(node,cfg,[](Event){return true;});
   ExecutionPort port(node,{"run","owner",ns,true},adapter,[](Event){return true;});
   Effect e;e.mission="m";e.operation="m.out";e.attempt="m.out.0";e.kind=EffectKind::Send;
   for(int i=0;i<32;++i){e.id=std::to_string(i);port.execute(e);}
   bool rejected=false;try{e.id="overflow";port.execute(e);}catch(const std::runtime_error&){rejected=true;}
   check(rejected&&port.faulted(),"queue overflow must latch failure");
   port.tick();check(port.faulted(),"draining must not clear failure");
   rejected=false;try{e.id="late-send";port.execute(e);}catch(const std::runtime_error&){rejected=true;}
   check(rejected,"late send cannot recover failed port");
   e.kind=EffectKind::Revoke;e.id="emergency";port.execute(e);port.tick();
   port.begin_shutdown();port.stop();adapter.stop();
  }
  node.reset();rclcpp::shutdown();std::cout<<"PASS bounded port overload and fail-closed latch; API unit, no Action Server/Goal\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";rclcpp::shutdown();return 1;}
}
