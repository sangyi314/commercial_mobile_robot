// SPDX-License-Identifier: MIT
#include "cmr_mission/execution_port.hpp"
#include <unistd.h>
#include <cmr_interfaces/srv/execution_lease.hpp>
#include <thread>
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
  {
   using Lease=cmr_interfaces::srv::ExecutionLease;int inspected=0,revoked=0,begun=0,contexts=0;
   auto service=node->create_service<Lease>(ns+"/lease",[&](Lease::Request::SharedPtr q,Lease::Response::SharedPtr r){r->gate_epoch="unit-gate";r->clock_epoch="unit-clock";r->ros_ns=1000000000;
    if(q->operation=="inspect"){++inspected;r->code="STATUS";r->state="DISARMED";}else if(q->operation=="revoke"){++revoked;r->code="REVOKED";r->state="STOPPING";}else{++begun;r->code="DENIED";r->state="DISARMED";}});
   AdapterConfig cfg;cfg.epoch="owner";cfg.action=ns+"/unused_action";NavigationAdapter adapter(node,cfg,[](Event){return true;});ExecutionPort* handle=nullptr;
   ExecutionPort port(node,{"run","owner",ns,true},adapter,[&](Event e){if(e.kind==Kind::ExecutionContext){++contexts;Effect r{"after-context","m","m.out","m.out.0","",EffectKind::Revoke};handle->execute(r);}return true;});handle=&port;
   port.execute({"reserve","m","m.out","m.out.0","",EffectKind::Send});port.execute({"cancel-before-inspect","m","m.out","m.out.0","",EffectKind::Revoke});
   rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);auto until=std::chrono::steady_clock::now()+std::chrono::seconds(4);
   while(std::chrono::steady_clock::now()<until&&revoked==0){port.tick();executor.spin_some(std::chrono::milliseconds(5));std::this_thread::sleep_for(std::chrono::milliseconds(1));}
   check(inspected==1&&contexts==1&&revoked==1&&begun==0,"early cancel must inspect then revoke; never Begin or send Goal");port.stop();adapter.stop();executor.cancel();executor.remove_node(node);
  }
  node.reset();rclcpp::shutdown();std::cout<<"PASS bounded port overload and fail-closed latch; API unit, no Action Server/Goal\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";rclcpp::shutdown();return 1;}
}
