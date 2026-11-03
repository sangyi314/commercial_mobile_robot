// SPDX-License-Identifier: MIT
#include "cmr_mission/delivery.hpp"
#include <iostream>
#include <stdexcept>
using namespace cmr::mission;
void ck(bool b,const char* why){if(!b)throw std::runtime_error(why);}
ArrivalWindow window(double x=0,double yaw=3.14){ArrivalWindow w;for(int i=0;i<=70;++i)w.add({{x,0,yaw},10000000000LL+i*50000000LL,1000+i*50,"m.out.0","clock",true,true});return w;}
ArrivalDecision check(const ArrivalWindow& w,std::int64_t now=4500,std::string id="m.out.0",std::string epoch="clock"){return w.evaluate({0,0,-3.14},id,epoch,10000000000LL,10200000000LL,13500000000LL,13500000000LL,now);}
int main(){try{
 auto w=window();ck(check(w).arrived,"wrap and continuous qualified window");ck(!check(w,4700).arrived,"equal receive freshness deadline");ck(!check(w,4500,"old").arrived,"identity");ck(!check(w,4500,"m.out.0","old").arrived,"clock epoch");ck(!check(window(.251)).arrived,"position");ck(!check(window(0,2.8)).arrived,"angle");
 w=window();w.add({{.5,0,3.14},13550000000LL,4550,"m.out.0","clock",true,true});ck(w.jumped(),"localization jump");ck(!check(w,4550).arrived,"jump fail closed");w=window();w.add({{0,0,3.14},1000000000,4550,"m.out.0","clock",true,true});ck(w.jumped()&&!check(w,4550).arrived,"rollback");
 w=window();w.add({{0,0,3.14},13550000000LL,4550,"m.out.0","clock",false,true});ck(!check(w,4550).arrived,"TF missing clears");w=window();w.add({{0,0,3.14},13900000000LL,4900,"m.out.0","clock",true,true});ck(!check(w,4900).arrived,"gap resets window");
 ck(!window().evaluate({0,0,3.14},"m.out.0","clock",14000000000LL,14000000000LL,14500000000LL,14500000000LL,4500).arrived,"preterminal samples");
 bool rejected=false;try{strict_json("{\"x\":1,\"x\":2}");}catch(...){rejected=true;}ck(rejected,"duplicate JSON key");rejected=false;try{checked_pose(strict_json(R"({"frame":"map","position":{"x":0,"y":0,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":0}})"));}catch(...){rejected=true;}ck(rejected,"zero quaternion");
 std::cout<<"PASS arrival windows, wrap, staleness, identity, jump, TF, clock and JSON counterexamples\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
