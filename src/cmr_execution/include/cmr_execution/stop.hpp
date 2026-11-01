// SPDX-License-Identifier: MIT
#pragma once
#include "gate.hpp"
#include <deque>
namespace cmr::execution {
struct Binding{Identity who;std::string clock,lease;std::uint64_t generation{};bool operator==(const Binding& b)const{return who==b.who&&clock==b.clock&&lease==b.lease&&generation==b.generation;}};
struct Sample{Binding binding;Ns stamp{};Ms received{};double x{},y{},yaw{},v{},w{},output_v{},output_w{};bool output_fresh{false};};
struct StopConfig{Ns span{3000000000},gap{500000000};Ms fresh{200};std::size_t samples{5},capacity{1024};double v{.01},w{.03},drift{.02},yaw{.03},zero{.001};};
struct StopResult{std::string verdict{"STOP_UNKNOWN"},reason{"NO_DATA"};Ns from{},to{},max_gap{};std::size_t samples{};double drift{},yaw{};};
class StopWindow{
 StopConfig c;std::deque<Sample> q;bool bad{false};std::string reason{"NO_DATA"};
 public:
 explicit StopWindow(StopConfig cfg={}):c(cfg){if(c.span<=0||c.gap<=0||c.fresh<=0||c.samples<2||c.capacity<c.samples||!std::isfinite(c.v)||!std::isfinite(c.w)||!std::isfinite(c.drift)||!std::isfinite(c.yaw)||!std::isfinite(c.zero)||c.v<0||c.w<0||c.drift<0||c.yaw<0||c.zero<0)throw std::invalid_argument("invalid stop config");}
 void reset(const std::string& why){q.clear();bad=false;reason=why;}
 void add(Sample s){
  for(double x:{s.x,s.y,s.yaw,s.v,s.w,s.output_v,s.output_w})if(!std::isfinite(x)){reset("NONFINITE");bad=true;return;}
  if(s.stamp<=0){reset("STAMP");bad=true;return;}
  if(!q.empty()&&!(q.back().binding==s.binding))reset("BINDING_CHANGED");
  if(!q.empty()&&(s.stamp<=q.back().stamp||s.received<q.back().received)){reset("TIME_DISCONTINUITY");bad=true;return;}
  if(!q.empty()&&(s.stamp-q.back().stamp>c.gap||s.received-q.back().received>=c.fresh))reset("SAMPLE_OR_RECEIVE_GAP");
  if(bad){bad=false;q.clear();}
  q.push_back(std::move(s));while(q.size()>c.capacity)q.pop_front();
  while(q.size()>2&&q[1].stamp<=q.back().stamp-c.span)q.pop_front();
 }
 StopResult evaluate(Ms now,Ns ros,const Binding& expected)const{
  StopResult r;r.reason=reason;if(q.empty()||bad)return r;const auto& first=q.front();const auto& last=q.back();r.from=first.stamp;r.to=last.stamp;r.samples=q.size();
  if(!(last.binding==expected)){r.reason="IDENTITY";return r;}
  if(now<last.received||now-last.received>=c.fresh||ros<last.stamp||ros-last.stamp>=c.gap){r.reason="STALE";return r;}
  if(q.size()<c.samples||last.stamp-first.stamp<c.span){r.reason="SHORT_WINDOW";return r;}
  Ns prev=first.stamp;for(const auto& s:q){r.max_gap=std::max(r.max_gap,s.stamp-prev);prev=s.stamp;r.drift=std::max(r.drift,std::hypot(s.x-first.x,s.y-first.y));r.yaw=std::max(r.yaw,std::abs(std::remainder(s.yaw-first.yaw,2*cmr::pi)));
   if(!(s.binding==expected)||!s.output_fresh||std::abs(s.output_v)>c.zero||std::abs(s.output_w)>c.zero){r.reason="OUTPUT_OR_IDENTITY";return r;}
   if(std::abs(s.v)>c.v||std::abs(s.w)>c.w){r.verdict="MOVING";r.reason="VELOCITY";return r;}}
  if(r.max_gap>c.gap){r.reason="SAMPLE_GAP";return r;}
  if(r.drift>c.drift||r.yaw>c.yaw){r.verdict="MOVING";r.reason="DRIFT";return r;}
  r.verdict="STOPPED";r.reason="WINDOW_PROVEN";return r;
 }
};
}
