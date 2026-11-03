// SPDX-License-Identifier: MIT
#include "cmr_mission/nav2_adapter.hpp"
#include "cmr_mission/action_correlation.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <mutex>
#include <deque>
#include <set>
#include <map>
#include <iomanip>
#include <sstream>
#include <atomic>
#include <iostream>
namespace cmr::mission {
namespace {
using Action=nav2_msgs::action::NavigateToPose;using Client=rclcpp_action::Client<Action>;using Handle=rclcpp_action::ClientGoalHandle<Action>;using Clock=std::chrono::steady_clock;
std::string uuid(const rclcpp_action::GoalUUID& id){std::ostringstream s;for(auto c:id)s<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(c);return s.str();}
}
struct NavigationAdapter::Impl:std::enable_shared_from_this<NavigationAdapter::Impl> {
 struct Goal {Effect effect;Handle::SharedPtr handle;std::string uuid;Clock::time_point created{Clock::now()},sent{},cancel_time{};bool dispatched{false},accepted{false},terminal{false},cancel_pending{false},cancel_sent{false},timed_out{false},cancel_timeout{false},cancel_ack{false},collection_timeout{false};std::unique_ptr<ActionCorrelation> correlation;};
 rclcpp::Node::WeakPtr node;Client::SharedPtr client;AdapterConfig config;EventSink sink;
 std::mutex mutex;std::deque<Effect> commands;std::map<std::string,std::shared_ptr<Goal>> goals;std::set<std::string> effects;
 std::deque<Event> critical;std::atomic<bool> fault{false},alive{true};std::uint64_t seq{0},feedback_dropped{0};bool closing{false};
 Event event(const std::shared_ptr<Goal>& g,Kind kind,std::string detail){Event e;e.id="ros-"+config.epoch+"-"+std::to_string(++seq);e.mission=g->effect.mission;e.epoch=config.epoch;e.operation=g->effect.operation;e.attempt=g->effect.attempt;e.uuid=g->uuid;e.kind=kind;e.detail=std::move(detail);return e;}
 void emit(Event e,bool important=true){if(!alive)return;std::cout<<"CALLBACK "<<e.id<<" kind="<<static_cast<int>(e.kind)<<" uuid="<<e.uuid<<" "<<e.detail<<std::endl;if(important){if(critical.size()>=64){fault=true;std::cerr<<"CRITICAL_EVENT_OVERFLOW locked; new sends prohibited\n";return;}critical.push_back(std::move(e));}else if(!critical.empty()||!sink(std::move(e)))++feedback_dropped;}
 void correlated(const std::shared_ptr<Goal>& g,Event e){
  auto ready=g->correlation->push(std::move(e));
  if(g->correlation->fault){fault=true;emit(event(g,Kind::Fatal,"CORRELATION_FAULT"));return;}
  for(auto& r:ready){if(r.kind==Kind::Succeeded||r.kind==Kind::Aborted||r.kind==Kind::CanceledResult)g->terminal=true;emit(r);}
 }
 void expire_before_callback(const std::shared_ptr<Goal>& g,bool acceptance){
  if(g->timed_out||g->terminal||!g->dispatched)return;
  const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-g->sent).count();
  if((acceptance&&!g->accepted&&deadline_elapsed(elapsed,config.accept_ms))||deadline_elapsed(elapsed,config.execute_ms)){
   g->timed_out=true;g->cancel_pending=true;
   emit(event(g,Kind::Fatal,acceptance?"LATE_ACCEPT_TIMEOUT":"LATE_EXECUTION_TIMEOUT"));
  }
 }
 void cancel(const std::shared_ptr<Goal>& g){g->cancel_pending=true;if(!g->handle || g->terminal || g->cancel_sent)return;g->cancel_sent=true;g->cancel_time=Clock::now();auto weak=weak_from_this();
  try{client->async_cancel_goal(g->handle,[weak,g](Client::CancelResponse::SharedPtr response){if(auto self=weak.lock();self&&self->alive){g->cancel_ack=true;self->emit(self->event(g,Kind::CancelAck,"CANCEL_RESPONSE code="+std::to_string(response->return_code)+" count="+std::to_string(response->goals_canceling.size())));}});}catch(const std::exception& e){emit(event(g,Kind::Fatal,std::string("CANCEL_UNKNOWN ")+e.what()));}}
};

NavigationAdapter::NavigationAdapter(rclcpp::Node::SharedPtr node,AdapterConfig config,EventSink sink):p(std::make_shared<Impl>()){
 if((config.simulation?(config.action!="/navigate_to_pose"||!getenv("ROS_DOMAIN_ID")||std::string(getenv("ROS_DOMAIN_ID"))!="71"):config.action.rfind("/cmr_test/",0)!=0)||config.epoch.empty()||config.readiness_ms<=0||config.accept_ms<=0||config.execute_ms<=0||config.cancel_ms<=0||config.collect_ms<=0)throw std::runtime_error("invalid adapter config");
 p->node=node;p->config=std::move(config);p->sink=std::move(sink);p->client=rclcpp_action::create_client<Action>(node,p->config.action);
}
NavigationAdapter::~NavigationAdapter(){stop();}
void NavigationAdapter::execute(const Effect& e){std::lock_guard<std::mutex> lock(p->mutex);if(!p->alive || p->fault)throw std::runtime_error("adapter disarmed");if(p->commands.size()>=32){p->fault=true;throw std::runtime_error("adapter command overflow");}p->commands.push_back(e);}
bool NavigationAdapter::faulted()const{return p->fault;}
bool NavigationAdapter::ready()const{return p->alive&&p->client->action_server_is_ready();}
void NavigationAdapter::begin_shutdown(){p->closing=true;}
void NavigationAdapter::stop(){if(!p||!p->alive.exchange(false))return;for(auto& pair:p->goals)if(pair.second->handle)p->client->stop_callbacks(pair.second->handle);p->critical.clear();std::cout<<"ADAPTER_CLOSED feedback_dropped="<<p->feedback_dropped<<std::endl;}
void NavigationAdapter::tick(){if(!p->alive)return;auto node=p->node.lock();if(!node)return;
 while(!p->critical.empty()){if(!p->sink(p->critical.front()))break;p->critical.pop_front();}
 std::deque<Effect> commands;{std::lock_guard<std::mutex> lock(p->mutex);commands.swap(p->commands);}
 for(const auto& e:commands){if(!p->effects.insert(e.id).second)continue;
  auto found=p->goals.find(e.attempt);
  if(e.kind==EffectKind::Send){if(p->fault)continue;for(const auto& pair:p->goals)if(!pair.second->terminal){p->fault=true;throw std::runtime_error("second unresolved adapter goal");}if(p->goals.size()>=8){for(auto it=p->goals.begin();it!=p->goals.end();){if(it->second->terminal)it=p->goals.erase(it);else ++it;}}
   auto g=std::make_shared<Impl::Goal>();g->effect=e;g->correlation=std::make_unique<ActionCorrelation>(p->config.epoch,e);p->goals[e.attempt]=g;
  }else if(found!=p->goals.end()){
   auto g=found->second;if(e.kind==EffectKind::Revoke){if(p->config.synthetic_revoke)p->emit(p->event(g,Kind::RevokeAck,"synthetic=true REVOKE_ACK"));if(!g->terminal)p->cancel(g);}else if(e.kind==EffectKind::CancelExact){if(!e.uuid.empty()&&!g->uuid.empty()&&e.uuid!=g->uuid){p->fault=true;continue;}p->cancel(g);}
  }
 }
 for(auto& pair:p->goals){auto g=pair.second;if(g->terminal)continue;auto now=Clock::now();auto age=[&](Clock::time_point t){return std::chrono::duration_cast<std::chrono::milliseconds>(now-t).count();};
  if(!g->dispatched){if(p->closing){g->terminal=true;p->emit(p->event(g,Kind::Rejected,"NOT_SENT SHUTDOWN"));continue;}if(p->client->action_server_is_ready()){
    std::size_t providers=0;
    for(const auto& full:node->get_node_names()){
     const auto slash=full.rfind('/');if(slash==std::string::npos)continue;
     const auto node_name=full.substr(slash+1),node_ns=slash==0?std::string("/"):full.substr(0,slash);
     const auto services=node->get_service_names_and_types_by_node(node_name,node_ns);
     if(services.count(p->config.action+"/_action/send_goal"))++providers;
    }
    if(providers>1){g->terminal=true;p->fault=true;p->emit(p->event(g,Kind::Rejected,"ISOLATION_EXTRA_SERVER count="+std::to_string(providers)));continue;}

    Action::Goal goal;goal.pose.header.frame_id="map";goal.pose.pose.orientation.w=1.0;goal.pose.pose.position.x=g->effect.operation.find(".return")!=std::string::npos?0.0:1.0;
    try{if(p->config.goal_payload_required&&g->effect.navigation_payload.empty())throw std::runtime_error("frozen target required");if(!g->effect.navigation_payload.empty()){
     const auto payload=strict_json(g->effect.navigation_payload);const auto target=payload.at("target");checked_pose(target);const auto bt=payload.at("bt");if(file_sha256(bt.at("path"))!=bt.at("sha256"))throw std::runtime_error("BT resource changed");
     goal.pose.header.frame_id=target.at("frame");goal.pose.pose.position.x=target.at("position").at("x");goal.pose.pose.position.y=target.at("position").at("y");goal.pose.pose.position.z=0;goal.pose.pose.orientation.x=0;goal.pose.pose.orientation.y=0;goal.pose.pose.orientation.z=target.at("orientation").at("z");goal.pose.pose.orientation.w=target.at("orientation").at("w");goal.behavior_tree=bt.at("path");
    }}catch(const std::exception& error){g->terminal=true;p->emit(p->event(g,Kind::Rejected,std::string("NOT_SENT TARGET_INVALID ")+error.what()));continue;}
    g->dispatched=true;g->sent=now;goal.pose.header.stamp=rclcpp::Time(p->config.source_now_ns?p->config.source_now_ns():node->now().nanoseconds());
    std::cout<<"NAV_DISPATCH "<<Json{{"mission",g->effect.mission},{"operation",g->effect.operation},{"attempt",g->effect.attempt},{"effect",g->effect.id},{"target_x",goal.pose.pose.position.x},{"target_y",goal.pose.pose.position.y},{"quaternion_z",goal.pose.pose.orientation.z},{"quaternion_w",goal.pose.pose.orientation.w},{"bt",goal.behavior_tree}}.dump()<<std::endl;
    auto weak=std::weak_ptr<Impl>(p);Client::SendGoalOptions opts;
    opts.goal_response_callback=[weak,g](Handle::SharedPtr handle){if(auto self=weak.lock();self&&self->alive){self->expire_before_callback(g,true);if(!handle){g->terminal=true;self->emit(self->event(g,Kind::Rejected,"ACTION_REJECTED"));return;}g->handle=handle;g->uuid=uuid(handle->get_goal_id());g->accepted=true;self->correlated(g,self->event(g,Kind::Accepted,"ACTION_ACCEPTED"));if(g->cancel_pending)self->cancel(g);}};
    opts.feedback_callback=[weak,g](Handle::SharedPtr handle,std::shared_ptr<const Action::Feedback> feedback){if(auto self=weak.lock();self&&self->alive){if(!g->accepted||uuid(handle->get_goal_id())!=g->uuid)return;self->emit(self->event(g,Kind::Feedback,"FEEDBACK remaining="+std::to_string(feedback->distance_remaining)+" internal_recoveries="+std::to_string(feedback->number_of_recoveries)),false);}};
    opts.result_callback=[weak,g](const Handle::WrappedResult& result){if(auto self=weak.lock();self&&self->alive){self->expire_before_callback(g,false);const auto id=uuid(result.goal_id);if(!g->uuid.empty()&&id!=g->uuid){self->emit(self->event(g,g->terminal?Kind::Feedback:Kind::Fatal,"UUID_CONFLICT"));return;}Kind kind=Kind::Fatal;switch(result.code){case rclcpp_action::ResultCode::SUCCEEDED:kind=Kind::Succeeded;break;case rclcpp_action::ResultCode::ABORTED:kind=Kind::Aborted;break;case rclcpp_action::ResultCode::CANCELED:kind=Kind::CanceledResult;break;default:break;}
      auto e=self->event(g,kind,"ACTION_RESULT code="+std::to_string(static_cast<int>(result.code)));e.uuid=id;if(auto node=self->node.lock())e.source_to_ns=self->config.source_now_ns?self->config.source_now_ns():node->now().nanoseconds();
      self->correlated(g,std::move(e));
    }};
    std::cout<<"SEND_CALL "<<g->effect.id<<std::endl;
    try{p->client->async_send_goal(goal,opts);}catch(const std::exception& ex){g->timed_out=true;p->emit(p->event(g,Kind::Fatal,std::string("SEND_UNKNOWN ")+ex.what()));}
   }else if(age(g->created)>=p->config.readiness_ms){g->terminal=true;p->emit(p->event(g,Kind::Rejected,"NOT_SENT READINESS_TIMEOUT"));}
  }else if(!g->timed_out && ((!g->accepted&&age(g->sent)>=p->config.accept_ms)||(g->accepted&&age(g->sent)>=p->config.execute_ms))){g->timed_out=true;g->cancel_pending=true;p->emit(p->event(g,Kind::Fatal,g->accepted?"EXECUTION_TIMEOUT":"ACCEPT_TIMEOUT"));p->cancel(g);}
  if(g->cancel_sent&&!g->cancel_ack&&!g->cancel_timeout&&age(g->cancel_time)>=p->config.cancel_ms){g->cancel_timeout=true;p->emit(p->event(g,Kind::Fatal,"CANCEL_ACK_TIMEOUT"));}
  if(g->cancel_sent&&!g->collection_timeout&&age(g->cancel_time)>=p->config.collect_ms){g->collection_timeout=true;p->emit(p->event(g,Kind::Fatal,"TERMINAL_COLLECTION_TIMEOUT"));}
 }
}
}
