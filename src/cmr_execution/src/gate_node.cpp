// SPDX-License-Identifier: MIT
#include "cmr_execution/ros_convert.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <cmr_interfaces/msg/gate_state.hpp>
#include <cmr_interfaces/msg/stop_report.hpp>
#include <cmr_interfaces/srv/execution_lease.hpp>
#include <fstream>
#include <iostream>
#include <set>
#include <atomic>
#include <csignal>
#include <thread>
using namespace cmr::execution;
using Status=cmr_interfaces::msg::GateState;using Stop=cmr_interfaces::msg::StopReport;using Lease=cmr_interfaces::srv::ExecutionLease;using Steady=std::chrono::steady_clock;
std::atomic<bool> exit_requested{false};void interrupted(int){exit_requested=true;}
class GateNode:public rclcpp::Node{
 Steady::time_point start{Steady::now()};std::unique_ptr<Gate> gate;Config config;std::string mode,source,source_ns,output,report_node;bool closing{false};
 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;rclcpp::Publisher<Status>::SharedPtr status;
 rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr candidate;rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock;rclcpp::Subscription<Stop>::SharedPtr stop;
 rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr frozen_parameters;
 rclcpp::Service<Lease>::SharedPtr service;rclcpp::TimerBase::SharedPtr timer,graph_timer;
 struct Pending{Candidate value;Ms received;};std::optional<Pending> pending;
 public:
 Ms now()const{return std::chrono::duration_cast<std::chrono::milliseconds>(Steady::now()-start).count();}
 GateNode():Node("execution_gate",rclcpp::NodeOptions().use_clock_thread(false)){
  mode=declare_parameter<std::string>("mode","refuse");const auto run=declare_parameter<std::string>("run_id","");const auto owner=declare_parameter<std::string>("owner_epoch","");source=declare_parameter<std::string>("source_node","");source_ns=declare_parameter<std::string>("source_namespace",get_namespace());report_node=declare_parameter<std::string>("report_node","execution_observer");output=declare_parameter<std::string>("output_topic","final");
  config.tick=declare_parameter("tick_ms",20);config.lease=declare_parameter("lease_ms",300);config.challenge=declare_parameter("challenge_ms",100);config.receive_age=declare_parameter("receive_age_ms",200);config.clock_stall=declare_parameter("clock_stall_ms",300);config.source_age=declare_parameter("source_age_ms",300)*1000000LL;
  config.max_v=declare_parameter("max_v",.2);config.max_w=declare_parameter("max_w",1.);config.frame=declare_parameter<std::string>("frame","base_footprint");
  const std::set<std::string> allowed={"mode","run_id","owner_epoch","source_node","source_namespace","report_node","output_topic","tick_ms","lease_ms","challenge_ms","receive_age_ms","clock_stall_ms","source_age_ms","max_v","max_w","frame","use_sim_time"};
  for(const auto& p:get_node_parameters_interface()->get_parameter_overrides())if(!allowed.count(p.first))throw std::runtime_error("unknown Gate parameter "+p.first);
  const char* domain=getenv("ROS_DOMAIN_ID");const auto ns=std::string(get_namespace());
  if((mode!="sink"&&mode!="simulation")||!domain||get_parameter("use_sim_time").as_bool()||source.empty()||(mode=="sink"&&(std::string(domain)!="72"||ns.rfind("/cmr_test/",0)!=0))||(mode=="simulation"&&(std::string(domain)!="71"||ns.rfind("/cmr_execution/",0)!=0)))throw std::runtime_error("isolated mode/domain/explicit clock required; implicit use_sim_time must be false; production unsupported");
  if(output!="final")throw std::runtime_error("final output must remain within run namespace");
  std::ifstream random("/proc/sys/kernel/random/uuid");std::string epoch;random>>epoch;if(epoch.empty())throw std::runtime_error("Gate epoch unavailable");gate=std::make_unique<Gate>(config,run,epoch,owner);
  const auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  pub=create_publisher<geometry_msgs::msg::Twist>("final",qos);status=create_publisher<Status>("state",qos);
  candidate=create_subscription<geometry_msgs::msg::TwistStamped>("candidate",qos,[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m,const rclcpp::MessageInfo& info){if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("candidate"),source,source_ns,info)){gate->route(false);pending.reset();return;}pending=Pending{Candidate{from(m->twist),checked_stamp(m->header.stamp),m->header.frame_id},now()};});
  clock=create_subscription<rosgraph_msgs::msg::Clock>("clock",rclcpp::QoS(1).best_effort(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr m,const rclcpp::MessageInfo& info){if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("clock"),mode=="sink"?source:"execution_bridge",get_namespace(),info))return;gate->clock(checked_stamp(m->clock),now());});
  stop=create_subscription<Stop>("stop_report",qos,[this](Stop::ConstSharedPtr m,const rclcpp::MessageInfo& info){
   if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("stop_report"),report_node,get_namespace(),info))return;
   if(m->lease_id!=gate->lease()||m->from_ros_ns<=0||m->max_gap_ns<0||m->drift_m<0||m->yaw_drift_rad<0||m->synthetic!=(mode=="sink")||m->verdict!="STOPPED"||m->samples<5||m->to_ros_ns-m->from_ros_ns<3000000000LL||m->max_gap_ns>500000000||m->to_ros_ns>gate->ros()||gate->ros()-m->to_ros_ns>=200000000||!std::isfinite(m->drift_m)||!std::isfinite(m->yaw_drift_rad)||m->drift_m>.02||m->yaw_drift_rad>.03)return;
   gate->stationary(from(m->identity),m->clock_epoch,m->generation,now(),200);
  });
  service=create_service<Lease>("lease",[this](Lease::Request::SharedPtr q,Lease::Response::SharedPtr r){
   const auto old_generation=gate->generation();Request req{q->operation,q->request_id,q->lease_id,q->challenge,from(q->identity),q->request_seq,q->old_goal_resolved};Reply reply;
   if(q->operation=="inspect")reply={"STATUS","",gate->lease(),gate->deadline(),0};
   else if(closing)reply={"CLOSED","",gate->lease(),gate->deadline(),0};
   else if(q->operation=="challenge_begin"||q->operation=="challenge_renew"){req.verb=q->operation=="challenge_begin"?"begin":"renew";reply=gate->issue(req,now());}
   else reply=gate->control(req,now());
   if(old_generation!=gate->generation())pending.reset();
   publish("control");r->code=reply.code;r->state=name(gate->state());r->reason=gate->reason();r->gate_epoch=gate->epoch_id();r->clock_epoch=gate->clock_epoch();r->lease_id=reply.lease;r->challenge=reply.challenge;r->deadline_gate_ms=reply.deadline;r->remaining_ms=std::max<Ms>(0,gate->deadline()-now());r->control_seq=reply.sequence;r->generation=gate->generation();r->ros_ns=gate->ros();
  });
  graph_timer=create_wall_timer(std::chrono::milliseconds(100),[this]{qualify();});
  timer=create_wall_timer(std::chrono::milliseconds(config.tick),[this]{
   if(pending){const auto stamp=pending->value.stamp;if(stamp<=gate->ros()||now()-pending->received>=config.receive_age){gate->candidate(pending->value,pending->received,now());pending.reset();}}
   publish();
  });
  frozen_parameters=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& ps){rcl_interfaces::msg::SetParametersResult r;r.successful=ps.empty();r.reason="Gate parameters frozen; explicit source clock only";return r;});
  std::cout<<"GATE_READY epoch="<<epoch<<" mode="<<mode<<std::endl;
 }
 void qualify(){
  bool ok=true;const auto ns=std::string(get_namespace());auto candidates=get_publishers_info_by_topic(ns+"/candidate");if(candidates.size()!=1)ok=false;for(const auto& p:candidates)if(p.node_name()!=source||p.node_namespace()!=source_ns||p.topic_type()!="geometry_msgs/msg/TwistStamped")ok=false;
  auto finals=get_publishers_info_by_topic(ns+"/final");if(finals.size()!=1||finals.front().node_name()!=get_name())ok=false;
  for(const auto& s:get_subscriptions_info_by_topic(ns+"/final")){const auto& name=s.node_name();if(mode=="sink"&&name=="execution_bridge")ok=false;if(s.topic_type()!="geometry_msgs/msg/Twist"||((s.node_namespace()!=ns|| (name!="execution_observer"&&name!=source&&name!="execution_sink"&&name!="execution_bridge"))&&!(s.node_namespace()=="/"&&(name.rfind("cmr_recorder_",0)==0||(mode=="simulation"&&name=="cmr_worker")))))ok=false;}
  auto observers=get_publishers_info_by_topic(ns+"/stop_report");if(observers.size()!=1||observers.front().node_name()!=report_node||observers.front().node_namespace()!=ns)ok=false;
  gate->route(ok);
 }
 void publish(const std::string& trigger="tick"){const auto d=gate->tick(now());auto u=to(d.u);pub->publish(u);Status s;s.identity=to(gate->identity());s.clock_epoch=gate->clock_epoch();s.lease_id=gate->lease();s.state=name(d.state);s.reason=d.reason;s.synthetic=mode=="sink";s.allowed=gate->active();s.gate_steady_ms=now();s.ros_ns=gate->ros();s.output_seq=d.output_seq;s.control_seq=d.control_seq;s.generation=d.generation;s.stamp_mutations=gate->mutations();s.output=u;s.trigger=trigger;status->publish(s);}
 void shutdown_gate(){closing=true;pending.reset();gate->trip("SHUTDOWN");publish("shutdown");}
};
int main(int argc,char** argv){rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,interrupted);std::signal(SIGTERM,interrupted);try{auto node=std::make_shared<GateNode>();rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);while(rclcpp::ok()&&!exit_requested){exec.spin_some(std::chrono::milliseconds(5));std::this_thread::sleep_for(std::chrono::milliseconds(1));}node->shutdown_gate();auto until=Steady::now()+std::chrono::milliseconds(3500);while(rclcpp::ok()&&Steady::now()<until){exec.spin_some(std::chrono::milliseconds(5));std::this_thread::sleep_for(std::chrono::milliseconds(1));}exec.cancel();exec.remove_node(node);node.reset();rclcpp::shutdown();return 0;}catch(const std::exception& e){std::cerr<<"GATE_REFUSED "<<e.what()<<std::endl;rclcpp::shutdown();return 2;}}
