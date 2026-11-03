// SPDX-License-Identifier: MIT
#include "cmr_execution/stop.hpp"
#include "cmr_execution/ros_convert.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <cmr_interfaces/msg/gate_state.hpp>
#include <cmr_interfaces/msg/stop_report.hpp>
#include <chrono>
#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
using namespace cmr::execution;using Status=cmr_interfaces::msg::GateState;using Stop=cmr_interfaces::msg::StopReport;
class Observer:public rclcpp::Node{
 std::chrono::steady_clock::time_point start{std::chrono::steady_clock::now()};StopWindow window;BodyWindow body;Binding binding;std::optional<Status> state;std::array<double,6> output{};Ms output_at{-10000};Ns clock_ns{};Ms clock_progress{};bool synthetic,body_required;
 rclcpp::Subscription<Status>::SharedPtr state_sub;rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output_sub;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub,body_sub;rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;rclcpp::Publisher<Stop>::SharedPtr pub;rclcpp::TimerBase::SharedPtr timer;rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr frozen_parameters;
 public:
 Ms now()const{return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();}
 Observer():Node("execution_observer",rclcpp::NodeOptions().use_clock_thread(false)){
  auto mode=declare_parameter<std::string>("mode","refuse");const char* domain=getenv("ROS_DOMAIN_ID");auto ns=std::string(get_namespace());synthetic=mode=="sink"||mode=="sink_body";body_required=mode=="simulation"||mode=="sink_body";
  if(!domain||get_parameter("use_sim_time").as_bool()||(mode!="sink"&&mode!="sink_body"&&mode!="simulation")||(synthetic&&(std::string(domain)!="72"||ns.rfind("/cmr_test/",0)!=0))||(!synthetic&&(std::string(domain)!="71"||ns.rfind("/cmr_execution/",0)!=0)))throw std::runtime_error("observer isolated mode required");
  auto qos=rclcpp::QoS(1).reliable().durability_volatile();pub=create_publisher<Stop>("stop_report",qos);
  state_sub=create_subscription<Status>("state",qos,[this](Status::ConstSharedPtr s,const rclcpp::MessageInfo& info){if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("state"),"execution_gate",get_namespace(),info)){window.reset("STATE_SOURCE");return;}Binding next{from(s->identity),s->clock_epoch,s->lease_id,s->generation};if(!(next==binding)){window.reset("NEW_GENERATION");body.reset();binding=next;}state=*s;});
  output_sub=create_subscription<geometry_msgs::msg::Twist>("final",qos,[this](geometry_msgs::msg::Twist::ConstSharedPtr s,const rclcpp::MessageInfo& info){if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("final"),"execution_gate",get_namespace(),info)){window.reset("FINAL_SOURCE");output_at=-10000;return;}output=from(*s);output_at=now();});
  clock_sub=create_subscription<rosgraph_msgs::msg::Clock>("clock",rclcpp::QoS(1).best_effort(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr s,const rclcpp::MessageInfo& info){if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("clock"),synthetic?"execution_sink":"execution_bridge",get_namespace(),info)){window.reset("CLOCK_SOURCE");return;}auto stamp=checked_stamp(s->clock);if(stamp<0){window.reset("INVALID_CLOCK");clock_ns=0;return;}if(stamp<clock_ns){window.reset("CLOCK_ROLLBACK");body.reset();}if(stamp!=clock_ns)clock_progress=now();clock_ns=stamp;});
  odom_sub=create_subscription<nav_msgs::msg::Odometry>("odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr o,const rclcpp::MessageInfo& info){if(!state)return;if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("odom"),synthetic?"execution_sink":"execution_bridge",get_namespace(),info)){window.reset("ODOM_SOURCE");return;}if(o->header.frame_id!="odom"||(o->child_frame_id!="base_footprint"&&o->child_frame_id!="base_link")){window.reset("ODOM_FRAME");return;}auto& p=o->pose.pose;auto& v=o->twist.twist;const auto norm=p.orientation.x*p.orientation.x+p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z+p.orientation.w*p.orientation.w;if(!std::isfinite(norm)||std::abs(norm-1)>.001){window.reset("INVALID_QUATERNION");return;}Sample s;s.binding=binding;s.stamp=checked_stamp(o->header.stamp);s.received=now();s.x=p.position.x;s.y=p.position.y;s.yaw=std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));s.v=std::hypot(std::hypot(v.linear.x,v.linear.y),v.linear.z);s.w=std::hypot(std::hypot(v.angular.x,v.angular.y),v.angular.z);s.output_v=output[0];s.output_w=output[5];s.output_fresh=now()-output_at<200;for(double x:output)if(!std::isfinite(x)||std::abs(x)>.001)s.output_fresh=false;window.add(s);});
  if(body_required)body_sub=create_subscription<nav_msgs::msg::Odometry>("body_odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr o,const rclcpp::MessageInfo& info){
   if(!state)return;
   if(!publisher_matches(*this,get_node_topics_interface()->resolve_topic_name("body_odom"),synthetic?"execution_sink":"execution_bridge",get_namespace(),info)||o->header.frame_id!="body_world"||o->child_frame_id!="body_base"){body.reset();return;}
   const auto& p=o->pose.pose;const auto& v=o->twist.twist;const double norm=p.orientation.x*p.orientation.x+p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z+p.orientation.w*p.orientation.w;
   if(!std::isfinite(norm)||std::abs(norm-1)>.001){body.reset();return;}
   Sample s;s.binding=binding;s.stamp=checked_stamp(o->header.stamp);s.received=now();s.x=p.position.x;s.y=p.position.y;s.yaw=std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));s.v=std::hypot(std::hypot(v.linear.x,v.linear.y),v.linear.z);s.w=std::hypot(std::hypot(v.angular.x,v.angular.y),v.angular.z);body.add(s);
  });
  frozen_parameters=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& ps){rcl_interfaces::msg::SetParametersResult r;r.successful=ps.empty();r.reason="Observer parameters frozen; explicit source clock only";return r;});
  timer=create_wall_timer(std::chrono::milliseconds(20),[this]{if(!state)return;if(now()-clock_progress>=300){window.reset("CLOCK_STALLED");body.reset();}auto r=window.evaluate(now(),clock_ns,binding);if(body_required&&r.verdict=="STOPPED"){auto b=body.evaluate(now(),clock_ns,binding,r.from,r.to);r.verdict=b.verdict;r.reason=b.reason;r.drift=std::max(r.drift,b.drift);r.yaw=std::max(r.yaw,b.yaw);r.max_gap=std::max(r.max_gap,b.max_gap);}Stop m;m.identity=to(binding.who);m.clock_epoch=binding.clock;m.lease_id=binding.lease;m.generation=binding.generation;m.synthetic=synthetic;m.verdict=r.verdict;m.reason=r.reason;m.c_boundary="CONSUMER_ACK_UNKNOWN";m.from_ros_ns=r.from;m.to_ros_ns=r.to;m.max_gap_ns=r.max_gap;m.samples=r.samples;m.drift_m=r.drift;m.yaw_drift_rad=r.yaw;m.evidence_ref=std::string(synthetic?"observer/":"observer/body-pose-v1/")+binding.clock+"/"+std::to_string(binding.generation)+"/"+std::to_string(r.from)+"-"+std::to_string(r.to);pub->publish(m);});
 }
};
static std::atomic<bool> observer_running{true};
static void observer_signal(int){observer_running=false;}
int main(int argc,char** argv){rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,observer_signal);std::signal(SIGTERM,observer_signal);try{auto node=std::make_shared<Observer>();rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);while(rclcpp::ok()&&observer_running){executor.spin_some(std::chrono::milliseconds(5));std::this_thread::sleep_for(std::chrono::milliseconds(1));}executor.cancel();executor.remove_node(node);node.reset();rclcpp::shutdown();return 0;}catch(const std::exception& e){std::cerr<<"OBSERVER_REFUSED "<<e.what()<<std::endl;rclcpp::shutdown();return 2;}}
