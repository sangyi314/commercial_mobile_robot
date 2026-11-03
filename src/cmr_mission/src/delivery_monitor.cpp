// SPDX-License-Identifier: MIT
#include "cmr_mission/delivery_monitor.hpp"
#include <cmr_execution/ros_convert.hpp>
#include <cmr_interfaces/msg/gate_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <atomic>
#include <cmath>
namespace cmr::mission {
using GateState=cmr_interfaces::msg::GateState;namespace x=cmr::execution;
struct DeliveryMonitor::Impl {
 rclcpp::Node::WeakPtr node;const StationCatalog* catalog;DeliveryMonitorConfig config;
 std::unique_ptr<tf2_ros::Buffer> buffer;std::unique_ptr<tf2_ros::TransformListener> listener;
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
 rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl;
 rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock;
 rclcpp::Subscription<GateState>::SharedPtr gate;
 std::int64_t now{},scan_at{-1000000},odom_at{-1000000},clock_progress{-1000000},source_ns{},gate_at{-1000000},graph_at{-1000000};bool map_good{false},localization_good{false},clock_fault{false},graph_good{false};std::optional<PoseObservation> previous_pose;bool pose_fault{false};std::optional<GateState> state;std::shared_ptr<const DeliveryHealth> current;
 std::string topic(const std::string& s)const{return config.topic_prefix+"/"+s;}
 bool sender(const std::string& topic,const std::string& physical,const rclcpp::MessageInfo& info)const{auto owner=node.lock();return owner&&x::publisher_matches(*owner,topic,config.synthetic?"execution_sink":physical,config.synthetic?config.ns:(physical=="execution_bridge"?config.ns:"/"),info);}
 bool graph(){
  auto owner=node.lock();if(!owner)return false;auto clients=owner->get_node_graph_interface();
  for(const auto& full:owner->get_node_names()){
   auto pos=full.rfind('/');if(pos==std::string::npos)continue;const auto name=full.substr(pos+1),ns=pos?full.substr(0,pos):"/";
   if(name=="waypoint_follower"||name=="docking_server")return false;
   auto services=clients->get_client_names_and_types_by_node(name,ns);
   if(services.count(config.action+"/_action/send_goal")&&!(name=="mission_node"&&ns==config.ns)&&!(name=="bt_navigator"&&ns=="/"&&!config.synthetic))return false;
  }
  if(!config.synthetic){if(!owner->get_publishers_info_by_topic("/cmd_vel").empty()||!owner->get_subscriptions_info_by_topic("/cmd_vel").empty())return false;
   auto pubs=owner->get_publishers_info_by_topic(config.ns+"/disabled_goal_pose");if(!pubs.empty())return false;}
  return true;
 }
};
DeliveryMonitor::DeliveryMonitor(rclcpp::Node::SharedPtr node,const StationCatalog& catalog,DeliveryMonitorConfig config):p(std::make_unique<Impl>()){
 p->node=node;p->catalog=&catalog;p->config=std::move(config);p->buffer=std::make_unique<tf2_ros::Buffer>(node->get_clock());p->listener=std::make_unique<tf2_ros::TransformListener>(*p->buffer,node,false);
 p->scan=node->create_subscription<sensor_msgs::msg::LaserScan>(p->topic("scan"),rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr s,const rclcpp::MessageInfo& i){if(!p->sender(p->topic("scan"),"execution_bridge",i)||s->ranges.empty())return;bool good=false;for(float v:s->ranges)good=good||(std::isfinite(v)&&v>=s->range_min&&v<=s->range_max);if(good)p->scan_at=p->now;});
 p->odom=node->create_subscription<nav_msgs::msg::Odometry>(p->topic("odom"),rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr s,const rclcpp::MessageInfo& i){if(!p->sender(p->topic("odom"),"execution_bridge",i)||s->header.frame_id!="odom"||(s->child_frame_id!="base_footprint"&&s->child_frame_id!="base_link"))return;auto q=s->pose.pose.orientation;auto v=s->twist.twist;bool good=std::isfinite(s->pose.pose.position.x)&&std::isfinite(s->pose.pose.position.y)&&std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-3;for(double n:{v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z})good=good&&std::isfinite(n);if(good)p->odom_at=p->now;});
 auto durable=rclcpp::QoS(1).reliable().transient_local();
 p->map=node->create_subscription<nav_msgs::msg::OccupancyGrid>(p->topic("map"),durable,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr s,const rclcpp::MessageInfo& i){if(!p->sender(p->topic("map"),"map_server",i))return;try{const auto& m=p->catalog->description().at("map").at("metadata");p->map_good=s->header.frame_id=="map"&&s->info.width==m.at("width").get<unsigned>()&&s->info.height==m.at("height").get<unsigned>()&&std::abs(s->info.resolution-m.at("resolution").get<double>())<1e-7&&std::abs(s->info.origin.position.x-m.at("origin_x").get<double>())<1e-7&&std::abs(s->info.origin.position.y-m.at("origin_y").get<double>())<1e-7&&s->data.size()==static_cast<std::size_t>(s->info.width)*s->info.height&&bytes_sha256(std::string(reinterpret_cast<const char*>(s->data.data()),s->data.size()))==m.at("grid_sha256")&&std::abs(s->info.origin.orientation.z-std::sin(m.at("origin_yaw").get<double>()/2))<1e-7&&std::abs(s->info.origin.orientation.w-std::cos(m.at("origin_yaw").get<double>()/2))<1e-7;}catch(...){p->map_good=false;}});
 p->amcl=node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(p->topic("amcl_pose"),durable,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr s,const rclcpp::MessageInfo& i){if(!p->sender(p->topic("amcl_pose"),"amcl",i))return;auto q=s->pose.pose.orientation;bool good=s->header.frame_id=="map"&&std::isfinite(s->pose.pose.position.x)&&std::isfinite(s->pose.pose.position.y)&&std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-3;for(auto n:{0,7,35})good=good&&std::isfinite(s->pose.covariance[n])&&s->pose.covariance[n]>=0&&s->pose.covariance[n]<=.5;p->localization_good=good;});
 p->clock=node->create_subscription<rosgraph_msgs::msg::Clock>(p->topic("clock"),rclcpp::QoS(1).reliable(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr s,const rclcpp::MessageInfo& i){if(!p->sender(p->topic("clock"),"execution_bridge",i))return;auto t=x::checked_stamp(s->clock);if(t<=0){if(p->source_ns)p->clock_fault=true;return;}if(t<p->source_ns)p->clock_fault=true;if(t!=p->source_ns)p->clock_progress=p->now;p->source_ns=t;});
 p->gate=node->create_subscription<GateState>(p->config.ns+"/state",rclcpp::QoS(1).reliable(),[this](GateState::ConstSharedPtr s,const rclcpp::MessageInfo& i){auto owner=p->node.lock();if(!owner||!x::publisher_matches(*owner,p->config.ns+"/state","execution_gate",p->config.ns,i))return;p->state=*s;p->gate_at=p->now;});
}
DeliveryMonitor::~DeliveryMonitor()=default;
void DeliveryMonitor::tick(std::int64_t now,bool ready){p->now=now;auto h=std::make_shared<DeliveryHealth>();h->now_ms=now;h->source_now_ns=p->source_ns;
 if(now-p->graph_at>=100){try{p->graph_good=p->graph();}catch(...){p->graph_good=false;}p->graph_at=now;}
 h->reason="INPUT_OR_GRAPH_UNQUALIFIED";h->healthy=ready&&p->graph_good&&p->map_good&&p->localization_good&&!p->clock_fault&&now-p->scan_at<500&&now-p->odom_at<200&&now-p->clock_progress<300&&p->state&&now-p->gate_at<100&&(p->state->identity.owner_epoch==p->config.epoch||(p->state->identity.mission_id.empty()&&p->state->state=="DISARMED"&&!p->state->allowed));
 if(p->state)h->clock_epoch=p->state->clock_epoch;
 try{auto t=p->buffer->lookupTransform("map","base_footprint",tf2::TimePointZero);const auto source=x::checked_stamp(t.header.stamp);auto q=t.transform.rotation;double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  h->pose.pose={t.transform.translation.x,t.transform.translation.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};h->pose.source_ns=source;h->pose.received_ms=now;h->pose.clock_epoch=h->clock_epoch;h->pose.localization_valid=p->localization_good;h->pose.tf_valid=source>0&&source<=p->source_ns&&p->source_ns-source<200000000&&std::isfinite(norm)&&std::abs(norm-1)<1e-3;if(h->pose.tf_valid){if(p->previous_pose&&h->pose.clock_epoch==p->previous_pose->clock_epoch&&h->pose.source_ns>p->previous_pose->source_ns){auto& prev=*p->previous_pose;double dt=(h->pose.source_ns-prev.source_ns)*1e-9;double dyaw=std::atan2(std::sin(h->pose.pose.yaw-prev.pose.yaw),std::cos(h->pose.pose.yaw-prev.pose.yaw));if(std::hypot(h->pose.pose.x-prev.pose.x,h->pose.pose.y-prev.pose.y)>.1+.5*dt||std::abs(dyaw)>.1+1.5*dt)p->pose_fault=true;}p->previous_pose=h->pose;}
  h->healthy=h->healthy&&h->pose.tf_valid&&!p->pose_fault;
 }catch(...){h->healthy=false;h->reason="TF_UNAVAILABLE";}
 if(h->healthy)h->reason="HEALTHY";else h->reason=Json{{"action_ready",ready},{"graph",p->graph_good},{"map",p->map_good},{"localization",p->localization_good},{"clock_fault",p->clock_fault},{"pose_fault",p->pose_fault},{"tf_valid",h->pose.tf_valid},{"scan_age_ms",now-p->scan_at},{"odom_age_ms",now-p->odom_at},{"clock_progress_age_ms",now-p->clock_progress},{"gate_age_ms",now-p->gate_at}}.dump();std::atomic_store(&p->current,std::shared_ptr<const DeliveryHealth>(h));
}
std::shared_ptr<const DeliveryHealth> DeliveryMonitor::snapshot()const{return std::atomic_load(&p->current);}
std::int64_t DeliveryMonitor::source_time()const{return p->source_ns;}
}
