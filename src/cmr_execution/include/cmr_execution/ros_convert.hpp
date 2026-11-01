// SPDX-License-Identifier: MIT
#pragma once
#include "gate.hpp"
#include <cmr_interfaces/msg/execution_identity.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
namespace cmr::execution {
// Validate wire time without throwing rclcpp::Time on malformed input.
inline Ns checked_stamp(const builtin_interfaces::msg::Time& t) noexcept {if(t.sec<0||t.nanosec>=1000000000U)return -1;return static_cast<Ns>(t.sec)*1000000000LL+t.nanosec;}
inline bool publisher_matches(rclcpp::Node& node,const std::string& topic,const std::string& expected_name,const std::string& expected_ns,const rclcpp::MessageInfo& info){
 auto pubs=node.get_publishers_info_by_topic(topic);if(pubs.size()!=1||pubs[0].node_name()!=expected_name||pubs[0].node_namespace()!=expected_ns)return false;
 const auto& gid=pubs[0].endpoint_gid();const auto& actual=info.get_rmw_message_info().publisher_gid;
 return std::equal(gid.begin(),gid.end(),actual.data);
}

inline Identity from(const cmr_interfaces::msg::ExecutionIdentity& x){return {x.run_id,x.gate_epoch,x.owner_epoch,x.mission_id,x.operation_id,x.attempt_id};}
inline cmr_interfaces::msg::ExecutionIdentity to(const Identity& x){cmr_interfaces::msg::ExecutionIdentity m;m.run_id=x.run;m.gate_epoch=x.gate;m.owner_epoch=x.owner;m.mission_id=x.mission;m.operation_id=x.operation;m.attempt_id=x.attempt;return m;}
inline std::array<double,6> from(const geometry_msgs::msg::Twist& x){return {x.linear.x,x.linear.y,x.linear.z,x.angular.x,x.angular.y,x.angular.z};}
inline geometry_msgs::msg::Twist to(const std::array<double,6>& x){geometry_msgs::msg::Twist m;m.linear.x=x[0];m.linear.y=x[1];m.linear.z=x[2];m.angular.x=x[3];m.angular.y=x[4];m.angular.z=x[5];return m;}
}
