// SPDX-License-Identifier: MIT
#pragma once
#include "delivery.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
namespace cmr::mission {
struct DeliveryHealth {bool healthy{false};std::string reason,clock_epoch;std::int64_t now_ms{},source_now_ns{};PoseObservation pose;};
struct DeliveryMonitorConfig {std::string run,epoch,ns,topic_prefix,action;bool synthetic{false};};
class DeliveryMonitor {
 struct Impl;std::unique_ptr<Impl> p;
 public:
 DeliveryMonitor(rclcpp::Node::SharedPtr,const StationCatalog&,DeliveryMonitorConfig);
 ~DeliveryMonitor();void tick(std::int64_t now_ms,bool action_ready);
 std::shared_ptr<const DeliveryHealth> snapshot()const;
 std::int64_t source_time()const;
};
}
