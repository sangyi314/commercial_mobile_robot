// SPDX-License-Identifier: MIT
#pragma once
#include "nav2_adapter.hpp"
namespace cmr::mission {
struct ExecutionConfig{std::string run,owner,ns;bool synthetic_observations{true};};
// Owner calls execute only to enqueue persisted effects; ROS work stays on executor.
class ExecutionPort final:public Port{
 struct Impl;std::shared_ptr<Impl> p;
 public:
 ExecutionPort(rclcpp::Node::SharedPtr,ExecutionConfig,NavigationAdapter&,NavigationAdapter::EventSink);
 ~ExecutionPort();bool faulted()const;bool evidence_current(const Record&,std::uint64_t generation)const;void execute(const Effect&)override;void tick();void begin_shutdown();void stop();
};
}
