// SPDX-License-Identifier: MIT
#pragma once
#include "mission_store.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>

namespace cmr::mission {
struct AdapterConfig {
  std::string action;
  std::string epoch;
  bool synthetic_revoke{true};
  int readiness_ms{1500};
  int accept_ms{1500};
  int execute_ms{10000};
  int cancel_ms{1000};
  int collect_ms{3000};
  bool simulation{false};
  bool goal_payload_required{false};
  std::function<std::int64_t()> source_now_ns;
};

class NavigationAdapter final : public Port {
  struct Impl;
  std::shared_ptr<Impl> p;

public:
  using EventSink = std::function<bool(Event)>;
  NavigationAdapter(rclcpp::Node::SharedPtr, AdapterConfig, EventSink);
  ~NavigationAdapter();
  void execute(const Effect &) override; // owner thread: queues only
  void tick();                           // executor thread only
  void begin_shutdown();                 // executor: prohibit new sends while collecting old goals
  void stop();                           // executor no longer executes callbacks after this
  bool faulted() const;
  bool ready() const;
};
} // namespace cmr::mission
