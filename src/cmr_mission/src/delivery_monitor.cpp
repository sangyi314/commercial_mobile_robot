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
using GateState = cmr_interfaces::msg::GateState;
namespace x = cmr::execution;

struct DeliveryMonitor::Impl {
  rclcpp::Node::WeakPtr node;
  const StationCatalog *catalog;
  DeliveryMonitorConfig config;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Subscription<GateState>::SharedPtr gate;
  std::int64_t now_ms{};
  std::int64_t scan_at_ms{-1000000};
  std::int64_t odom_at_ms{-1000000};
  std::int64_t clock_progress_ms{-1000000};
  std::int64_t source_ns{};
  std::int64_t gate_at_ms{-1000000};
  std::int64_t graph_at_ms{-1000000};
  bool map_valid{false};
  bool localization_valid{false};
  bool clock_fault{false};
  bool graph_valid{false};
  std::optional<PoseObservation> previous_pose;
  bool pose_fault{false};
  std::optional<GateState> state;
  std::shared_ptr<const DeliveryHealth> current;

  std::string topic(const std::string &s) const {
    return config.topic_prefix + "/" + s;
  }

  bool is_expected_publisher(const std::string &topic, const std::string &expected_node_name,
              const rclcpp::MessageInfo &info) const {
    auto owner = node.lock();
    return owner
           && x::publisher_matches(
               *owner, topic, config.synthetic ? "execution_sink" : expected_node_name,
               config.synthetic ? config.ns : (expected_node_name == "execution_bridge" ? config.ns : "/"),
               info);
  }

  bool check_graph_valid() {
    auto owner = node.lock();
    if (!owner) {
      return false;
    }
    auto graph = owner->get_node_graph_interface();
    for (const auto &full_node_name : graph->get_node_names()) {
      auto pos = full_node_name.rfind('/');
      if (pos == std::string::npos) {
        continue;
      }
      const auto name = full_node_name.substr(pos + 1);
      const auto ns = pos ? full_node_name.substr(0, pos) : "/";
      if (name == "waypoint_follower" || name == "docking_server") {
        return false;
      }
      auto service_clients = graph->get_client_names_and_types_by_node(name, ns);
      if (service_clients.count(config.action + "/_action/send_goal")
          && !(name == "mission_node" && ns == config.ns)
          && !(name == "bt_navigator" && ns == "/" && !config.synthetic)) {
        return false;
      }
    }
    if (!config.synthetic) {
      if (!owner->get_publishers_info_by_topic("/cmd_vel").empty()
    || !owner->get_subscriptions_info_by_topic("/cmd_vel").empty()
    || !owner->get_publishers_info_by_topic(
        config.ns + "/disabled_goal_pose").empty()) {
    return false;
    }
    }
    return true;
  }
};

DeliveryMonitor::DeliveryMonitor(rclcpp::Node::SharedPtr node, const StationCatalog &catalog,
                                 DeliveryMonitorConfig config)
    : p(std::make_unique<Impl>()) {
  p->node = node;
  p->catalog = &catalog;
  p->config = std::move(config);
  p->buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  p->listener = std::make_unique<tf2_ros::TransformListener>(*p->buffer, node, false);
  p->scan = node->create_subscription<sensor_msgs::msg::LaserScan>(
      p->topic("scan"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr s, const rclcpp::MessageInfo &i) {
        if (!p->is_expected_publisher(p->topic("scan"), "execution_bridge", i) || s->ranges.empty()) {
          return;
        }
        bool good = false;
        for (float v : s->ranges) {
          good = good || (std::isfinite(v) && v >= s->range_min && v <= s->range_max);
        }
        if (good) {
          p->scan_at_ms = p->now_ms;
        }
      });
  p->odom = node->create_subscription<nav_msgs::msg::Odometry>(
      p->topic("odom"), rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr s, const rclcpp::MessageInfo &i) {
        if (!p->is_expected_publisher(p->topic("odom"), "execution_bridge", i) || s->header.frame_id != "odom"
            || (s->child_frame_id != "base_footprint" && s->child_frame_id != "base_link")) {
          return;
        }
        auto q = s->pose.pose.orientation;
        auto v = s->twist.twist;
        bool good = std::isfinite(s->pose.pose.position.x) && std::isfinite(s->pose.pose.position.y)
                    && std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) < 1e-3;
        for (double n :
             {v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z}) {
          good = good && std::isfinite(n);
        }
        if (good) {
          p->odom_at_ms = p->now_ms;
        }
      });
  auto durable = rclcpp::QoS(1).reliable().transient_local();
  p->map = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      p->topic("map"), durable,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr s, const rclcpp::MessageInfo &i) {
        if (!p->is_expected_publisher(p->topic("map"), "map_server", i)) {
          return;
        }
        try {
          const auto &m = p->catalog->description().at("map").at("metadata");
          p->map_valid =
              s->header.frame_id == "map" && s->info.width == m.at("width").get<unsigned>()
              && s->info.height == m.at("height").get<unsigned>()
              && std::abs(s->info.resolution - m.at("resolution").get<double>()) < 1e-7
              && std::abs(s->info.origin.position.x - m.at("origin_x").get<double>()) < 1e-7
              && std::abs(s->info.origin.position.y - m.at("origin_y").get<double>()) < 1e-7
              && s->data.size() == static_cast<std::size_t>(s->info.width) * s->info.height
              && bytes_sha256(
                     std::string(reinterpret_cast<const char *>(s->data.data()), s->data.size()))
                     == m.at("grid_sha256")
              && std::abs(s->info.origin.orientation.z
                          - std::sin(m.at("origin_yaw").get<double>() / 2))
                     < 1e-7
              && std::abs(s->info.origin.orientation.w
                          - std::cos(m.at("origin_yaw").get<double>() / 2))
                     < 1e-7;
        } catch (...) {
          p->map_valid = false;
        }
      });
  p->amcl = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      p->topic("amcl_pose"), durable,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr s,
             const rclcpp::MessageInfo &i) {
        if (!p->is_expected_publisher(p->topic("amcl_pose"), "amcl", i)) {
          return;
        }
        auto q = s->pose.pose.orientation;
        bool good = s->header.frame_id == "map" && std::isfinite(s->pose.pose.position.x)
                    && std::isfinite(s->pose.pose.position.y)
                    && std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) < 1e-3;
        for (auto n : {0, 7, 35}) {
          good = good && std::isfinite(s->pose.covariance[n]) && s->pose.covariance[n] >= 0
                 && s->pose.covariance[n] <= .5;
        }
        p->localization_valid = good;
      });
  p->clock = node->create_subscription<rosgraph_msgs::msg::Clock>(
      p->topic("clock"), rclcpp::QoS(1).reliable(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr s, const rclcpp::MessageInfo &i) {
        if (!p->is_expected_publisher(p->topic("clock"), "execution_bridge", i)) {
          return;
        }
        auto t = x::checked_stamp(s->clock);
        if (t <= 0) {
          if (p->source_ns) {
            p->clock_fault = true;
          }
          return;
        }
        if (t < p->source_ns) {
          p->clock_fault = true;
        }
        if (t != p->source_ns) {
          p->clock_progress_ms = p->now_ms;
        }
        p->source_ns = t;
      });
  p->gate = node->create_subscription<GateState>(
      p->config.ns + "/state", rclcpp::QoS(1).reliable(),
      [this](GateState::ConstSharedPtr s, const rclcpp::MessageInfo &i) {
        auto owner = p->node.lock();
        if (!owner
            || !x::publisher_matches(*owner, p->config.ns + "/state", "execution_gate",
                                     p->config.ns, i)) {
          return;
        }
        p->state = *s;
        p->gate_at_ms = p->now_ms;
      });
}

DeliveryMonitor::~DeliveryMonitor() = default;

void DeliveryMonitor::tick(std::int64_t now, bool ready) {
  p->now_ms = now;
  auto h = std::make_shared<DeliveryHealth>();
  h->now_ms = now;
  h->source_now_ns = p->source_ns;
  if (now - p->graph_at_ms >= 100) {
    try {
      p->graph_valid = p->check_graph_valid();
    } catch (...) {
      p->graph_valid = false;
    }
    p->graph_at_ms = now;
  }

  // Input health is provisional until the matching source-clock TF check below.
  h->reason = "INPUT_OR_GRAPH_UNQUALIFIED";
  h->healthy = ready && p->graph_valid && p->map_valid && p->localization_valid && !p->clock_fault
               && now - p->scan_at_ms < 500 && now - p->odom_at_ms < 200 && now - p->clock_progress_ms < 300
               && p->state && now - p->gate_at_ms < 100
               && (p->state->identity.owner_epoch == p->config.epoch
                   || (p->state->identity.mission_id.empty() && p->state->state == "DISARMED"
                       && !p->state->allowed));
  if (p->state) {
    h->clock_epoch = p->state->clock_epoch;
  }
  try {
    auto t = p->buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const auto source = x::checked_stamp(t.header.stamp);
    auto q = t.transform.rotation;
    double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    h->pose.pose = {t.transform.translation.x, t.transform.translation.y,
                    std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    h->pose.source_ns = source;
    h->pose.received_ms = now;
    h->pose.clock_epoch = h->clock_epoch;
    h->pose.localization_valid = p->localization_valid;
    h->pose.tf_valid = source > 0 && source <= p->source_ns && p->source_ns - source < 200000000
                       && std::isfinite(norm) && std::abs(norm - 1) < 1e-3;
    if (h->pose.tf_valid) {
      if (p->previous_pose && h->pose.clock_epoch == p->previous_pose->clock_epoch
          && h->pose.source_ns > p->previous_pose->source_ns) {
        auto &prev = *p->previous_pose;
        double dt = (h->pose.source_ns - prev.source_ns) * 1e-9;
        double dyaw = std::atan2(std::sin(h->pose.pose.yaw - prev.pose.yaw),
                                 std::cos(h->pose.pose.yaw - prev.pose.yaw));
        if (std::hypot(h->pose.pose.x - prev.pose.x, h->pose.pose.y - prev.pose.y) > .1 + .5 * dt
            || std::abs(dyaw) > .1 + 1.5 * dt) {
          p->pose_fault = true;
        }
      }
      p->previous_pose = h->pose;
    }
    h->healthy = h->healthy && h->pose.tf_valid && !p->pose_fault;
  } catch (...) {
    h->healthy = false;
    h->reason = "TF_UNAVAILABLE";
  }
  if (h->healthy) {
    h->reason = "HEALTHY";
  } else {
    h->reason = Json{{"action_ready", ready},
                     {"graph", p->graph_valid},
                     {"map", p->map_valid},
                     {"localization", p->localization_valid},
                     {"clock_fault", p->clock_fault},
                     {"pose_fault", p->pose_fault},
                     {"tf_valid", h->pose.tf_valid},
                     {"scan_age_ms", now - p->scan_at_ms},
                     {"odom_age_ms", now - p->odom_at_ms},
                     {"clock_progress_age_ms", now - p->clock_progress_ms},
                     {"gate_age_ms", now - p->gate_at_ms}}
                    .dump();
  }
  std::atomic_store(&p->current, std::shared_ptr<const DeliveryHealth>(h));
}

std::shared_ptr<const DeliveryHealth> DeliveryMonitor::snapshot() const {
  return std::atomic_load(&p->current);
}

std::int64_t DeliveryMonitor::source_time() const {
  return p->source_ns;
}
} // namespace cmr::mission
