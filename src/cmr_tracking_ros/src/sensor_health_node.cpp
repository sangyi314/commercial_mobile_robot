#include "cmr_tracking_ros/input_adapter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>

#include <cmr_execution/ros_convert.hpp>
#include <cmr_interfaces/msg/gate_state.hpp>
#include <cmr_tracking/time_contract.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/utils.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace cmr::tracking_ros {
namespace {

using namespace std::chrono_literals;
using tracking::Nanoseconds;
using tracking::StampStatus;
using tracking::Health;
using cmr::execution::checked_stamp;
using cmr::execution::publisher_matches;
using Clock = rosgraph_msgs::msg::Clock;
using Gate = cmr_interfaces::msg::GateState;
using Context = cmr_interfaces::msg::P2Context;
using Odom = nav_msgs::msg::Odometry;
using Imu = sensor_msgs::msg::Imu;
using Scan = sensor_msgs::msg::LaserScan;
using Tf = tf2_msgs::msg::TFMessage;
using Transform = geometry_msgs::msg::TransformStamped;
using Gid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

constexpr std::array<const char *, 3> topics{"/scan", "/odom", "/imu"};
constexpr std::array<Nanoseconds, 3> ages{
  tracking::scan_max_age_ns, tracking::odom_max_age_ns, tracking::imu_max_age_ns};

enum Source : std::size_t { scan, odom, imu };

std::string uuid()
{
  std::ifstream file("/proc/sys/kernel/random/uuid");
  std::string value;
  if (!(file >> value) || value.size() != 36) {
    throw std::runtime_error("UUID unavailable");
  }
  return value;
}

Nanoseconds steady_now()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

Gid gid_of(const rclcpp::MessageInfo & info)
{
  Gid result;
  std::copy_n(info.get_rmw_message_info().publisher_gid.data, result.size(), result.begin());
  return result;
}

const char * reason(StampStatus status)
{
  switch (status) {
    case StampStatus::accepted: return "NONE";
    case StampStatus::stale: return "INPUT_STALE";
    case StampStatus::future:
    case StampStatus::pending: return "FUTURE_STAMP";
    case StampStatus::duplicate: return "DUPLICATE_STAMP";
    case StampStatus::out_of_order: return "OUT_OF_ORDER";
    case StampStatus::different_epoch:
    case StampStatus::clock_reset: return "CLOCK_RESET";
    case StampStatus::invalid: return "SCHEMA_INVALID";
    case StampStatus::uninitialized: return "NO_OBSERVATION";
  }
  return "SCHEMA_INVALID";
}

struct Pending {
  Nanoseconds stamp;
  Nanoseconds received_ros;
  Nanoseconds received_steady;
  std::string epoch;
  const char * waiting_for{nullptr};
  // A null return requests TF and can be retried within the original deadline.
  std::function<const char * ()> validate_and_forward;
};

struct Input {
  std::deque<Pending> pending;
  std::optional<Nanoseconds> last;
  std::optional<Gid> gid;
  std::string reason{"NO_OBSERVATION"};
};

struct Owner {
  Gid gid;
  std::string node;
  std::string name_space;
};

class SensorHealth final : public rclcpp::Node {
public:
  SensorHealth()
  : Node("sensor_health", rclcpp::NodeOptions().use_clock_thread(false)),
    tf_(std::make_unique<tf2::BufferCore>()),
    run_(declare_parameter<std::string>("run_id", "")),
    attached_(declare_parameter<std::string>("authority", "") == "ATTACHED_GATE"),
    fault_test_(declare_parameter<std::string>("fault_test", "")),
    source_node_(declare_parameter<std::string>("source_node", "")),
    source_namespace_(declare_parameter<std::string>("source_namespace", "")),
    p2_namespace_("/cmr_p2/" + run_),
    gate_topic_("/cmr_execution/" + run_ + "/state"),
    model_{declare_parameter("wheel_vx_variance", 0.0),
      declare_parameter("imu_wz_variance", 0.0), declare_parameter("imu_wz_bias", 0.0)}
  {
    if (run_ == "RUN_ID" || !std::regex_match(run_, std::regex("[A-Za-z0-9_]{1,71}")) ||
      get_namespace() != p2_namespace_ ||
      get_parameter("authority").as_string() != (attached_ ? "ATTACHED_GATE" : "OFFLINE_LOCAL") ||
      !get_parameter("use_sim_time").as_bool() || source_node_.empty() ||
      source_namespace_.empty() || source_namespace_[0] != '/' ||
      (attached_ && fault_test_.empty() && (source_node_ != "execution_bridge" ||
      source_namespace_ != "/cmr_execution/" + run_)) ||
      (!fault_test_.empty() && (!attached_ ||
      (fault_test_ != "P201.TFOwnership" && fault_test_ != "P201.FaultTime") ||
      source_node_ != "p2_fault_proxy" || source_namespace_ != p2_namespace_)) ||
      !std::isfinite(model_.wheel_vx_variance) || model_.wheel_vx_variance <= 0 ||
      !std::isfinite(model_.imu_wz_variance) || model_.imu_wz_variance <= 0 ||
      !std::isfinite(model_.imu_wz_bias))
    {
      throw std::runtime_error("invalid P2 run, source identity or covariance model");
    }
    for (std::size_t i = 0; i < topics.size(); ++i) {
      resolved_topics_[i] = get_node_topics_interface()->resolve_topic_name(topics[i]);
    }
    clock_topic_ = get_node_topics_interface()->resolve_topic_name("/clock");
    tf_topic_ = get_node_topics_interface()->resolve_topic_name("/tf");
    static_topic_ = get_node_topics_interface()->resolve_topic_name("/tf_static");
    context_.config_hash = declare_parameter<std::string>("config_hash", "");
    if (!std::regex_match(context_.config_hash, std::regex("[a-f0-9]{64}"))) {
      throw std::runtime_error("config_hash must be SHA-256");
    }
    source_level_ = declare_parameter<std::string>("source_level", "");
    execution_mode_ = declare_parameter<std::string>("execution_mode", "");
    source_id_ = declare_parameter<std::string>("source_id", "");
    model_version_ = declare_parameter<std::string>("model_version", "");
    context_.map_version = declare_parameter<std::string>("map_version", "UNAVAILABLE");
    if (source_id_.empty() || source_id_.size() > 256 || model_version_.empty() ||
      model_version_.size() > 64 || context_.map_version.size() > 128 ||
      (attached_ && (source_level_ != "E2_SIM_SENSOR" || execution_mode_ != "SIM_SENSOR")))
    {
      throw std::runtime_error("invalid P2 source metadata");
    }
    context_.mode = attached_ ? "ATTACHED_GATE" : "OFFLINE_LOCAL";
    context_.run_id = run_;
    context_.authority_session = uuid();
    context_.context_id = uuid();
    context_.odom_epoch = uuid();
    context_.clock_epoch = attached_ ? "UNAVAILABLE" : context_.authority_session + "/1";
    const auto trace = declare_parameter<std::string>("trace_path", "");
    if (!trace.empty()) {
      if (std::filesystem::exists(trace)) {
        throw std::runtime_error("P2 timestamp trace already exists");
      }
      trace_.open(trace);
      if (!trace_) {
        throw std::runtime_error("P2 timestamp trace unavailable");
      }
      trace_ << "topic,stamp_ns,receive_ros_ns,receive_steady_ns,clock_epoch,reason\n";
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto control_qos = rclcpp::QoS(1).reliable().transient_local();
    context_pub_ = create_publisher<Context>("context", control_qos);
    wheel_pub_ = create_publisher<Odom>("wheel_odom", sensor_qos);
    imu_pub_ = create_publisher<Imu>("imu_validated", sensor_qos);
    filtered_pub_ = create_publisher<Odom>("odometry/filtered", rclcpp::QoS(5));
    filter_clock_pub_ = create_publisher<Clock>("filter_clock", sensor_qos);
    reset_client_ = create_client<std_srvs::srv::Empty>("reset");
    clock_sub_ = create_subscription<Clock>(
      "/clock", sensor_qos, [this](const Clock & msg, const rclcpp::MessageInfo & info) {
        receive_clock(msg, info);
      });
    if (attached_) {
      gate_sub_ = create_subscription<Gate>(
        gate_topic_, rclcpp::QoS(5), [this](const Gate & msg, const rclcpp::MessageInfo & info) {
          receive_gate(msg, info);
        });
    }
    tf_sub_ = create_subscription<Tf>(
      "/tf", rclcpp::QoS(100).best_effort(),
      [this](const Tf & msg, const rclcpp::MessageInfo & info) {receive_tf(msg, tf_topic_, info, false);});
    static_sub_ = create_subscription<Tf>(
      "/tf_static", rclcpp::QoS(10).reliable().transient_local(),
      [this](const Tf & msg, const rclcpp::MessageInfo & info) {
        receive_tf(msg, static_topic_, info, true);
      });
    scan_sub_ = create_subscription<Scan>(
      "/scan", sensor_qos, [this](Scan::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        const auto stamp = checked_stamp(msg->header.stamp);
        receive(scan, stamp, info, [this, msg, stamp]() -> const char * {
          if (!valid_scan(*msg)) {
            return "SCHEMA_INVALID";
          }
          if (lookup("odom", "base_scan", stamp) == std::nullopt) {
            return nullptr;
          }
          return "NONE";
        });
      });
    odom_sub_ = create_subscription<Odom>(
      "/odom", sensor_qos, [this](Odom::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        receive(odom, checked_stamp(msg->header.stamp), info, [this, msg]() -> const char * {
          if (msg->header.frame_id != "odom" || msg->child_frame_id != "base_footprint") {
            return "FRAME_MISMATCH";
          }
          const auto converted = wheel_odom(*msg, model_);
          if (!converted) {
            return "INVALID_COVARIANCE";
          }
          wheel_pub_->publish(*converted);
          return "NONE";
        });
      });
    imu_sub_ = create_subscription<Imu>(
      "/imu", sensor_qos, [this](Imu::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        const auto stamp = checked_stamp(msg->header.stamp);
        receive(imu, stamp, info, [this, msg, stamp]() -> const char * {
          if (msg->header.frame_id != "imu_link") {
            return "FRAME_MISMATCH";
          }
          const auto t = lookup("base_footprint", "imu_link", stamp);
          if (!t) {
            return nullptr;
          }
          const auto converted = angular_imu(*msg, model_, t->transform.rotation);
          if (!converted) {
            return "INVALID_COVARIANCE";
          }
          imu_pub_->publish(*converted);
          return "NONE";
        });
      });
    filtered_sub_ = create_subscription<Odom>(
      "filter_output_internal", sensor_qos,
      [this](const Odom & msg, const rclcpp::MessageInfo & info) {
        const auto stamp = checked_stamp(msg.header.stamp);
        if (!blocked_ && health_reason(steady_now()) == "NONE" &&
          msg.header.frame_id == "odom" && msg.child_frame_id == "base_footprint" &&
          stamp > last_filtered_ && stamp <= last_clock_ &&
          last_clock_ - stamp <= tracking::odom_max_age_ns &&
          publisher_matches(*this, p2_namespace_ + "/filter_output_internal",
          "ekf_filter_node", p2_namespace_, info))
        {
          last_filtered_ = stamp;
          filtered_pub_->publish(msg);
        }
      });
    timer_ = create_wall_timer(5ms, [this]() {tick();});
  }

private:
  bool clock_ok(Nanoseconds steady) const
  {
    return last_clock_ > 0 && steady - last_advance_ < tracking::steady_timeout_ns;
  }

  bool authority_ok(Nanoseconds steady) const
  {
    return !attached_ ||
           (last_gate_ > 0 && steady - last_gate_ < tracking::steady_timeout_ns &&
           context_.clock_epoch != "UNAVAILABLE");
  }

  void barrier(const char * why)
  {
    if (blocked_) {
      return;
    }
    blocked_ = true;
    block_reason_ = why;
    for (auto & source : inputs_) {
      source.pending.clear();
      source.last.reset();
    }
    recovery_.reset();
    tf_->clear();
    context_.odom_epoch = uuid();
    context_.localization_version = "UNAVAILABLE";
    if (!attached_ && block_reason_ == "CLOCK_RESET") {
      context_.clock_epoch = context_.authority_session + "/2";
    }
    RCLCPP_ERROR(get_logger(), "Epoch barrier %s. A new source session and P2 launch are required.", why);
    publish_context(steady_now(), true);
  }

  void receive_clock(const Clock & msg, const rclcpp::MessageInfo & info)
  {
    if (blocked_ || !publisher_matches(*this, clock_topic_, source_node_, source_namespace_, info)) {
      return;
    }
    const auto gid = gid_of(info);
    const auto time = checked_stamp(msg.clock);
    if (time < last_clock_ || (clock_gid_ && *clock_gid_ != gid)) {
      barrier("CLOCK_RESET");
      return;
    }
    clock_gid_ = gid;
    if (time > last_clock_ && time > 0) {
      last_advance_ = steady_now();
      last_clock_ = time;
      if (authority_ok(last_advance_)) {
        filter_clock_pub_->publish(msg);
        // A clock advance can release an already queued measurement. Do the
        // same bounded admission here in case the wall timer is delayed.
        for (const auto type : {odom, imu, scan}) {
          const auto & pending = inputs_[type].pending;
          if (!pending.empty() && pending.front().stamp <= time) {
            process(type, last_advance_);
          }
        }
      }
    }
  }

  void receive_gate(const Gate & msg, const rclcpp::MessageInfo & info)
  {
    if (!publisher_matches(
        *this, gate_topic_, "execution_gate", "/cmr_execution/" + run_, info))
    {
      return;
    }
    // Gate publishes its source clock before the first business authorization.
    // Its ExecutionIdentity is empty in START_DISARMED; do not issue a lease just
    // to populate identity. Authenticate the run-scoped publisher and mirror the
    // clock verbatim, retaining an explicit unavailable gate identity meanwhile.
    const bool unbound = msg.state == "DISARMED" && !msg.allowed &&
      msg.identity.run_id.empty() && msg.identity.gate_epoch.empty() &&
      msg.identity.owner_epoch.empty() && msg.identity.mission_id.empty() &&
      msg.identity.operation_id.empty() && msg.identity.attempt_id.empty();
    if (msg.synthetic || (!unbound && (msg.identity.run_id != run_ ||
      msg.identity.gate_epoch.empty())) || msg.identity.gate_epoch.size() > 128 ||
      msg.clock_epoch.empty() || msg.clock_epoch.size() > 128 || msg.ros_ns <= 0)
    {
      return;
    }
    const auto gate_epoch = unbound ? std::string("UNAVAILABLE") : msg.identity.gate_epoch;
    const auto gid = gid_of(info);
    if (blocked_) {
      // Keep mirroring a genuine Gate clock transition even when this P2
      // session is irreversibly blocked. This does not relabel queued data.
      if (block_reason_ == "CLOCK_RESET" && gate_gid_ && *gate_gid_ == gid &&
        (context_.clock_epoch != msg.clock_epoch || context_.gate_epoch != gate_epoch))
      {
        context_.clock_epoch = msg.clock_epoch;
        context_.gate_epoch = gate_epoch;
        publish_context(steady_now(), true);
      }
      return;
    }
    if (gate_gid_ && (*gate_gid_ != gid ||
      (context_.gate_epoch != "UNAVAILABLE" && context_.gate_epoch != gate_epoch) ||
      context_.clock_epoch != msg.clock_epoch))
    {
      context_.clock_epoch = msg.clock_epoch;
      context_.gate_epoch = gate_epoch;
      barrier("CLOCK_RESET");
      return;
    }
    gate_gid_ = gid;
    last_gate_ = steady_now();
    context_.clock_epoch = msg.clock_epoch;
    context_.gate_epoch = gate_epoch;
  }

  void receive_tf(const Tf & msg, const std::string & topic,
    const rclcpp::MessageInfo & info, bool is_static)
  {
    if (blocked_) {
      return;
    }
    const auto gid = gid_of(info);
    const auto endpoints = get_publishers_info_by_topic(topic);
    const auto endpoint = std::find_if(endpoints.begin(), endpoints.end(),
      [&gid](const auto & p) {return p.endpoint_gid() == gid;});
    if (endpoint == endpoints.end()) {
      return;
    }
    for (const auto & t : msg.transforms) {
      const std::pair edge(t.header.frame_id, t.child_frame_id);
      const auto found = edges_.find(edge);
      if (found != edges_.end() && found->second.gid != gid) {
        RCLCPP_ERROR(get_logger(), "Duplicate TF owner for %s -> %s",
          edge.first.c_str(), edge.second.c_str());
        barrier("TF_UNAVAILABLE");
        return;
      }
      if (found == edges_.end()) {
        if (edges_.size() == 128) {
          barrier("CAPACITY");
          return;
        }
        edges_.emplace(edge, Owner{gid, endpoint->node_name(), endpoint->node_namespace()});
        RCLCPP_INFO(get_logger(), "TF_OWNER %s",
          nlohmann::json{{"parent", edge.first}, {"child", edge.second},
            {"topic", topic}, {"node", endpoint->node_name()},
            {"namespace", endpoint->node_namespace()}, {"gid", gid}}.dump().c_str());
      }
      if (!tf_->setTransform(t, endpoint->node_name(), is_static)) {
        continue;
      }
      if (edge.first == "map" && edge.second == "odom") {
        evaluate_localization(t);
      }
    }
  }

  void evaluate_localization(const Transform & next)
  {
    const auto stamp = checked_stamp(next.header.stamp);
    if (stamp < 0 || (map_odom_ && stamp <= checked_stamp(map_odom_->header.stamp))) {
      return;
    }
    if (!map_odom_) {
      context_.localization_version = uuid();
    } else {
      const auto & a = map_odom_->transform;
      const auto & b = next.transform;
      const tf2::Quaternion qa(a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w);
      const tf2::Quaternion qb(b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w);
      const double yaw_delta = std::remainder(tf2::getYaw(qb) - tf2::getYaw(qa), 2.0 * std::acos(-1.0));
      if (stamp - checked_stamp(map_odom_->header.stamp) > 500'000'000 ||
        std::hypot(b.translation.x - a.translation.x, b.translation.y - a.translation.y) > 0.30 ||
        std::abs(yaw_delta) > 0.20)
      {
        context_.localization_version = uuid();
        reset_recovery();
        localization_recovering_ = true;
      }
    }
    map_odom_ = next;
  }

  std::optional<Transform> lookup(const std::string & target, const std::string & source,
    Nanoseconds stamp)
  {
    if (stamp <= 0) {
      return std::nullopt;
    }
    try {
      return tf_->lookupTransform(target, source, tf2::TimePoint(std::chrono::nanoseconds(stamp)));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void receive(Source type, Nanoseconds stamp, const rclcpp::MessageInfo & info,
    std::function<const char * ()> action)
  {
    const auto steady = steady_now();
    if (blocked_ || !clock_ok(steady) || !authority_ok(steady)) {
      return;
    }
    if (!publisher_matches(*this, resolved_topics_[type], source_node_, source_namespace_, info)) {
      fail(type, "PRODUCER_LOST");
      return;
    }
    auto & input = inputs_[type];
    const auto gid = gid_of(info);
    if (input.gid && *input.gid != gid) {
      barrier("PRODUCER_LOST");
      return;
    }
    if (!input.gid) {
      RCLCPP_INFO(get_logger(), "P2_INPUT_SOURCE %s",
        nlohmann::json{{"topic", resolved_topics_[type]}, {"node", source_node_},
          {"namespace", source_namespace_}, {"publisher_gid", gid},
          {"source_id", source_id_}, {"config_hash", context_.config_hash},
          {"model_version", model_version_}}.dump().c_str());
    }
    input.gid = gid;
    Pending sample{stamp, last_clock_, steady, context_.clock_epoch, nullptr, std::move(action)};
    const auto arrival_state = tracking::validate_stamp({
      stamp, last_clock_, sample.epoch, context_.clock_epoch, input.last, {}, true}, ages[type]);
    if (arrival_state != StampStatus::accepted && arrival_state != StampStatus::pending) {
      fail(type, reason(arrival_state));
      record(type, sample, reason(arrival_state));
      return;
    }
    if (arrival_state == StampStatus::pending) {
      sample.waiting_for = "FUTURE_STAMP";
    }
    if (input.pending.size() == 5) {
      fail(type, "CAPACITY");
      record(type, input.pending.front(), "CAPACITY");
      input.pending.pop_front();
    }
    input.pending.push_back(std::move(sample));
    process(type, steady);
  }

  void process(Source type, Nanoseconds steady)
  {
    auto & input = inputs_[type];
    while (!input.pending.empty()) {
      auto & pending = input.pending.front();
      const auto state = tracking::validate_stamp({
        pending.stamp, last_clock_, pending.epoch, context_.clock_epoch,
        input.last, {}, true}, ages[type]);
      const bool waiting = steady - pending.received_steady < tracking::future_tolerance_ns;
      const char * outcome = nullptr;
      if (!waiting && pending.waiting_for) {
        outcome = pending.waiting_for;
      } else if (state == StampStatus::pending && waiting) {
        pending.waiting_for = "FUTURE_STAMP";
        return;
      } else if (state == StampStatus::accepted) {
        outcome = pending.validate_and_forward();
        if (!outcome && waiting) {
          pending.waiting_for = "TF_UNAVAILABLE";
          return;
        }
        if (!outcome) {
          outcome = "TF_UNAVAILABLE";
        }
      } else {
        outcome = reason(state);
      }
      const std::string_view result(outcome);
      record(type, pending, result);
      if (result == "NONE") {
        input.last = pending.stamp;
        input.reason = outcome;
        if (type == scan && fresh(odom) && fresh(imu)) {
          if (!recovery_wall_start_) {
            recovery_wall_start_ = steady;
          }
          recovery_.accept_scan(pending.stamp);
        }
      } else {
        fail(type, outcome);
      }
      input.pending.pop_front();
    }
  }

  bool fresh(Source type) const
  {
    const auto & input = inputs_[type];
    return input.reason == "NONE" && input.last && *input.last <= last_clock_ &&
           last_clock_ - *input.last <= ages[type];
  }

  void reset_recovery()
  {
    recovery_.reset();
    recovery_wall_start_.reset();
  }

  void fail(Source type, const char * error)
  {
    inputs_[type].reason = error;
    reset_recovery();
  }

  void record(Source type, const Pending & sample, std::string_view result)
  {
    if (trace_.is_open()) {
      trace_ << topics[type] << ',' << sample.stamp << ',' << sample.received_ros << ','
             << sample.received_steady << ',' << sample.epoch << ',' << result << '\n';
      if (!trace_) {
        throw std::runtime_error("P2 timestamp trace write failed");
      }
    }
  }

  std::string health_reason(Nanoseconds steady) const
  {
    if (blocked_) {
      return block_reason_;
    }
    if (!authority_ok(steady)) {
      return "PRODUCER_LOST";
    }
    if (!clock_ok(steady)) {
      return last_clock_ <= 0 ? "NO_OBSERVATION" : "INPUT_STALE";
    }
    for (const auto type : {imu, odom, scan}) {
      if (!fresh(type)) {
        const auto & input = inputs_[type];
        return input.reason == "NONE" ? "INPUT_STALE" : input.reason;
      }
    }
    if (!recovery_.ready(last_clock_) || !recovery_wall_start_ ||
      steady - *recovery_wall_start_ < tracking::recovery_window_ns)
    {
      return localization_recovering_ ? "LOCALIZATION_JUMP" : "NO_OBSERVATION";
    }
    return "NONE";
  }

  void publish_context(Nanoseconds steady, bool force)
  {
    const auto next = health_reason(steady);
    if (!force && next == context_.reason && steady - context_published_ < 100'000'000) {
      return;
    }
    if (context_.sequence == std::numeric_limits<std::uint64_t>::max()) {
      throw std::runtime_error("P2 sequence exhausted");
    }
    const auto old = context_.reason;
    context_.health = next == "NONE" ? Health::ok : Health::invalid;
    context_.reason = next;
    if (next == "NONE") {
      localization_recovering_ = false;
    }
    ++context_.sequence;
    context_pub_->publish(to_message(context_));
    context_published_ = steady;
    if (next != old) {
      RCLCPP_INFO(get_logger(), "P2_CONTEXT %s", next.c_str());
      RCLCPP_INFO(get_logger(), "P2_META %s",
        nlohmann::json{{"schema_version", context_.schema_version},
          {"context_id", context_.context_id}, {"clock_epoch", context_.clock_epoch},
          {"odom_epoch", context_.odom_epoch}, {"frame_id", "odom"},
          {"source_level", source_level_}, {"execution_mode", execution_mode_},
          {"source_id", source_id_}, {"producer_session", context_.authority_session},
          {"sequence", context_.sequence}, {"map_version", context_.map_version},
          {"localization_version", context_.localization_version},
          {"config_hash", context_.config_hash}, {"model_version", model_version_},
          {"health", static_cast<int>(context_.health)}, {"reason", next}}.dump().c_str());
    }
  }

  void qualify_graph()
  {
    if (get_publishers_info_by_topic(p2_namespace_ + "/context").size() > 1 ||
      get_publishers_info_by_topic(clock_topic_).size() > 1 ||
      (attached_ && get_publishers_info_by_topic(gate_topic_).size() > 1))
    {
      barrier("PRODUCER_LOST");
      return;
    }
    for (const auto type : {scan, odom, imu}) {
      const auto endpoints = get_publishers_info_by_topic(resolved_topics_[type]);
      if (endpoints.size() > 1 ||
        (inputs_[type].gid && endpoints.size() == 1 &&
        *inputs_[type].gid != endpoints.front().endpoint_gid()))
      {
        barrier("PRODUCER_LOST");
        return;
      }
    }
  }

  void tick()
  {
    const auto steady = steady_now();
    if (steady - graph_checked_ >= 100'000'000) {
      qualify_graph();
      graph_checked_ = steady;
      if (trace_.is_open()) {
        trace_.flush();
      }
    }
    if (blocked_) {
      scan_sub_.reset();
      odom_sub_.reset();
      imu_sub_.reset();
      tf_sub_.reset();
      static_sub_.reset();
      if (!reset_sent_ && reset_client_->service_is_ready()) {
        reset_sent_ = true;
        reset_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
      }
    } else if (clock_ok(steady) && authority_ok(steady)) {
      for (const auto type : {odom, imu, scan}) {
        process(type, steady);
      }
    }
    if (blocked_ || !clock_ok(steady) || !authority_ok(steady) ||
      !fresh(scan) || !fresh(odom) || !fresh(imu))
    {
      reset_recovery();
    }
    publish_context(steady, false);
  }

  std::unique_ptr<tf2::BufferCore> tf_;
  std::string run_;
  bool attached_;
  std::string fault_test_;
  std::array<std::string, 3> resolved_topics_;
  std::string clock_topic_, tf_topic_, static_topic_;
  std::string source_node_, source_namespace_, p2_namespace_, gate_topic_;
  std::string source_level_, execution_mode_, source_id_, model_version_;
  WheelImuModel model_;
  tracking::P2Context context_;
  tracking::ScanRecovery recovery_;
  std::optional<Nanoseconds> recovery_wall_start_;
  std::array<Input, 3> inputs_;
  std::map<std::pair<std::string, std::string>, Owner> edges_;
  std::optional<Gid> clock_gid_, gate_gid_;
  std::optional<Transform> map_odom_;
  std::ofstream trace_;
  Nanoseconds last_clock_{0}, last_advance_{0}, last_gate_{0}, last_filtered_{-1};
  Nanoseconds graph_checked_{0}, context_published_{0};
  bool blocked_{false}, reset_sent_{false}, localization_recovering_{false};
  std::string block_reason_;
  rclcpp::Publisher<Context>::SharedPtr context_pub_;
  rclcpp::Publisher<Odom>::SharedPtr wheel_pub_, filtered_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<Clock>::SharedPtr filter_clock_pub_;
  rclcpp::Subscription<Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<Gate>::SharedPtr gate_sub_;
  rclcpp::Subscription<Tf>::SharedPtr tf_sub_, static_sub_;
  rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_, filtered_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace cmr::tracking_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // The sole executor serializes clock, input admission, reset and publication.
    rclcpp::spin(std::make_shared<cmr::tracking_ros::SensorHealth>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("sensor_health"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
}
