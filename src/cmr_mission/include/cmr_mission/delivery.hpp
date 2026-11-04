// SPDX-License-Identifier: MIT
#pragma once
#include "admission.hpp"
#include <nlohmann/json.hpp>
#include <deque>

namespace cmr::mission {
using Json = nlohmann::json;
Json strict_json(const std::string &);
std::string file_sha256(const std::string &);
std::string bytes_sha256(const std::string &);

struct Pose2 {
  double x{};
  double y{};
  double yaw{};
};

Pose2 checked_pose(const Json &);

class StationCatalog {
  Json value;
  std::string fingerprint;

public:
  explicit StationCatalog(const std::string &path);

  const Json &description() const {
    return value;
  }

  const std::string &sha256() const {
    return fingerprint;
  }

  bool resources_current() const;
  Json freeze(const MissionRequest &) const;
};

struct PoseObservation {
  Pose2 pose;
  std::int64_t source_ns{};
  std::int64_t received_ms{};
  std::string identity;
  std::string clock_epoch;
  bool tf_valid{false};
  bool localization_valid{false};
};

struct ArrivalDecision {
  bool arrived{false};
  std::string reason;
  double position_error{};
  double yaw_error{};
  std::int64_t from_ns{};
  std::int64_t to_ns{};
  std::size_t samples{};
};

class ArrivalWindow {
  std::deque<PoseObservation> samples;
  bool discontinuity{false};

public:
  void reset() {
    samples.clear();
    discontinuity = false;
  }

  void add(const PoseObservation &);
  ArrivalDecision evaluate(const Pose2 &, const std::string &identity,
                           const std::string &clock_epoch, std::int64_t terminal_ns,
                           std::int64_t stop_from_ns, std::int64_t stop_to_ns,
                           std::int64_t source_now_ns, std::int64_t now_ms) const;

  bool jumped() const {
    return discontinuity;
  }
};
} // namespace cmr::mission
