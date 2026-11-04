// SPDX-License-Identifier: MIT
#include "cmr_mission/delivery.hpp"
#include <openssl/evp.h>
#include <memory>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <set>
#include <array>

namespace cmr::mission {
namespace {
std::string read(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("resource unavailable: " + path);
  }
  return {std::istreambuf_iterator<char>(f), {}};
}

double angle(double x) {
  return std::atan2(std::sin(x), std::cos(x));
}

void fields(const Json &j, std::initializer_list<const char *> expected) {
  if (!j.is_object() || j.size() != expected.size()) {
    throw std::runtime_error("unexpected fields");
  }
  for (auto k : expected) {
    if (!j.contains(k)) {
      throw std::runtime_error(std::string("missing field ") + k);
    }
  }
}

bool hash(const Json &v) {
  const auto s = v.get<std::string>();
  return s.size() == 64 && s.find_first_not_of("0123456789abcdef") == std::string::npos;
}

void finite(double v) {
  if (!std::isfinite(v)) {
    throw std::runtime_error("nonfinite station");
  }
}

bool pose_good(const Pose2 &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
} // namespace

Json strict_json(const std::string &text) {
  if (text.size() > 1048576) {
    throw std::runtime_error("JSON size bound");
  }
  std::vector<std::set<std::string>> keys;
  return Json::parse(text, [&](int depth, Json::parse_event_t ev, Json &parsed) {
    if (depth > 32) {
      throw std::runtime_error("JSON depth bound");
    }
    if (ev == Json::parse_event_t::object_start) {
      keys.emplace_back();
    } else if (ev == Json::parse_event_t::object_end) {
      keys.pop_back();
    } else if (ev == Json::parse_event_t::key
               && !keys.back().insert(parsed.get<std::string>()).second) {
      throw std::runtime_error("duplicate JSON key");
    }
    return true;
  });
}

std::string file_sha256(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f || !std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("resource unavailable: " + path);
  }
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> c(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!c || EVP_DigestInit_ex(c.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("digest init");
  }
  std::array<char, 16384> b{};
  while (f) {
    f.read(b.data(), b.size());
    if (f.gcount()
        && EVP_DigestUpdate(c.get(), b.data(), static_cast<std::size_t>(f.gcount())) != 1) {
      throw std::runtime_error("digest update");
    }
  }
  if (!f.eof()) {
    throw std::runtime_error("resource read failure");
  }
  unsigned char out[EVP_MAX_MD_SIZE];
  unsigned int size = 0;
  if (EVP_DigestFinal_ex(c.get(), out, &size) != 1) {
    throw std::runtime_error("digest final");
  }
  std::ostringstream s;
  for (unsigned int n = 0; n < size; ++n) {
    s << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(out[n]);
  }
  return s.str();
}

std::string bytes_sha256(const std::string &data) {
  unsigned char out[EVP_MAX_MD_SIZE];
  unsigned int count = 0;
  if (EVP_Digest(data.data(), data.size(), out, &count, EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("digest bytes");
  }
  std::ostringstream text;
  for (unsigned n = 0; n < count; ++n) {
    text << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(out[n]);
  }
  return text.str();
}

Pose2 checked_pose(const Json &j) {
  fields(j, {"frame", "position", "orientation"});
  if (j.at("frame") != "map") {
    throw std::runtime_error("station frame");
  }
  fields(j.at("position"), {"x", "y", "z"});
  fields(j.at("orientation"), {"x", "y", "z", "w"});
  auto p = j.at("position");
  auto q = j.at("orientation");
  double x = p.at("x");
  double y = p.at("y");
  double z = p.at("z");
  double qx = q.at("x");
  double qy = q.at("y");
  double qz = q.at("z");
  double qw = q.at("w");
  for (double v : {x, y, z, qx, qy, qz, qw}) {
    finite(v);
  }
  if (z != 0 || qx != 0 || qy != 0 || std::abs(qz * qz + qw * qw - 1.) > 1e-6) {
    throw std::runtime_error("station planar quaternion");
  }
  return {x, y, 2 * std::atan2(qz, qw)};
}

StationCatalog::StationCatalog(const std::string &path)
    : value(strict_json(read(path))), fingerprint(file_sha256(path)) {
  fields(value,
         {"schema_version", "map_version", "station_version", "map", "bt", "bounds", "stations"});
  if (value.at("schema_version") != 1 || !valid_identifier(value.at("map_version"))
      || !valid_identifier(value.at("station_version"))) {
    throw std::runtime_error("catalog version");
  }
  const auto &m = value.at("map");
  fields(m, {"yaml_path", "yaml_sha256", "image_path", "image_sha256", "metadata"});
  fields(value.at("bt"), {"path", "sha256"});
  if (!hash(m.at("yaml_sha256")) || !hash(m.at("image_sha256"))
      || !hash(value.at("bt").at("sha256"))) {
    throw std::runtime_error("catalog hash");
  }
  const auto &md = m.at("metadata");
  fields(md,
         {"width", "height", "resolution", "origin_x", "origin_y", "origin_yaw", "grid_sha256"});
  if (!md.at("width").is_number_unsigned() && !md.at("width").is_number_integer()) {
    throw std::runtime_error("map width");
  }
  if (!md.at("height").is_number_unsigned() && !md.at("height").is_number_integer()) {
    throw std::runtime_error("map height");
  }
  if (md.at("width") <= 0 || md.at("width") > 4096 || md.at("height") <= 0 || md.at("height") > 4096
      || !hash(md.at("grid_sha256"))) {
    throw std::runtime_error("map metadata");
  }
  for (auto k : {"resolution", "origin_x", "origin_y", "origin_yaw"}) {
    finite(md.at(k));
  }
  if (md.at("resolution").get<double>() <= 0 || md.at("resolution").get<double>() > 1) {
    throw std::runtime_error("map resolution");
  }
  const auto &b = value.at("bounds");
  fields(b, {"min_x", "max_x", "min_y", "max_y"});
  for (auto v : b) {
    finite(v.get<double>());
  }
  if (b.at("min_x").get<double>() >= b.at("max_x").get<double>()
      || b.at("min_y").get<double>() >= b.at("max_y").get<double>()) {
    throw std::runtime_error("bounds");
  }
  if (!value.at("stations").is_array() || value.at("stations").empty()
      || value.at("stations").size() > 32) {
    throw std::runtime_error("station count");
  }
  std::set<std::string> ids;
  for (const auto &s : value.at("stations")) {
    fields(s, {"station_id", "map_version", "station_version", "pose"});
    if (!valid_identifier(s.at("station_id")) || !ids.insert(s.at("station_id")).second
        || s.at("map_version") != value.at("map_version")
        || s.at("station_version") != value.at("station_version")) {
      throw std::runtime_error("station identity");
    }
    auto p = checked_pose(s.at("pose"));
    if (p.x < b.at("min_x").get<double>() || p.x > b.at("max_x").get<double>()
        || p.y < b.at("min_y").get<double>() || p.y > b.at("max_y").get<double>()) {
      throw std::runtime_error("station outside reviewed bounds");
    }
  }
  if (!resources_current()) {
    throw std::runtime_error("catalog resource mismatch");
  }
}

bool StationCatalog::resources_current() const {
  try {
    const auto &m = value.at("map");
    const auto &b = value.at("bt");
    return file_sha256(m.at("yaml_path")) == m.at("yaml_sha256")
           && file_sha256(m.at("image_path")) == m.at("image_sha256")
           && file_sha256(b.at("path")) == b.at("sha256");
  } catch (...) {
    return false;
  }
}

Json StationCatalog::freeze(const MissionRequest &r) const {
  if (validate(r) != Validation::Valid || r.map_version != value.at("map_version")
      || r.station_version != value.at("station_version")) {
    throw std::runtime_error("INVALID_STATION_VERSION");
  }
  Json out = {{"catalog_sha256", fingerprint},
              {"map_version", r.map_version},
              {"station_version", r.station_version},
              {"map", value.at("map")},
              {"bt", value.at("bt")}};
  for (auto leg : {"outbound", "return"}) {
    const auto &id = std::string(leg) == "outbound" ? r.delivery_station : r.return_station;
    bool found = false;
    for (const auto &s : value.at("stations")) {
      if (s.at("station_id") == id) {
        out[leg] = s;
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error("UNKNOWN_STATION");
    }
  }
  return out;
}

void ArrivalWindow::add(const PoseObservation &s) {
  if (!pose_good(s.pose) || s.source_ns <= 0 || !s.tf_valid || !s.localization_valid) {
    samples.clear();
    return;
  }
  if (!samples.empty()) {
    const auto &p = samples.back();
    if (s.identity != p.identity || s.clock_epoch != p.clock_epoch) {
      reset();
    } else if (s.source_ns < p.source_ns || s.received_ms < p.received_ms) {
      samples.clear();
      discontinuity = true;
    } else if (s.source_ns == p.source_ns) {
      return;
    } else {
      double dt = (s.source_ns - p.source_ns) * 1e-9;
      if (s.received_ms - p.received_ms >= 200 || dt > .2) {
        samples.clear();
      } else if (std::hypot(s.pose.x - p.pose.x, s.pose.y - p.pose.y) > .1 + .5 * dt
                 || std::abs(angle(s.pose.yaw - p.pose.yaw)) > .1 + 1.5 * dt) {
        samples.clear();
        discontinuity = true;
      }
    }
  }
  samples.push_back(s);
  while (samples.size() > 512
         || (!samples.empty() && s.source_ns - samples.front().source_ns > 4000000000LL)) {
    samples.pop_front();
  }
}

ArrivalDecision ArrivalWindow::evaluate(const Pose2 &target, const std::string &id,
                                        const std::string &clock, std::int64_t terminal,
                                        std::int64_t stop_from, std::int64_t stop_to,
                                        std::int64_t source_now, std::int64_t now) const {
  ArrivalDecision d;
  d.reason = "ARRIVAL_WINDOW_UNKNOWN";
  if (discontinuity) {
    d.reason = "LOCALIZATION_JUMP_OR_CLOCK_ROLLBACK";
    return d;
  }
  if (!pose_good(target) || samples.empty() || terminal <= 0 || stop_from < terminal
      || stop_to < stop_from) {
    return d;
  }
  // Validate the latest observation before walking the terminal/stop-aligned window.
  const auto &last = samples.back();
  if (last.identity != id || last.clock_epoch != clock || now < last.received_ms
      || now - last.received_ms >= 200 || last.source_ns > source_now
      || source_now - last.source_ns >= 200000000LL
      || std::llabs(last.source_ns - stop_to) >= 200000000LL) {
    return d;
  }
  std::int64_t previous = 0;
  for (const auto &s : samples) {
    if (s.source_ns < std::max(terminal, stop_from)
        || s.source_ns < last.source_ns - 1200000000LL) {
      continue;
    }
    if (s.identity != id || s.clock_epoch != clock || !s.tf_valid || !s.localization_valid) {
      return d;
    }
    if (previous && s.source_ns - previous >= 200000000LL) {
      return d;
    }
    previous = s.source_ns;
    double pos = std::hypot(s.pose.x - target.x, s.pose.y - target.y);
    double yaw = std::abs(angle(s.pose.yaw - target.yaw));
    if (pos > .25 || yaw > .25) {
      d.reason = "OUTSIDE_ARRIVAL_TOLERANCE";
      return d;
    }
    if (!d.samples) {
      d.from_ns = s.source_ns;
    }
    ++d.samples;
    d.to_ns = s.source_ns;
    d.position_error = pos;
    d.yaw_error = yaw;
  }
  d.arrived = d.samples >= 5 && d.to_ns - d.from_ns >= 1000000000LL;
  d.reason = d.arrived ? "ARRIVED" : "ARRIVAL_SPAN_SHORT";
  return d;
}
} // namespace cmr::mission
