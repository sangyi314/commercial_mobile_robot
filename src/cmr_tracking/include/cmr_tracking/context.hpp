#pragma once

#include <cstdint>
#include <string>

namespace cmr::tracking {

inline constexpr char schema_version[] = "cmr.p2/1.1";

enum class Health : std::uint8_t { ok = 0, degraded = 1, invalid = 2 };

struct P2Meta {
  std::string schema_version{tracking::schema_version};
  std::string context_id;
  std::string clock_epoch;
  std::string odom_epoch;
  std::string frame_id{"odom"};
  std::string source_level;
  std::string execution_mode;
  std::string source_id;
  std::string producer_session;
  std::uint64_t sequence{0};
  std::string map_version{"UNAVAILABLE"};
  std::string localization_version{"UNAVAILABLE"};
  std::string config_hash;
  std::string model_version;
};

struct P2Context {
  std::string schema_version{tracking::schema_version};
  std::string context_id;
  std::string authority_session;
  std::string mode;
  std::string run_id;
  std::string gate_epoch;
  std::string clock_epoch;
  std::string odom_epoch;
  std::string localization_version{"UNAVAILABLE"};
  std::string map_version{"UNAVAILABLE"};
  std::string config_hash;
  std::uint64_t sequence{0};
  Health health{Health::invalid};
  std::string reason{"NO_OBSERVATION"};
};

}  // namespace cmr::tracking
