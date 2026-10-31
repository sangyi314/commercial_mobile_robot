// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace cmr::mission {

// Value objects only. No accepted Mission or external effect is created here.
struct MissionRequest {
  std::uint32_t schema_version{1};
  std::string mission_id;
  std::string delivery_station;
  std::string return_station;
  std::string map_version;
  std::string station_version;
  std::int64_t created_at_unix_ms{0};
};

enum class Validation { Valid, Schema, Identifier, CreatedAt };
enum class Admission { Invalid, InvalidContext, Existing, Conflict, Busy, CreateIntent };

// Internal snapshot, obtained under the future store's write transaction.
// The store must look up the request ID before invoking this function.
struct AdmissionContext {
  std::optional<MissionRequest> existing;
  bool active_slot_held{false};
};

inline bool valid_identifier(const std::string& value) {
  if (value.empty() || value.size() > 64) return false;
  for (const unsigned char c : value) {
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')) {
      return false;
    }
  }
  return true;
}

inline Validation validate(const MissionRequest& r) {
  if (r.schema_version != 1) return Validation::Schema;
  if (!valid_identifier(r.mission_id) || !valid_identifier(r.delivery_station) ||
      !valid_identifier(r.return_station) || !valid_identifier(r.map_version) ||
      !valid_identifier(r.station_version)) return Validation::Identifier;
  if (r.created_at_unix_ms <= 0) return Validation::CreatedAt;
  return Validation::Valid;
}

// Requires valid requests; audit timestamps are deliberately not business content.
inline bool same_business_content(const MissionRequest& a, const MissionRequest& b) {
  return a.schema_version == b.schema_version &&
         a.delivery_station == b.delivery_station &&
         a.return_station == b.return_station &&
         a.map_version == b.map_version && a.station_version == b.station_version;
}

inline Admission decide_admission(const MissionRequest& request,
                                  const AdmissionContext& context) {
  if (validate(request) != Validation::Valid) return Admission::Invalid;
  if (context.existing) {
    if (validate(*context.existing) != Validation::Valid ||
        context.existing->mission_id != request.mission_id) {
      return Admission::InvalidContext;
    }
    return same_business_content(request, *context.existing)
      ? Admission::Existing : Admission::Conflict;
  }
  return context.active_slot_held ? Admission::Busy : Admission::CreateIntent;
}

}  // namespace cmr::mission
