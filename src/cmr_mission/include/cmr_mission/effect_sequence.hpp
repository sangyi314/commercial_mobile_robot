// SPDX-License-Identifier: MIT
#pragma once
#include "mission.hpp"
#include <charconv>
#include <stdexcept>
#include <string_view>

namespace cmr::mission {
// Store-generated renew IDs are mission:committed_record_version:4.
// The Mission identifier alphabet excludes ':'. Version zero/overflow/aliases
// fail closed. This receipt never grants authority or creates a new context.
inline std::uint64_t renew_effect_sequence(const Effect &effect) {
  const std::string prefix = effect.mission + ":";
  const std::string_view text(effect.id);
  if (effect.kind != EffectKind::RenewLease || text.substr(0, prefix.size()) != prefix) {
    throw std::invalid_argument("renew effect identity");
  }
  const auto middle = text.substr(prefix.size());
  const auto colon = middle.find(':');
  if (colon == std::string_view::npos || middle.substr(colon) != ":4" || colon == 0
      || middle[0] == '0') {
    throw std::invalid_argument("renew effect format");
  }
  std::uint64_t version = 0;
  const auto part = middle.substr(0, colon);
  const auto result = std::from_chars(part.data(), part.data() + part.size(), version);
  if (result.ec != std::errc{} || result.ptr != part.data() + part.size() || version == 0) {
    throw std::invalid_argument("renew effect version");
  }
  return version;
}
} // namespace cmr::mission
