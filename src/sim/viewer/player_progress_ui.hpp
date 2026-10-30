#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <raylib.h>

#include "sim/viewer/experience_manifest.hpp"
#include "sim/viewer/player_profile.hpp"

namespace lost_signal::sim::viewer {

struct ChapterCatalogEntry {
  std::string experience_id;
  std::string title;
  std::uint8_t stars = 0;
  std::uint32_t completions = 0;
};

std::vector<ChapterCatalogEntry> load_chapter_catalog(
    const PlayerProfile& profile, const ExperienceManifest& current,
    const std::string& root = "configs/experiences");

// These panels are presentation-only. They consume authored manifest text and
// the local profile; neither function has access to a ScenarioSession or any
// SIM truth. `font`, `scale`, and `zh_cn` are passed explicitly so the Viewer
// main loop does not need to expose its global text state.
void draw_player_profile_panel(const ExperienceManifest& experience,
                               const PlayerProfile* profile, bool detailed,
                               Font font, float scale, bool zh_cn);

void draw_chapter_catalog_panel(
    const std::vector<ChapterCatalogEntry>& entries,
    const std::string& current_experience_id, Font font, float scale,
    bool zh_cn);

}  // namespace lost_signal::sim::viewer
