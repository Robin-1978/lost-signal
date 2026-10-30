#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <raylib.h>

#include "sim/experiment/episode_pair.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/viewer/experience_flow.hpp"
#include "sim/viewer/experience_manifest.hpp"
#include "sim/viewer/frame.hpp"
#include "sim/viewer/keyframe_review.hpp"
#include "sim/viewer/player_progress_ui.hpp"

namespace lost_signal::sim::viewer {

struct VisibleRunSummary {
  bool recorded = false;
  bool timed_out = false;
  bool objectives_done = false;
  bool objectives_passed = false;
  double scan_period_s = 0.0;
  eval::EpisodeScore score;

  bool safe() const {
    return recorded && !timed_out && objectives_done && objectives_passed &&
           score.collision_steps == 0;
  }
};

struct ExperimentUiState {
  VisibleRunSummary baseline;
  VisibleRunSummary variant;
  experiment::EpisodePair episodes;
  std::vector<ReviewKeyframe> baseline_keyframes;
  std::vector<ReviewKeyframe> variant_keyframes;
  std::optional<experiment::EpisodeArtifact> standalone;
};

struct ExperienceHitRegions {
  Rectangle world_hotspot{};
  Rectangle previous{};
  Rectangle next{};
  Rectangle primary{};
  Rectangle retry{};
  Rectangle next_chapter{};
  std::vector<Rectangle> walkaround;
  std::vector<Rectangle> choices;
  std::vector<std::pair<Rectangle, std::size_t>> causal_keyframes;

  void clear() {
    world_hotspot = {};
    previous = {};
    next = {};
    primary = {};
    retry = {};
    next_chapter = {};
    walkaround.clear();
    choices.clear();
    causal_keyframes.clear();
  }
};

void draw_experience_overlay(
    const ExperienceManifest& experience, ExperiencePhase phase,
    ExperienceRunKind run_kind, ExperienceViewLayer view_layer,
    bool comparison_complete, double active_scan_period_s, int selected_choice,
    int walkaround_stop, std::size_t walkaround_visited_count,
    std::size_t required_walkaround_visited_count,
    std::size_t required_walkaround_count, bool walkaround_complete,
    const std::vector<ReviewKeyframe>& review_keyframes,
    int selected_review_keyframe, bool objectives_done, bool objectives_passed,
    const eval::EpisodeScore& score, const Frame* frame,
    const ExperimentUiState* experiment_ui, const PlayerProfile* player_profile,
    const std::vector<ChapterCatalogEntry>* chapter_catalog, bool show_catalog,
    bool timed_out, PredictionAssessment prediction_assessment,
    const experiment::ExperienceGateVerdict* gate_verdict,
    const std::string& action_error, bool actions_enabled, Font font,
    float text_scale, bool zh_cn, ExperienceHitRegions* hit_regions);

}  // namespace lost_signal::sim::viewer
