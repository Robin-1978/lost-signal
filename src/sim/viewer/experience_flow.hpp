#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sim/viewer/experience_manifest.hpp"

namespace lost_signal::sim::viewer {

enum class ExperiencePhase { kBriefing, kPrediction, kRunning, kDebrief };

enum class ExperienceRunKind { kBaseline, kVariant };

enum class ExperienceEndReason {
 kNone,
 kScenarioFinished,
 kTimeLimit,
};

struct ExperienceEvidence {
 bool terminal = false;
 bool all_robot_done = false;
 bool all_sim_accepted = false;
 bool collision_free = false;
 bool comparison_complete = false;
 bool baseline_safe = false;
 bool variant_safe = false;
 double elapsed_delta_s = 0.0;
 double clearance_delta_m = 0.0;
 double minimum_elapsed_increase_s = 0.0;
 double minimum_clearance_loss_m = 0.0;
 double equivalent_elapsed_tolerance_s = 0.0;
 double equivalent_clearance_tolerance_m = 0.0;
};

// Evaluates the player's authored hypothesis from this run's structured
// outcome. Static-answer fallback keeps older manifests loadable.
bool choice_supported(const ExperienceChoice& choice,
 const ExperienceEvidence& evidence);

enum class PredictionAssessment {
 kPending,
 kSupported,
 kRefuted,
 kInsufficientEvidence,
};

// Evidence-driven questions are evaluated after the run. A prediction is
// refuted only when another declared outcome is supported; otherwise a
// completed run with no matching outcome is reported honestly as
// insufficient evidence.
PredictionAssessment assess_prediction(
 std::size_t selected_choice, const std::vector<ExperienceChoice>& choices,
 const ExperienceEvidence& evidence);

// Presentation-only state machine. It owns no World, RobotApplication, or
// truth state; callers report only terminal status and recorded step count.
// Keyboard, mouse, and future gamepad inputs therefore share identical
// transitions instead of embedding mission logic in the raylib frontend.
class ExperienceFlow {
 public:
 struct Config {
 std::size_t walkaround_count = 0;
 std::size_t choice_count = 0;
 std::uint64_t time_limit_steps = 0;
 // Empty keeps the legacy contract: every stop is required. Otherwise one
 // flag per stop lets an experience keep optional detail cards without
 // turning the opening into a forced manual.
 std::vector<bool> required_walkaround;
 // A controlled variant may declare its own observation budget. Zero falls
 // back to the chapter budget so older manifests keep their behavior.
 std::uint64_t variant_time_limit_steps = 0;
 };

 explicit ExperienceFlow(Config config);

 ExperiencePhase phase() const { return phase_; }
 ExperienceEndReason end_reason() const { return end_reason_; }
 std::size_t walkaround_index() const { return walkaround_index_; }
 std::size_t walkaround_visited_count() const;
 std::size_t required_walkaround_count() const;
 std::size_t required_walkaround_visited_count() const;
 bool walkaround_complete() const;
 bool walkaround_visited(std::size_t index) const;
 int selected_choice() const { return selected_choice_; }
 bool timed_out() const { return end_reason_ == ExperienceEndReason::kTimeLimit; }
 ExperienceRunKind run_kind() const { return run_kind_; }
 bool baseline_complete() const { return baseline_complete_; }
 bool comparison_complete() const { return comparison_complete_; }

 void inspect_previous();
 void inspect_next();
 void focus(std::size_t index);
 void inspect(std::size_t index);
 bool continue_to_prediction();
 bool select_choice(std::size_t choice_index,
 std::uint64_t episode_steps);
 // The caller first rebuilds the exact authored runtime and applies the one
 // declared experiment variable. Flow owns only the presentation lifecycle.
 bool start_variant(std::uint64_t episode_steps);
 // Re-run the current condition after the caller has rebuilt its runtime.
 // The locked prediction and completed baseline remain intact.
 bool retry_current(std::uint64_t episode_steps);

 // Returns true exactly once when this call enters debrief.
 bool update(std::uint64_t episode_steps, bool scenario_finished);
 void reset();

 private:
 Config config_;
 ExperiencePhase phase_ = ExperiencePhase::kBriefing;
 ExperienceEndReason end_reason_ = ExperienceEndReason::kNone;
 std::size_t walkaround_index_ = 0;
 std::vector<bool> walkaround_visited_;
 std::vector<bool> walkaround_required_;
 int selected_choice_ = -1;
 std::uint64_t running_start_step_ = 0;
 ExperienceRunKind run_kind_ = ExperienceRunKind::kBaseline;
 bool baseline_complete_ = false;
 bool comparison_complete_ = false;
};

} // namespace lost_signal::sim::viewer
