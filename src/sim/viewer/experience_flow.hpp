#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sim/viewer/experience_manifest.hpp"

namespace lost_signal::sim::viewer {

enum class ExperiencePhase { kBriefing, kPrediction, kRunning, kDebrief };

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
};

// Evaluates the player's authored hypothesis from this run's structured
// outcome. Static-answer fallback keeps older manifests loadable.
bool choice_supported(const ExperienceChoice& choice,
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
 };

 explicit ExperienceFlow(Config config);

 ExperiencePhase phase() const { return phase_; }
 ExperienceEndReason end_reason() const { return end_reason_; }
 std::size_t walkaround_index() const { return walkaround_index_; }
 std::size_t walkaround_visited_count() const;
 bool walkaround_complete() const;
 int selected_choice() const { return selected_choice_; }
 bool timed_out() const { return end_reason_ == ExperienceEndReason::kTimeLimit; }

 void inspect_previous();
 void inspect_next();
 bool continue_to_prediction();
 bool select_choice(std::size_t choice_index,
 std::uint64_t episode_steps);

 // Returns true exactly once when this call enters debrief.
 bool update(std::uint64_t episode_steps, bool scenario_finished);
 void reset();

 private:
 Config config_;
 ExperiencePhase phase_ = ExperiencePhase::kBriefing;
 ExperienceEndReason end_reason_ = ExperienceEndReason::kNone;
 std::size_t walkaround_index_ = 0;
 std::vector<bool> walkaround_visited_;
 int selected_choice_ = -1;
 std::uint64_t running_start_step_ = 0;
};

} // namespace lost_signal::sim::viewer
