#include "sim/viewer/experience_flow.hpp"

#include <stdexcept>

namespace lost_signal::sim::viewer {

bool choice_supported(const ExperienceChoice& choice,
 const ExperienceEvidence& evidence) {
 switch (choice.evidence_rule) {
 case ExperienceEvidenceRule::kIndependentAcceptance:
 return evidence.terminal && evidence.all_robot_done &&
 evidence.all_sim_accepted && evidence.collision_free;
 case ExperienceEvidenceRule::kSimRejected:
 return evidence.terminal && evidence.all_robot_done &&
 (!evidence.all_sim_accepted || !evidence.collision_free);
 case ExperienceEvidenceRule::kIncomplete:
 return evidence.terminal && !evidence.all_robot_done;
 case ExperienceEvidenceRule::kStaticAnswer:
 return choice.correct;
 }
 return false;
}

ExperienceFlow::ExperienceFlow(Config config) : config_(config) {
 if (config_.choice_count < 2 || config_.choice_count > 4) {
 throw std::invalid_argument("experience flow needs two to four choices");
 }
}

void ExperienceFlow::inspect_previous() {
 if (phase_ != ExperiencePhase::kBriefing ||
 config_.walkaround_count == 0) return;
 walkaround_index_ = (walkaround_index_ + config_.walkaround_count - 1) %
 config_.walkaround_count;
}

void ExperienceFlow::inspect_next() {
 if (phase_ != ExperiencePhase::kBriefing ||
 config_.walkaround_count == 0) return;
 walkaround_index_ = (walkaround_index_ + 1) % config_.walkaround_count;
}

bool ExperienceFlow::continue_to_prediction() {
 if (phase_ != ExperiencePhase::kBriefing) return false;
 phase_ = ExperiencePhase::kPrediction;
 return true;
}

bool ExperienceFlow::select_choice(std::size_t choice_index,
 std::uint64_t episode_steps) {
 if (phase_ != ExperiencePhase::kPrediction ||
 choice_index >= config_.choice_count) return false;
 selected_choice_ = static_cast<int>(choice_index);
 running_start_step_ = episode_steps;
 phase_ = ExperiencePhase::kRunning;
 return true;
}

bool ExperienceFlow::update(std::uint64_t episode_steps,
 bool scenario_finished) {
 if (phase_ != ExperiencePhase::kRunning) return false;
 if (scenario_finished) {
 phase_ = ExperiencePhase::kDebrief;
 end_reason_ = ExperienceEndReason::kScenarioFinished;
 return true;
 }
 const std::uint64_t elapsed = episode_steps >= running_start_step_
 ? episode_steps - running_start_step_ : 0;
 if (config_.time_limit_steps > 0 && elapsed >= config_.time_limit_steps) {
 phase_ = ExperiencePhase::kDebrief;
 end_reason_ = ExperienceEndReason::kTimeLimit;
 return true;
 }
 return false;
}

void ExperienceFlow::reset() {
 phase_ = ExperiencePhase::kBriefing;
 end_reason_ = ExperienceEndReason::kNone;
 walkaround_index_ = 0;
 selected_choice_ = -1;
 running_start_step_ = 0;
}

} // namespace lost_signal::sim::viewer
