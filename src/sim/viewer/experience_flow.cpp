#include "sim/viewer/experience_flow.hpp"

#include <algorithm>
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
 case ExperienceEvidenceRule::kComparisonTradeoff:
 return evidence.comparison_complete && evidence.baseline_safe &&
 evidence.variant_safe &&
 evidence.elapsed_delta_s >= evidence.minimum_elapsed_increase_s &&
 evidence.clearance_delta_m <= -evidence.minimum_clearance_loss_m;
 case ExperienceEvidenceRule::kComparisonEquivalent:
 return evidence.comparison_complete && evidence.baseline_safe &&
 evidence.variant_safe &&
 std::abs(evidence.elapsed_delta_s) <=
 evidence.equivalent_elapsed_tolerance_s &&
 std::abs(evidence.clearance_delta_m) <=
 evidence.equivalent_clearance_tolerance_m;
 case ExperienceEvidenceRule::kComparisonUnsafe:
 return evidence.comparison_complete && evidence.baseline_safe &&
 !evidence.variant_safe;
 }
 return false;
}

ExperienceFlow::ExperienceFlow(Config config) : config_(config) {
 if (config_.choice_count < 2 || config_.choice_count > 4) {
 throw std::invalid_argument("experience flow needs two to four choices");
 }
 if (!config_.required_walkaround.empty() &&
 config_.required_walkaround.size() != config_.walkaround_count) {
 throw std::invalid_argument(
 "experience flow needs one required flag per walkaround stop");
 }
 walkaround_visited_.assign(config_.walkaround_count, false);
 walkaround_required_ = config_.required_walkaround.empty()
 ? std::vector<bool>(config_.walkaround_count, true)
 : config_.required_walkaround;
}

std::size_t ExperienceFlow::walkaround_visited_count() const {
 return static_cast<std::size_t>(std::count(
 walkaround_visited_.begin(), walkaround_visited_.end(), true));
}

bool ExperienceFlow::walkaround_complete() const {
 for (std::size_t index = 0; index < walkaround_visited_.size(); ++index) {
 if (walkaround_required_[index] && !walkaround_visited_[index]) return false;
 }
 return true;
}

std::size_t ExperienceFlow::required_walkaround_count() const {
 return static_cast<std::size_t>(std::count(
 walkaround_required_.begin(), walkaround_required_.end(), true));
}

std::size_t ExperienceFlow::required_walkaround_visited_count() const {
 std::size_t count = 0;
 for (std::size_t index = 0; index < walkaround_visited_.size(); ++index) {
 if (walkaround_required_[index] && walkaround_visited_[index]) ++count;
 }
 return count;
}

bool ExperienceFlow::walkaround_visited(std::size_t index) const {
 return index < walkaround_visited_.size() && walkaround_visited_[index];
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

void ExperienceFlow::focus(std::size_t index) {
 if (phase_ != ExperiencePhase::kBriefing ||
 index >= walkaround_visited_.size()) return;
 walkaround_index_ = index;
}

void ExperienceFlow::inspect(std::size_t index) {
 if (phase_ != ExperiencePhase::kBriefing ||
 index >= walkaround_visited_.size()) return;
 walkaround_index_ = index;
 walkaround_visited_[index] = true;
}

bool ExperienceFlow::continue_to_prediction() {
 if (phase_ != ExperiencePhase::kBriefing) return false;
 if (!walkaround_complete()) return false;
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

bool ExperienceFlow::start_variant(std::uint64_t episode_steps) {
 if (phase_ != ExperiencePhase::kDebrief || !baseline_complete_ ||
 run_kind_ != ExperienceRunKind::kBaseline) return false;
 run_kind_ = ExperienceRunKind::kVariant;
 comparison_complete_ = false;
 end_reason_ = ExperienceEndReason::kNone;
 running_start_step_ = episode_steps;
 phase_ = ExperiencePhase::kRunning;
 return true;
}

bool ExperienceFlow::retry_current(std::uint64_t episode_steps) {
 if (phase_ != ExperiencePhase::kDebrief) return false;
 if (run_kind_ == ExperienceRunKind::kBaseline) {
 baseline_complete_ = false;
 } else {
 comparison_complete_ = false;
 }
 end_reason_ = ExperienceEndReason::kNone;
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
 if (run_kind_ == ExperienceRunKind::kBaseline) baseline_complete_ = true;
 else comparison_complete_ = true;
 return true;
 }
 const std::uint64_t elapsed = episode_steps >= running_start_step_
 ? episode_steps - running_start_step_ : 0;
 const std::uint64_t active_time_limit_steps =
 run_kind_ == ExperienceRunKind::kVariant &&
 config_.variant_time_limit_steps > 0
 ? config_.variant_time_limit_steps : config_.time_limit_steps;
 if (active_time_limit_steps > 0 && elapsed >= active_time_limit_steps) {
 phase_ = ExperiencePhase::kDebrief;
 end_reason_ = ExperienceEndReason::kTimeLimit;
 if (run_kind_ == ExperienceRunKind::kBaseline) baseline_complete_ = true;
 else comparison_complete_ = true;
 return true;
 }
 return false;
}

void ExperienceFlow::reset() {
 phase_ = ExperiencePhase::kBriefing;
 end_reason_ = ExperienceEndReason::kNone;
 walkaround_index_ = 0;
 std::fill(walkaround_visited_.begin(), walkaround_visited_.end(), false);
 selected_choice_ = -1;
 running_start_step_ = 0;
 run_kind_ = ExperienceRunKind::kBaseline;
 baseline_complete_ = false;
 comparison_complete_ = false;
}

} // namespace lost_signal::sim::viewer
