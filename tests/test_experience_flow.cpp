#include <cassert>
#include <cstdio>
#include <stdexcept>

#include "sim/viewer/experience_flow.hpp"

int main() {
 using namespace lost_signal::sim::viewer;

 bool rejected = false;
 try {
 (void)ExperienceFlow({4, 1, 100});
 } catch (const std::invalid_argument&) {
 rejected = true;
 }
 assert(rejected);

 ExperienceChoice accepted;
 accepted.evidence_rule = ExperienceEvidenceRule::kIndependentAcceptance;
 ExperienceChoice sim_rejected;
 sim_rejected.evidence_rule = ExperienceEvidenceRule::kSimRejected;
 ExperienceChoice incomplete;
 incomplete.evidence_rule = ExperienceEvidenceRule::kIncomplete;
 ExperienceEvidence evidence{true, true, true, true};
 assert(choice_supported(accepted, evidence));
 assert(!choice_supported(sim_rejected, evidence));
 assert(!choice_supported(incomplete, evidence));
 evidence.all_sim_accepted = false;
 assert(!choice_supported(accepted, evidence));
 assert(choice_supported(sim_rejected, evidence));
 evidence.all_robot_done = false;
 assert(!choice_supported(sim_rejected, evidence));
 assert(choice_supported(incomplete, evidence));

 ExperienceChoice tradeoff;
 tradeoff.evidence_rule = ExperienceEvidenceRule::kComparisonTradeoff;
 ExperienceChoice equivalent;
 equivalent.evidence_rule = ExperienceEvidenceRule::kComparisonEquivalent;
 ExperienceChoice unsafe;
 unsafe.evidence_rule = ExperienceEvidenceRule::kComparisonUnsafe;
 evidence = {};
 evidence.comparison_complete = true;
 evidence.baseline_safe = true;
 evidence.variant_safe = true;
 evidence.minimum_elapsed_increase_s = 1.0;
 evidence.minimum_clearance_loss_m = 0.05;
 evidence.equivalent_elapsed_tolerance_s = 0.5;
 evidence.equivalent_clearance_tolerance_m = 0.02;
 evidence.elapsed_delta_s = 4.76;
 evidence.clearance_delta_m = -0.156;
 assert(choice_supported(tradeoff, evidence));
 assert(!choice_supported(equivalent, evidence));
 assert(!choice_supported(unsafe, evidence));
 evidence.elapsed_delta_s = 0.2;
 evidence.clearance_delta_m = -0.01;
 assert(!choice_supported(tradeoff, evidence));
 assert(choice_supported(equivalent, evidence));
 evidence.variant_safe = false;
 assert(!choice_supported(equivalent, evidence));
 assert(choice_supported(unsafe, evidence));

 ExperienceFlow flow({4, 3, 10});
 assert(flow.phase() == ExperiencePhase::kBriefing);
 assert(flow.walkaround_index() == 0);
 assert(flow.walkaround_visited_count() == 1);
 assert(!flow.walkaround_complete());
 flow.inspect_previous();
 assert(flow.walkaround_index() == 3);
 assert(flow.walkaround_visited_count() == 2);
 flow.inspect_next();
 flow.inspect_next();
 assert(flow.walkaround_index() == 1);
 assert(flow.walkaround_visited_count() == 3);
 assert(!flow.continue_to_prediction());
 flow.inspect_next();
 assert(flow.walkaround_index() == 2);
 assert(flow.walkaround_complete());
 assert(flow.continue_to_prediction());
 assert(!flow.continue_to_prediction());
 assert(flow.phase() == ExperiencePhase::kPrediction);
 assert(!flow.select_choice(3, 5));
 assert(flow.select_choice(2, 5));
 assert(flow.selected_choice() == 2);
 assert(flow.phase() == ExperiencePhase::kRunning);
 assert(!flow.update(14, false));
 assert(flow.update(15, false));
 assert(flow.phase() == ExperiencePhase::kDebrief);
 assert(flow.timed_out());
 assert(flow.baseline_complete());
 assert(!flow.comparison_complete());
 assert(flow.run_kind() == ExperienceRunKind::kBaseline);
 assert(!flow.update(16, true));
 assert(flow.start_variant(0));
 assert(flow.phase() == ExperiencePhase::kRunning);
 assert(flow.run_kind() == ExperienceRunKind::kVariant);
 assert(flow.selected_choice() == 2);
 assert(flow.update(3, true));
 assert(flow.comparison_complete());
 assert(!flow.start_variant(0));
 assert(flow.retry_current(0));
 assert(flow.phase() == ExperiencePhase::kRunning);
 assert(flow.baseline_complete());
 assert(!flow.comparison_complete());
 assert(flow.update(4, true));
 assert(flow.comparison_complete());

 flow.reset();
 assert(flow.phase() == ExperiencePhase::kBriefing);
 assert(flow.selected_choice() == -1);
 assert(!flow.timed_out());
 assert(flow.walkaround_visited_count() == 1);
 assert(!flow.continue_to_prediction());

 // Optional detail cards stay inspectable without blocking the experiment.
 ExperienceFlow focused({4, 3, 10, {true, false, true, false}});
 assert(focused.required_walkaround_count() == 2);
 assert(focused.required_walkaround_visited_count() == 1);
 assert(!focused.walkaround_complete());
 focused.inspect(2);
 assert(focused.walkaround_index() == 2);
 assert(focused.walkaround_visited(2));
 assert(focused.required_walkaround_visited_count() == 2);
 assert(focused.walkaround_complete());
 assert(focused.continue_to_prediction());

 rejected = false;
 try {
 (void)ExperienceFlow({4, 3, 10, {true, false}});
 } catch (const std::invalid_argument&) {
 rejected = true;
 }
 assert(rejected);

 ExperienceFlow direct({0, 3, 10});
 assert(direct.walkaround_complete());
 assert(direct.continue_to_prediction());
 assert(direct.select_choice(0, 30));
 assert(direct.update(31, true));
 assert(direct.end_reason() == ExperienceEndReason::kScenarioFinished);

 // The variant uses its authored experiment budget rather than inheriting a
 // potentially different baseline chapter budget.
 ExperienceFlow variant_budget({0, 3, 10, {}, 3});
 assert(variant_budget.continue_to_prediction());
 assert(variant_budget.select_choice(0, 0));
 assert(variant_budget.update(1, true));
 assert(variant_budget.start_variant(0));
 assert(!variant_budget.update(2, false));
 assert(variant_budget.update(3, false));
 assert(variant_budget.timed_out());

 std::puts("test_experience_flow: ok");
 return 0;
}
