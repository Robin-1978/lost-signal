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

 ExperienceFlow flow({4, 3, 10});
 assert(flow.phase() == ExperiencePhase::kBriefing);
 assert(flow.walkaround_index() == 0);
 flow.inspect_previous();
 assert(flow.walkaround_index() == 3);
 flow.inspect_next();
 flow.inspect_next();
 assert(flow.walkaround_index() == 1);
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
 assert(!flow.update(16, true));

 flow.reset();
 assert(flow.phase() == ExperiencePhase::kBriefing);
 assert(flow.selected_choice() == -1);
 assert(!flow.timed_out());
 assert(flow.continue_to_prediction());
 assert(flow.select_choice(0, 30));
 assert(flow.update(31, true));
 assert(flow.end_reason() == ExperienceEndReason::kScenarioFinished);

 std::puts("test_experience_flow: ok");
 return 0;
}
