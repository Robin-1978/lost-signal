#include "performance_score.hpp"

#include <algorithm>
#include <cmath>

namespace mission {
namespace {

double ratio_score(double target, double actual) {
 if (target <= 0.0)
 return 100.0;
 if (!std::isfinite(actual) || actual <= 0.0)
 return 0.0;
 return 100.0 * std::clamp(target / actual, 0.0, 1.0);
}

} // namespace

PerformanceScore compute_performance_score(const PerformanceScoreInput& input) {
 PerformanceScore score;
 if (!input.successful)
 return score;

 score.ranked = true;
 score.safety = input.ideal_clearance_m <= 0.0
 ? 100.0
 : 100.0 * std::clamp(input.clearance_m / input.ideal_clearance_m, 0.0, 1.0);
 score.pace = ratio_score(input.target_time_s, input.actual_time_s);
 score.efficiency = ratio_score(input.target_energy_wh, input.actual_energy_wh);

 // Total variation of the commanded wheel-edge speed exposes oscillation
 // while remaining comparable across chassis. Dividing by travelled distance
 // prevents a longer mission from being penalized merely for being longer.
 const double normalized_variation = std::max(0.0, input.command_variation_mps) /
 std::max(1.0, input.path_length_m);
 score.smoothness = 100.0 / (1.0 + normalized_variation);

 // Waiting can be the correct safe action. The first 40% of a run is therefore
 // unpenalized here; total elapsed time is already reflected by the pace axis.
 // Recovery and repeated action changes expose indecision without changing
 // whether the run is considered safe or successful.
 const double duration = std::max(1.0, input.actual_time_s);
 const double excess_wait_s = std::max(0.0, input.wait_time_s - 0.40 * duration);
 const double switch_allowance = std::max(1.0, duration / 10.0);
 const double excess_switches = std::max(0.0,
 static_cast<double>(input.strategy_switches) - switch_allowance);
 const double decisiveness_penalty =
 2.0 * excess_wait_s / duration +
 3.0 * std::max(0.0, input.recovery_time_s) / duration +
 2.0 * excess_switches / duration;
 score.decisiveness = 100.0 / (1.0 + decisiveness_penalty);

 score.overall = 0.30 * score.safety + 0.25 * score.pace +
 0.15 * score.efficiency + 0.15 * score.smoothness +
 0.15 * score.decisiveness;
 return score;
}

} // namespace mission
