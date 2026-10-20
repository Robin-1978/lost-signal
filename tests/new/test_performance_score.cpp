#include "sim/game/mission/performance_score.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 using mission::PerformanceScoreInput;
 using mission::compute_performance_score;

 const auto failed = compute_performance_score({});
 CHECK(!failed.ranked);
 CHECK(failed.overall == 0.0);

 PerformanceScoreInput baseline;
 baseline.successful = true;
 baseline.actual_time_s = 95.0;
 baseline.target_time_s = 95.0;
 baseline.actual_energy_wh = 2.0;
 baseline.target_energy_wh = 2.0;
 baseline.clearance_m = 0.6;
 baseline.ideal_clearance_m = 0.6;
 baseline.path_length_m = 20.0;
 const auto perfect = compute_performance_score(baseline);
 CHECK(perfect.ranked);
 CHECK(std::abs(perfect.overall - 100.0) < 1e-12);

 auto slow = baseline;
 slow.actual_time_s = 133.0;
 const auto slow_score = compute_performance_score(slow);
 CHECK(std::abs(slow_score.pace - 100.0 * 95.0 / 133.0) < 1e-12);
 CHECK(slow_score.overall < perfect.overall);

 // A successful close pass remains ranked. The ideal margin changes quality,
 // not physical collision or trajectory-admission semantics.
 auto close_pass = baseline;
 close_pass.clearance_m = 0.3;
 const auto close_score = compute_performance_score(close_pass);
 CHECK(close_score.ranked);
 CHECK(std::abs(close_score.safety - 50.0) < 1e-12);
 auto extra_clearance = baseline;
 extra_clearance.clearance_m = 4.0;
 CHECK(compute_performance_score(extra_clearance).safety == 100.0);

 auto inefficient = baseline;
 inefficient.actual_energy_wh = 4.0;
 CHECK(compute_performance_score(inefficient).efficiency == 50.0);
 auto oscillating = baseline;
 oscillating.command_variation_mps = 20.0;
 CHECK(compute_performance_score(oscillating).smoothness < perfect.smoothness);
 auto indecisive = baseline;
 indecisive.wait_time_s = 80.0;
 indecisive.recovery_time_s = 15.0;
 indecisive.strategy_switches = 30;
 CHECK(compute_performance_score(indecisive).decisiveness < perfect.decisiveness);

 std::cout << "performance score green\n";
 return 0;
}
