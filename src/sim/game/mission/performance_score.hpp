#pragma once

namespace mission {

// GAME-owned evaluation of an already completed run. These values describe
// quality and never enter ROBOT trajectory admission or collision checks.
struct PerformanceScoreInput {
 bool successful{false};
 double actual_time_s{0.0};
 double target_time_s{0.0};
 double actual_energy_wh{0.0};
 double target_energy_wh{0.0};
 double clearance_m{0.0};
 double ideal_clearance_m{0.0};
 double path_length_m{0.0};
 double command_variation_mps{0.0};
 double wait_time_s{0.0};
 double recovery_time_s{0.0};
 int strategy_switches{0};
};

struct PerformanceScore {
 bool ranked{false};
 double overall{0.0};
 double safety{0.0};
 double pace{0.0};
 double efficiency{0.0};
 double smoothness{0.0};
 double decisiveness{0.0};
};

PerformanceScore compute_performance_score(const PerformanceScoreInput& input);

} // namespace mission
