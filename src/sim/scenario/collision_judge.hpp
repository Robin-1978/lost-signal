#pragma once

// Truth-side collision judge (SIM orchestration, never ROBOT input).
// Physics truth is a SIM outcome, never a ROBOT observation: the judge
// assesses robot-circle vs static boxes + actors AFTER the plant steps,
// for mission failure, scoring, and replay diagnostics. It is intentionally
// not reachable from RobotApplication — disagreements (robot thinks clear,
// truth says contact) are recorded as eval signals, never tick inputs.

#include <vector>

#include "sim/eval/episode.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::scenario {

eval::CollisionOutcome assess_collision(
    double body_x_m, double body_y_m, double robot_radius_m,
    const std::vector<world::StaticBox>& boxes,
    const std::vector<world::Actor>& actors);

}  // namespace lost_signal::sim::scenario
