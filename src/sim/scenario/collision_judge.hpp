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

// Belly / high-center assessment (SIM truth-side, never ROBOT input).
// Clearance = engine body height minus the highest authored surface over
// taps across the footprint disc, minus the chassis belly clearance.
// Flat floor reads the ride height (positive); straddling a center ridge
// or beaching on a crest drives it negative. Taps (center + cardinals)
// catch narrow ridges a center-only sample would miss; the max keeps it
// conservative. Slabs count (decks are surfaces); boxes do not (the static
// judge owns them). Callers gate on a live plant (drive_count > 0): frozen
// zero truth carries no height, and z = 0 must never assess as contact.
eval::CollisionOutcome assess_terrain_contact(
    double body_x_m, double body_y_m, double body_z_m,
    double footprint_radius_m,
    const std::vector<world::TerrainPatch>& patches,
    const std::vector<world::SupportSlab>& slabs,
    double ground_clearance_m);

}  // namespace lost_signal::sim::scenario
