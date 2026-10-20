#pragma once

// Compatibility include for the SIM mission layer. Collision geometry and
// clearance are ROBOT autonomy contracts; SIM supplies observations and
// authored geometry but does not own the algorithm.
#include "robot/autonomy/validation/collision_monitor.hpp"

namespace mission {
using CollisionBackend = sim::autonomy::CollisionBackend;
using CollisionBox = sim::autonomy::CollisionBox;
using CollisionActorObservation = sim::autonomy::CollisionActorObservation;
using CollisionObservation = sim::autonomy::CollisionObservation;
using CollisionAssessment = sim::autonomy::CollisionAssessment;
using CollisionMonitor = sim::autonomy::CollisionMonitor;
} // namespace mission
