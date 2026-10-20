#pragma once

#include "robot/core/math.hpp"

#include <algorithm>

namespace sim {

// Transport-neutral planar object state. Simulation truth, perception output,
// prediction and planning may all use this value without depending on one
// another's implementation layer.
struct DynamicObjectState {
 Vec2 p{0.0, 0.0};
 Vec2 v{0.0, 0.0};
 double radius{0.45};
 // Radial position-prediction uncertainty, kept separate from physical
 // geometry so planners can report physical and conservative clearance with
 // one unambiguous contract. Predictors grow this value with forecast time.
 double position_uncertainty_m{0.0};
 // True when perception has enough timestamp-distinct history for this state
 // to participate in a behavior decision. This is estimator readiness, not
 // proof of periodic motion or a physical capability bound. Local collision
 // avoidance still consumes unready tracks conservatively.
 bool prediction_ready{false};

 double occupied_radius_m() const {
 return std::max(0.0, radius) + std::max(0.0, position_uncertainty_m);
 }
};

} // namespace sim
