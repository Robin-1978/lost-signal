#pragma once

#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/planning/global_planner.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-side task completion policy. It consumes only the estimated state and
// the map-resolved goal. The footprint is deliberately absent: collision
// clearance and goal accuracy are different contracts, so a larger chassis
// must not report arrival farther away from the requested pose.
struct GoalArrivalConfig {
 double position_tolerance_m = 0.25;
 double heading_tolerance_rad = 0.20;
 double maximum_forward_speed_mps = 0.08;
 double maximum_yaw_rate_radps = 0.08;
 // Completion is a pose claim, so the estimate uncertainty must fit inside
 // the same contract. A geometrically reached but uncertain robot remains
 // under local planning so it can collect a safe observation baseline.
 double maximum_position_sigma_m = 0.25;
 double maximum_heading_sigma_rad = 0.15;
 int settle_cycles = 3;
};

struct GoalArrivalDecision {
 bool hold = false;
 bool arrived = false;
 double position_error_m = 0.0;
 double heading_error_rad = 0.0;
 std::string reason = "inactive";
};

class GoalArrivalPolicy {
 public:
 explicit GoalArrivalPolicy(GoalArrivalConfig config = {});

 void reset();
 GoalArrivalDecision evaluate(const EstimatedState& estimate,
 const RoutePoint& goal, bool route_active,
 bool observation_motion_requested = false);

 private:
 GoalArrivalConfig config_;
 int settled_cycles_ = 0;
};

} // namespace lost_signal::robot::autonomy
