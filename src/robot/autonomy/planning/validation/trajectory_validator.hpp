#pragma once

#include "robot/autonomy/planning/local/dwa.hpp"

#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace sim {

enum class TrajectoryVerdict {
 Accepted,
 MinimumRiskEscape,
 // A behavior-level schedule proved the complete crossing window. The local
 // trajectory preserves forward progress and physical clearance when a
 // newly observed soft-margin change makes stopping inside less safe.
 CommittedTransit,
 InvalidStructure,
 StaticCollision,
 DynamicCollision,
 PredictionHorizonShort,
 UnsafeEmergencyStop,
 BehaviorConstraint,
};

// Geometry supplied by behavior planning to bound otherwise feasible local
// trajectories. The stop-plane tangent points toward the conflict; before a
// commitment, rollouts may approach that plane but may not cross it.
struct TrajectoryConstraints {
 bool stop_plane_active{false};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 double stop_tolerance_m{0.0};
 // RO-1 braking admissibility. The exact candidate is followed during the
 // decision/actuator response delay, then both differential twist components
 // decay to zero under the identified tire-limited deceleration.
 double emergency_stop_delay_s{0.0};
 double emergency_decel_mps2{0.0};
 // Once behavior has committed to a conflict-zone transit, a fully
 // margin-compliant forward rollout may be the minimum-risk contingency when
 // braking would strand the body in the conflict. This never waives nominal
 // static or dynamic trajectory validation.
 bool committed_transit{false};
 double committed_progress_floor_mps{0.0};
};

struct TrajectoryValidation {
 TrajectoryVerdict verdict{TrajectoryVerdict::InvalidStructure};
 double min_dynamic_clearance_m{std::numeric_limits<double>::infinity()};
 std::string diagnostic;
 bool accepted() const {
 return verdict == TrajectoryVerdict::Accepted || verdict == TrajectoryVerdict::MinimumRiskEscape ||
 verdict == TrajectoryVerdict::CommittedTransit;
 }
 bool minimum_risk_escape() const { return verdict == TrajectoryVerdict::MinimumRiskEscape; }
 bool committed_transit() const { return verdict == TrajectoryVerdict::CommittedTransit; }
};

struct ValidatedTrajectorySelection {
 ExecutableTrajectory trajectory;
 TrajectoryValidation validation;
 std::size_t accepted_candidates{0};
};

// Validates the exact time-parameterized rollout selected by the local
// planner. The same samples are published to control and replay; callers may
// not substitute a global route or a constant-speed approximation.
TrajectoryValidation
validate_executable_trajectory(const ExecutableTrajectory& trajectory, const GridMap& planning_map,
                               const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                               double prediction_frame_dt_s, double robot_radius_m, double required_margin_m,
 StaticFeasibility terrain_feasible = {}, const TrajectoryConstraints& constraints = {});

// Applies the continuous contract to every collision-free rollout before
// score arbitration. Fully margin-compliant trajectories outrank minimum-
// risk escapes; score orders candidates only inside the same verdict class.
ValidatedTrajectorySelection
select_executable_trajectory(const DwaResult& proposals, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double prediction_frame_dt_s, double robot_radius_m, double required_margin_m,
 StaticFeasibility terrain_feasible = {}, const TrajectoryConstraints& constraints = {});

const char* trajectory_verdict_name(TrajectoryVerdict verdict);

} // namespace sim
