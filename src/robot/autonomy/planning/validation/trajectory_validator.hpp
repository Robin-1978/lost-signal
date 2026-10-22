#pragma once

#include "robot/autonomy/mapping/terrain.hpp"
#include "robot/autonomy/planning/local/dwa.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"

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
 DynamicStabilityViolation,
};

using TerrainSampleQuery = std::function<TerrainSample(Vec2)>;
// Safe longitudinal braking authority at a predicted map pose. The result is
// an acceleration magnitude in m/s^2 and may vary with slope, surface and
// support geometry. Keeping this as a query lets the braking tail encounter
// terrain in time order instead of applying a distant lookahead's worst value
// to the robot's current flat-ground motion.
using SafeStopDecelQuery = std::function<double(Vec2, double heading_rad)>;

// Geometry supplied by behavior planning to bound otherwise feasible local
// trajectories. The stop-plane tangent points toward the conflict; before a
// commitment, rollouts may approach that plane but may not cross it.
struct TrajectoryConstraints {
 bool stop_plane_active{false};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 double stop_tolerance_m{0.0};
 // Distinguishes an explicitly configured brake contract whose current safe
 // authority is zero from a platform that does not model emergency braking.
 bool emergency_stop_required{false};
 // RO-1 braking admissibility. The exact candidate is followed during the
 // decision/actuator response delay, then both differential twist components
 // decay to zero under the identified tire-limited deceleration.
 double emergency_stop_delay_s{0.0};
 double emergency_decel_mps2{0.0};
 SafeStopDecelQuery safe_stop_decel;
 // Differential emergency braking is integrated in wheel space. This must
 // be the same driven-wheel separation used by command execution; zero keeps
 // the generic twist-space fallback for non-differential providers.
 double differential_track_m{0.0};
 // Once behavior has committed to a conflict-zone transit, a fully
 // margin-compliant forward rollout may be the minimum-risk contingency when
 // braking would strand the body in the conflict. This never waives nominal
 // static or dynamic trajectory validation.
 bool committed_transit{false};
 double committed_progress_floor_mps{0.0};
 // One-cycle contingency used only when an approaching dynamic object would
 // strike a stopped, already-committed differential base and every forward
 // candidate failed. It requires real reverse progress and a lower-risk
 // result than brake-and-hold; it is never enabled by static margins alone.
 bool dynamic_reverse_escape{false};
 double dynamic_reverse_speed_floor_mps{0.0};
 std::vector<Vec2> committed_reference_path;
 double committed_tracking_corridor_m{0.0};
 // Remaining suffix of the exact behavior trajectory, expressed relative to
 // this planning cycle. A committed local action is checked against this
 // timed continuation instead of being treated as a permanent hold at the
 // end of its short DWA rollout.
 std::vector<TimedPose> committed_reference_trajectory;
 // Candidate-level quasi-static rollover envelope. Terrain comes from the
 // deployable map/perception contract; simulation world truth is not visible
 // here. A non-positive dimension or risk limit disables this optional
 // capability for platforms that have not declared stability geometry.
 TerrainSampleQuery terrain_sample;
 double support_width_m{0.0};
 std::vector<Vec2> support_polygon;
 double center_of_mass_x_m{0.0};
 double center_of_mass_height_m{0.0};
 double max_dynamic_rollover_risk{0.0};
 // Sensor-frame returns relative to the trajectory start pose. A candidate
 // may not sweep through one even if a localization error placed the same
 // return in an occupied or free map cell incorrectly.
 std::vector<Vec2> live_obstacle_points_in_base;
 double live_obstacle_margin_m{0.05};
};

struct TrajectoryValidation {
 TrajectoryVerdict verdict{TrajectoryVerdict::InvalidStructure};
 // True only when the candidate's next command interval can be joined to
 // the remaining time-parameterized behavior trajectory and that complete
 // continuation remains safe against the current forecast. A locally safe
 // hold or detour does not preserve an earlier crossing admission merely
 // because its short rollout is collision-free.
 bool preserves_committed_schedule{false};
 // Tracking evidence at the next command boundary, plus a short preview
 // cost over the rollout. Selection uses the preview to choose among
 // candidates that preserve the same admitted schedule; position-only
 // arbitration can wait until a curve is no longer reachable before
 // increasing yaw authority.
 double committed_position_error_m{std::numeric_limits<double>::infinity()};
 double committed_heading_error_rad{std::numeric_limits<double>::infinity()};
 double committed_speed_error_mps{std::numeric_limits<double>::infinity()};
 double committed_tracking_cost{std::numeric_limits<double>::infinity()};
 double min_dynamic_clearance_m{std::numeric_limits<double>::infinity()};
 // Positive separation gained by a named minimum-risk escape from an
 // already-violated soft margin. Selection uses this before ordinary planner
 // score, because a contingency must maximize recovery from the hazard.
 double escape_clearance_gain_m{0.0};
 double max_dynamic_rollover_risk{0.0};
 std::string diagnostic;
 bool accepted() const {
 return verdict == TrajectoryVerdict::Accepted || verdict == TrajectoryVerdict::MinimumRiskEscape ||
 verdict == TrajectoryVerdict::CommittedTransit;
 }
 bool minimum_risk_escape() const { return verdict == TrajectoryVerdict::MinimumRiskEscape; }
 bool committed_transit() const { return verdict == TrajectoryVerdict::CommittedTransit; }
 // Behavior authority is owned by ROBOT. A local recovery label does not
 // revoke an admitted maneuver when the same validation still proves its
 // complete timed continuation. SIM/game adapters consume these predicates
 // and do not reinterpret planner verdict combinations.
 bool preserves_behavior_authority() const {
 return preserves_committed_schedule || committed_transit();
 }
 bool requires_committed_contingency() const {
 return minimum_risk_escape() && !preserves_behavior_authority();
 }
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
