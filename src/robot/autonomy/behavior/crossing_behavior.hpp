#pragma once

#include "robot/autonomy/mapping/observed_obstacle_layer.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"
#include "robot/autonomy/planning/path.hpp"

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace sim {

using CrossingPathFeasibility = std::function<bool(Vec2, Vec2)>;

// Map-semantic traffic approach used for occlusion diagnostics. The polyline
// starts on the conflict-region boundary and extends outward along a possible
// participant path. It describes geometry only: a map cannot provide the
// physical capability of an unknown real-world participant.
struct CrossingObservationApproach {
 std::vector<Vec2> centerline;
 double half_width_m{0.0};
};

// Neutral map semantics consumed by autonomy. Mission content adapters may
// author boxes, polygons, or lanelets later; this first contract exposes only
// the region needed to derive route entry, stop, and exit points.
struct ConflictZone {
 std::string id;
 Vec2 min;
 Vec2 max;
 std::vector<CrossingObservationApproach> observation_approaches;
};

// Behavior authority is part of the autonomy input contract. A world/content
// adapter may declare that a region is observable only or may constrain
// motion, but it never owns the state machine or computes a stop pose.
enum class CrossingAuthority { Observe, Control };

struct CrossingZoneSpec {
 ConflictZone geometry;
 CrossingAuthority authority{CrossingAuthority::Observe};
};

struct CrossingEvaluation {
 bool relevant{false};
 // Physical body occupancy is derived from geometry. Controller lookahead
 // progress must never stand in for entering or clearing the crossing.
 bool body_in_conflict{false};
 // The body has entered the approach envelope from which stopping and
 // waiting is no longer proved safe, even if its physical footprint has not
 // yet intersected the semantic conflict box.
 bool no_return_started{false};
 bool stop_reached{false};
 // An empty or stale forecast is unknown, never evidence that a controlled
 // crossing is clear. Perception must provide at least one current tracked
 // hypothesis before behavior may accumulate a release confirmation.
 bool prediction_evidence_available{false};
 // Every relevant track has enough timestamp-distinct history for behavior
 // prediction. This is estimator readiness, not proof of a bounded cycle;
 // velocity sign alone is never a temporal conflict test.
 bool prediction_models_ready{false};
 bool relevant_threats_moving_away{false};
 // LiDAR coverage is diagnostic evidence. It cannot independently prove an
 // authored approach clear or replace a missing live track.
 bool free_space_evidence_available{false};
 bool observed_approaches_clear{false};
 double observation_stamp_s{-std::numeric_limits<double>::infinity()};
 double observation_coverage{0.0};
 double unseen_arrival_lower_bound_s{0.0};
 std::string zone_id;
 std::size_t entry_index{0};
 std::size_t exit_index{0};
 std::size_t stop_index{0};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 Vec2 exit_point{0, 0};
 Vec2 exit_direction{1, 0};
 bool body_cleared{false};
 double stop_distance_m{0.0};
 double transit_speed_mps{0.0};
 std::vector<TimedPose> execution_trajectory;
 // Continuous trajectories contain intermediate curve samples, not stop
 // waypoints. Executors must track them without inserting a stop and an
 // in-place turn at every sample.
 bool execution_continuous{false};
 double execution_duration_s{0.0};
 double execution_min_gap_m{std::numeric_limits<double>::infinity()};
 double execution_required_gap_m{0.0};
 DepartureAssessment departure;
};

struct CrossingSafetyEnvelope {
 double ego_radius_m{0.0};
 double clearance_m{0.0};
 // Longitudinal uncertainty from localization, discrete control and actuator
 // response. The stop controller settles this far before the geometric line.
 double stop_position_tolerance_m{0.0};
 // A crossing commitment is a pose contract, not just a point-and-speed
 // contract. The body must face the route tangent before confirmation can
 // accumulate; otherwise a differential robot can be released while it is
 // still turning back toward an overshot stop line.
 double stop_heading_tolerance_rad{0.35};
 // Validated bound between the behavior reference and the executable local
 // trajectory. Admission reserves this tube in addition to actor clearance;
 // a center-line gap narrower than the tracker contract is not executable.
 double tracking_error_m{0.0};
 // Bounded downstream sweep of the collision/reference frame while the
 // chassis controller converges to the hold pose. An offset differential
 // axle moves base_link during yaw even when the driven axle is stationary.
 // The stop pose is backed up by this amount before the validator is allowed
 // to consume the same tolerance; the pair preserves physical clearance.
 double stop_plane_tracking_tolerance_m{0.0};
};

// Vehicle-neutral execution limits used by behavior planning. Mission code
// resolves these from the assembled platform; autonomy never depends on a
// concrete simulated chassis or driver implementation.
struct CrossingExecutionCaps {
 double preferred_speed_mps{0.0};
 double max_speed_mps{0.0};
 double max_accel_mps2{0.0};
 double max_decel_mps2{0.0};
 double max_yaw_rate_radps{0.0};
 double max_yaw_accel_radps2{0.0};
 // End-to-end decision and actuator response delay before a newly requested
 // rest-to-motion phase develops measurable motion.
 double actuator_response_delay_s{0.0};
 // Route distance over which a mapped conflict may activate behavior. This
 // is the deployable prediction horizon used for approach control, separate
 // from any longer forecast generated to score a complete transit.
 double activation_horizon_s{0.0};
 // True only when the active local planner and actuator plant have a
 // validated continuous-curvature tracking contract. A chassis being able
 // to spin or generate yaw rate is insufficient evidence by itself.
 bool continuous_curvature_tracking{false};
};

// Observable pose gate used before a stopped vehicle may commit to a timed
// crossing. Both longitudinal and lateral error matter: a distant pose can
// lie beyond the infinite stop plane without being at the derived stop pose.
struct CrossingReadiness {
 bool position_ready{false};
 // Inner acquisition threshold for the stop-pose controller. Behavior still
 // uses position_ready for admission; this tighter state gives final heading
 // alignment room for measured contact/localization motion.
 bool position_settled{false};
 bool speed_ready{false};
 bool heading_ready{false};
 double stop_longitudinal_m{0.0};
 double stop_lateral_m{0.0};
 double heading_error_rad{0.0};
 bool ready() const { return position_ready && speed_ready && heading_ready; }
};

CrossingReadiness assess_crossing_readiness(
 Vec2 ego_position, double ego_speed_mps, double ego_heading_rad,
 Vec2 stop_point, Vec2 stop_direction,
 const CrossingSafetyEnvelope& safety);

struct CrossingPoseApproach {
 bool reverse{false};
 double body_heading_rad{0.0};
 bool control_position_acquired{false};
 double control_distance_m{0.0};
};

// A differential pose correction may use either gear. Choosing the body
// heading with the smaller rotation avoids a pi-radian bearing flip after a
// centimetre-scale stop-point overshoot. Forced recovery remains aligned to
// the route-derived stop direction.
CrossingPoseApproach crossing_pose_approach(
 Vec2 ego_position, double ego_heading_rad, Vec2 stop_point,
 Vec2 stop_direction, bool force_reverse,
 double drive_axle_x_m, double acquisition_radius_m);

// Locate the next conflict zone on the canonical global route and assess the
// bounded route segment through its exit against time-indexed tracked-object
// predictions. This function owns no mission state and reads no actor truth.
CrossingEvaluation evaluate_crossing(const std::vector<ConflictZone>& zones,
                                     const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_heading_rad, double ego_speed_mps,
                                     const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution,
 const std::vector<ObservedRay2D>& visibility_rays = {},
 double visibility_stamp_s = -std::numeric_limits<double>::infinity(),
 double evaluation_stamp_s = -std::numeric_limits<double>::infinity(),
 bool committed_continuation = false,
 CrossingPathFeasibility path_feasible = {});

enum class CrossingPhase { Inactive, Holding, Committed, Cleared };

struct CrossingInput {
 bool relevant{false};
 bool body_in_conflict{false};
 bool no_return_started{false};
 bool body_cleared{false};
 // A stop-controlled crossing may accumulate its safe-window confirmation
 // only after the body reaches the fixed stop line and settles.
 bool ready_to_commit{false};
 bool safe_to_commit{false};
 bool position_ready{false};
 bool position_settled{false};
 // A changed forecast may make the previously admitted transit unsafe before
 // the body enters the conflict. Once the local safety layer has brought the
 // vehicle to rest, behavior can revoke the stale commitment and return to
 // the route-derived hold contract.
 bool stopped_before_conflict{false};
 // Feedback from the independently validated local execution on the prior
 // control cycle. A planned zero-speed corner or alignment is not a safety
 // stop and must not revoke its own commitment.
 bool committed_execution_blocked{false};
 // The prior local command was a separately named minimum-risk escape, not
 // an execution of the admitted crossing trajectory. Once that contingency
 // has carried the body completely out of the conflict approach envelope,
 // the old time schedule is no longer meaningful and must be re-admitted.
 bool committed_contingency_escape{false};
 Vec2 ego_position{0, 0};
 double stop_longitudinal_m{0.0};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 bool stop_valid{false};
 Vec2 exit_point{0, 0};
 Vec2 exit_direction{1, 0};
 bool exit_valid{false};
 double transit_speed_mps{0.0};
 double dt_s{0.0};
 // Confirmation counts distinct sensor observations. A fast decision loop
 // must not count one LiDAR frame twice.
 double evidence_stamp_s{-std::numeric_limits<double>::infinity()};
 // The behavior trajectory and the spatial reserve proven at admission.
 // They are latched with the commitment so later perception updates can
 // revoke safety claims without silently changing what motion was proved.
 std::vector<TimedPose> execution_trajectory;
 bool execution_continuous{false};
 double tracking_corridor_m{0.0};
 double execution_initial_heading_rad{0.0};
 bool execution_alignment_required{false};
 bool execution_alignment_complete{true};
};

struct CrossingState {
 CrossingPhase phase{CrossingPhase::Inactive};
 double clear_time_s{0.0};
 int clear_observation_samples{0};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 bool stop_valid{false};
 Vec2 exit_point{0, 0};
 Vec2 exit_direction{1, 0};
 bool exit_valid{false};
 double transit_speed_mps{0.0};
 std::string zone_id;
 double last_clear_evidence_stamp_s{-std::numeric_limits<double>::infinity()};
 std::vector<TimedPose> execution_trajectory;
 bool execution_continuous{false};
 double tracking_corridor_m{0.0};
 // A pre-entry commitment was revoked before physical conflict entry.
 // Recovery is an explicit reverse maneuver along the route; it must not be
 // inferred from a forward route follower's score.
 bool recovery_to_stop{false};
 // Recovery gear uses a spatial Schmitt trigger. Contact and localization
 // motion near the stop plane must not alternate the desired body heading.
 bool recovery_reverse{false};
 bool hold_position_acquired{false};
 // Latches when the body crosses the no-return line or physically enters the
 // conflict region. A later local rejection must never reverse the vehicle
 // back through cross traffic; the committed maneuver continues to refuge.
 bool no_return_started{false};
 bool contingency_escape_started{false};
 // The original stop pose is no longer reachable without re-entering the
 // hazard after a minimum-risk lateral/upstream escape. Admission is rebuilt
 // from this measured, validator-approved stationary refuge.
 bool relocated_hold{false};
 double execution_initial_heading_rad{0.0};
 bool execution_alignment_required{false};
 bool execution_alignment_complete{true};
 double execution_elapsed_s{0.0};
};

struct CrossingOutput {
 CrossingPhase phase{CrossingPhase::Inactive};
 bool hold_active{false};
 bool committed{false};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 bool stop_valid{false};
 Vec2 exit_point{0, 0};
 bool exit_valid{false};
 double transit_speed_mps{0.0};
 std::vector<TimedPose> execution_trajectory;
 bool execution_continuous{false};
 double tracking_corridor_m{0.0};
 bool recovery_to_stop{false};
 bool recovery_reverse{false};
 bool hold_position_acquired{false};
 bool no_return_started{false};
 bool relocated_hold{false};
 double execution_initial_heading_rad{0.0};
 bool execution_alignment_required{false};
 bool execution_alignment_complete{true};
 // Time since this exact trajectory was committed. The executor consumes
 // the latched trajectory in the same time domain used by admission rather
 // than turning it into an untimed list of waypoints.
 double execution_elapsed_s{0.0};
};

// Release must be supported by independent consecutive observations. A
// duration-only gate degenerates into one sample on a slow computer and can
// outlive target visibility on a fast one; the observation count preserves
// the same evidence contract at every declared decision cadence.
constexpr int kCrossingCommitConfirmSamples = 2;
// Time separation around the predicted conflict interval. Admission sweeps
// both early and late execution by this amount; the spatial validator remains
// an independent hard gate.
constexpr double kCrossingTimeMarginS = 0.20;
constexpr double kCrossingInitialHeadingToleranceRad = 0.08;
// Current differential crossing executor's declared position tolerance.
// This is a control-space quantity in metres, independent of occupancy-grid
// resolution. A measured platform/provider may replace it with a calibrated
// tracking envelope without changing behavior semantics.
constexpr double kCrossingPositionTrackingToleranceM = 0.10;
CrossingOutput govern_crossing(CrossingState& state, const CrossingInput& input,
                               const std::string& zone_id = {});

struct CrossingBehaviorDecision {
 CrossingEvaluation assessment;
 bool has_active_zone{false};
 bool control_authority{false};
 CrossingOutput control;
 bool phase_changed{false};
 bool ready_to_commit{false};
 bool safe_to_commit{false};
 CrossingReadiness readiness;
};

// Stateful autonomy behavior. Its inputs contain only map semantics, the
// estimated ego state, the global route, and perceived-object predictions.
// Simulation actors, mission objectives, physics bodies, and renderer state
// are intentionally absent, so the same object can run on a real robot.
class CrossingBehaviorSystem {
 public:
 void reset();
 CrossingBehaviorDecision assess(
 const std::vector<CrossingZoneSpec>& zones,
 const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_speed_mps, double ego_heading_rad,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution, double decision_dt_s,
 const std::vector<ObservedRay2D>& visibility_rays = {},
 double visibility_stamp_s = -std::numeric_limits<double>::infinity(),
 double evaluation_stamp_s = -std::numeric_limits<double>::infinity(),
 bool committed_execution_blocked = false,
 bool committed_contingency_escape = false,
 CrossingPathFeasibility path_feasible = {});

 private:
 CrossingState state_;
 std::string cleared_zone_id_;
};

} // namespace sim
