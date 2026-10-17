#pragma once

#include "autonomy/planning/local/predictor.hpp"
#include "autonomy/planning/path.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sim {

// Neutral map semantics consumed by autonomy. Mission content adapters may
// author boxes, polygons, or lanelets later; this first contract exposes only
// the region needed to derive route entry, stop, and exit points.
struct ConflictZone {
 std::string id;
 Vec2 min;
 Vec2 max;
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
 bool stop_reached{false};
 // An empty or stale forecast is unknown, never evidence that a controlled
 // crossing is clear. Perception must provide at least one current tracked
 // hypothesis before behavior may accumulate a release confirmation.
 bool prediction_evidence_available{false};
 bool relevant_threats_moving_away{false};
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
};

// Observable pose gate used before a stopped vehicle may commit to a timed
// crossing. Both longitudinal and lateral error matter: a distant pose can
// lie beyond the infinite stop plane without being at the derived stop pose.
struct CrossingReadiness {
 bool position_ready{false};
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

// Locate the next conflict zone on the canonical global route and assess the
// bounded route segment through its exit against time-indexed tracked-object
// predictions. This function owns no mission state and reads no actor truth.
CrossingEvaluation evaluate_crossing(const std::vector<ConflictZone>& zones,
                                     const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_heading_rad, double ego_speed_mps,
                                     const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution);

enum class CrossingPhase { Inactive, Holding, Committed, Cleared };

struct CrossingInput {
 bool relevant{false};
 bool body_in_conflict{false};
 bool body_cleared{false};
 // A stop-controlled crossing may accumulate its safe-window confirmation
 // only after the body reaches the fixed stop line and settles.
 bool ready_to_commit{false};
 bool safe_to_commit{false};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 bool stop_valid{false};
 Vec2 exit_point{0, 0};
 Vec2 exit_direction{1, 0};
 bool exit_valid{false};
 double transit_speed_mps{0.0};
 double dt_s{0.0};
};

struct CrossingState {
 CrossingPhase phase{CrossingPhase::Inactive};
 double clear_time_s{0.0};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 bool stop_valid{false};
 Vec2 exit_point{0, 0};
 Vec2 exit_direction{1, 0};
 bool exit_valid{false};
 double transit_speed_mps{0.0};
 std::string zone_id;
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
};

constexpr double kCrossingCommitConfirmS = 0.40;
// Time separation around the predicted conflict interval. Admission sweeps
// both early and late execution by this amount; the spatial validator remains
// an independent hard gate.
constexpr double kCrossingTimeMarginS = 0.20;
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
 const CrossingExecutionCaps& execution, double decision_dt_s);

 private:
 CrossingState state_;
 std::string cleared_zone_id_;
};

} // namespace sim
