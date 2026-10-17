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

struct CrossingEvaluation {
 bool relevant{false};
 bool inside_zone{false};
 bool stop_reached{false};
 std::string zone_id;
 std::size_t entry_index{0};
 std::size_t exit_index{0};
 std::size_t stop_index{0};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 double stop_distance_m{0.0};
 DepartureAssessment departure;
};

// Locate the next conflict zone on the canonical global route and assess the
// bounded route segment through its exit against time-indexed tracked-object
// predictions. This function owns no mission state and reads no actor truth.
CrossingEvaluation evaluate_crossing(const std::vector<ConflictZone>& zones,
                                     const std::vector<RoutePoint>& route, std::size_t path_index,
                                     Vec2 ego_position, double cruise_mps, double brake_decel_mps2,
                                     const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                                     double frame_dt_s, double ego_radius_m, double margin_m);

enum class CrossingPhase { Inactive, Holding, Committed };

struct CrossingInput {
 bool relevant{false};
 bool inside_zone{false};
 // A stop-controlled crossing may accumulate its safe-window confirmation
 // only after the body reaches the fixed stop line and settles.
 bool ready_to_commit{false};
 bool safe_to_commit{false};
 Vec2 stop_point{0, 0};
 bool stop_valid{false};
 double dt_s{0.0};
};

struct CrossingState {
 CrossingPhase phase{CrossingPhase::Inactive};
 double clear_time_s{0.0};
 Vec2 stop_point{0, 0};
 bool stop_valid{false};
 std::string zone_id;
};

struct CrossingOutput {
 CrossingPhase phase{CrossingPhase::Inactive};
 bool hold_active{false};
 bool committed{false};
 Vec2 stop_point{0, 0};
 bool stop_valid{false};
};

constexpr double kCrossingCommitConfirmS = 0.20;
CrossingOutput govern_crossing(CrossingState& state, const CrossingInput& input,
                               const std::string& zone_id = {});

} // namespace sim
