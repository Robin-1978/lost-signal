#pragma once

#include "autonomy/behavior/crossing_behavior.hpp"
#include "mission/content.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace mission {

struct DynamicTrafficDecision {
 sim::CrossingEvaluation assessment;
 bool has_active_zone{false};
 bool control_authority{false};
 sim::CrossingOutput control;
};

// Mission-to-autonomy adapter for authored traffic semantics. It translates
// content activation/authority once, then delegates neutral geometry and
// prediction math to autonomy. Actor truth and physics objects never enter.
class DynamicTrafficSystem {
 public:
 void reset();
 DynamicTrafficDecision assess(const std::vector<ConflictZoneDefinition>& definitions,
                               const std::string& level_id, const std::vector<sim::RoutePoint>& route,
                               std::size_t path_index, sim::Vec2 ego_position, double ego_speed_mps,
                               double cruise_mps, double brake_decel_mps2,
                               const std::vector<std::vector<sim::DynamicObjectState>>& predicted_frames,
                               double frame_dt_s, double ego_radius_m, double margin_m, double decision_dt_s);

 private:
 sim::CrossingState crossing_state_;
};

} // namespace mission
