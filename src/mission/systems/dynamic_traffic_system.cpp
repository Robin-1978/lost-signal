#include "mission/systems/dynamic_traffic_system.hpp"

#include <algorithm>
#include <cmath>

namespace mission {

void DynamicTrafficSystem::reset() {
 crossing_state_ = {};
}

DynamicTrafficDecision
DynamicTrafficSystem::assess(const std::vector<ConflictZoneDefinition>& definitions,
                             const std::string& level_id, const std::vector<sim::RoutePoint>& route,
                             std::size_t path_index, sim::Vec2 ego_position, double ego_speed_mps,
                             double cruise_mps, double brake_decel_mps2,
                             const std::vector<std::vector<sim::DynamicObjectState>>& predicted_frames,
                             double frame_dt_s, double ego_radius_m, double margin_m, double decision_dt_s) {
 std::vector<sim::ConflictZone> active;
 active.reserve(definitions.size());
 for (const auto& definition : definitions) {
  if (active_in_level(definition.active_in, level_id))
   active.push_back({definition.id, definition.min, definition.max});
 }

 DynamicTrafficDecision decision;
 decision.has_active_zone = !active.empty();
 decision.assessment =
   sim::evaluate_crossing(active, route, path_index, ego_position, cruise_mps, brake_decel_mps2,
                          predicted_frames, frame_dt_s, ego_radius_m, margin_m);
 if (!decision.assessment.relevant) {
  decision.control = sim::govern_crossing(crossing_state_, {});
  return decision;
 }

 const auto definition = std::find_if(definitions.begin(), definitions.end(), [&](const auto& candidate) {
  return candidate.id == decision.assessment.zone_id && active_in_level(candidate.active_in, level_id);
 });
 decision.control_authority = definition != definitions.end() && definition->authority == "control";
 if (!decision.control_authority) {
  decision.control = sim::govern_crossing(crossing_state_, {});
  return decision;
 }

 sim::CrossingInput input;
 input.relevant = true;
 input.inside_zone = decision.assessment.inside_zone;
 input.ready_to_commit = decision.assessment.stop_reached && std::abs(ego_speed_mps) <= 0.10;
 input.safe_to_commit = decision.assessment.departure.reason == sim::DepartReason::kGoNow;
 input.stop_point = decision.assessment.stop_point;
 input.stop_valid = true;
 input.dt_s = decision_dt_s;
 decision.control = sim::govern_crossing(crossing_state_, input, decision.assessment.zone_id);
 return decision;
}

} // namespace mission
