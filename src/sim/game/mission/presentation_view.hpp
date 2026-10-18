#pragma once

#include "sim/game/scenario/scenario_session.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace mission {

// Read-only boundary presented to graphical consumers for one main-loop
// frame. The current single-threaded client borrows immutable state and
// authored content from ScenarioSession; it exposes no stepping, control,
// replay-seek, or lifecycle operation. A concurrent client can later replace
// this borrowed view with an owned snapshot without changing renderer input
// semantics.
class PresentationFrameView {
 public:
 explicit PresentationFrameView(const ScenarioSession& mission)
 : mission_(mission), snapshot_(&mission.snapshot()) {}

 // Render between the previous and current fixed-step states. This owned
 // presentation sample is display-only: physics, sensors, autonomy, replay
 // and artifacts continue to consume the authoritative current snapshot.
 PresentationFrameView(const ScenarioSession& mission, const MissionSnapshot& previous,
 double alpha)
 : mission_(mission), interpolated_(interpolate(previous, mission.snapshot(), alpha)),
 snapshot_(&*interpolated_) {}

 bool endless_mode() const { return mission_.endless_mode(); }
 const FieldDispatchState& field_dispatch() const { return mission_.field_dispatch(); }
 const FieldOperatorState& field_operator() const { return mission_.field_operator(); }
 const FieldIntentState& field_intent() const { return mission_.field_intent(); }
 const WorldActivityState& world_activity() const { return mission_.world_activity(); }
 const GoalStep* active_goal() const { return mission_.active_goal(); }
 const LevelDefinition& level() const { return mission_.level(); }
 const MapDefinition& map_definition() const { return mission_.map_definition(); }
 const BuildSelection& build() const { return mission_.build(); }
 const RobotDefinition& robot() const { return mission_.robot(); }
 const sim::GridMap& navigation_map() const { return mission_.navigation_map(); }
 const sim::TerrainModel& terrain_model() const { return mission_.terrain_model(); }
 const std::optional<ConstructionPlanResolution>& committed_construction() const {
 return mission_.committed_construction();
 }
 const sim::ChassisCaps& chassis_caps() const { return mission_.chassis_caps(); }
 const std::vector<sim::RoutePoint>& planned_route() const { return mission_.planned_route(); }
 const MissionSnapshot& snapshot() const { return *snapshot_; }
 const PayloadState& payload_state() const { return mission_.payload_state(); }
 const MissionResult& result() const { return mission_.result(); }
 const std::vector<MissionSnapshot>& replay() const { return mission_.replay(); }
 double navigation_traversal_cost(sim::Vec2 from, sim::Vec2 to) const {
 return mission_.navigation_traversal_cost(from, to);
 }

 private:
 static double lerp(double from, double to, double alpha) {
 return from + (to - from) * alpha;
 }

 static double lerp_angle(double from, double to, double alpha) {
 return sim::wrap_angle(from + sim::wrap_angle(to - from) * alpha);
 }

 static sim::physics::Quaternion interpolate_rotation(sim::physics::Quaternion from,
 sim::physics::Quaternion to, double alpha) {
 const double dot = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
 if (dot < 0.0) {
 to = {-to.x, -to.y, -to.z, -to.w};
 }
 sim::physics::Quaternion result{
 lerp(from.x, to.x, alpha),
 lerp(from.y, to.y, alpha),
 lerp(from.z, to.z, alpha),
 lerp(from.w, to.w, alpha),
 };
 const double length = std::sqrt(result.x * result.x + result.y * result.y +
 result.z * result.z + result.w * result.w);
 if (length > 1e-12) {
 result.x /= length;
 result.y /= length;
 result.z /= length;
 result.w /= length;
 }
 return result;
 }

 static MissionSnapshot interpolate(const MissionSnapshot& previous,
 const MissionSnapshot& current, double alpha) {
 MissionSnapshot output = current;
 const double blend = std::clamp(alpha, 0.0, 1.0);
 output.time_s = lerp(previous.time_s, current.time_s, blend);
 output.robot.x = lerp(previous.robot.x, current.robot.x, blend);
 output.robot.y = lerp(previous.robot.y, current.robot.y, blend);
 output.robot.theta = lerp_angle(previous.robot.theta, current.robot.theta, blend);
 output.robot.v = lerp(previous.robot.v, current.robot.v, blend);
 output.robot.w = lerp(previous.robot.w, current.robot.w, blend);
 output.robot.lateral_v = lerp(previous.robot.lateral_v, current.robot.lateral_v, blend);
 output.robot_height_m = lerp(previous.robot_height_m, current.robot_height_m, blend);
 output.robot_roll_rad = lerp_angle(previous.robot_roll_rad, current.robot_roll_rad, blend);
 output.robot_pitch_rad = lerp_angle(previous.robot_pitch_rad, current.robot_pitch_rad, blend);
 output.estimated_position.x = lerp(previous.estimated_position.x,
 current.estimated_position.x, blend);
 output.estimated_position.y = lerp(previous.estimated_position.y,
 current.estimated_position.y, blend);
 output.estimated_heading = lerp_angle(previous.estimated_heading,
 current.estimated_heading, blend);

 if (previous.actors.size() == output.actors.size()) {
 for (std::size_t index = 0; index < output.actors.size(); ++index) {
 if (previous.actors[index].id != output.actors[index].id)
 continue;
 output.actors[index].position.x = lerp(previous.actors[index].position.x,
 current.actors[index].position.x, blend);
 output.actors[index].position.y = lerp(previous.actors[index].position.y,
 current.actors[index].position.y, blend);
 output.actors[index].heading_rad = lerp_angle(previous.actors[index].heading_rad,
 current.actors[index].heading_rad, blend);
 }
 }

 for (auto& actuation : output.control.mount_actuations) {
 const auto old = std::find_if(previous.control.mount_actuations.begin(),
 previous.control.mount_actuations.end(), [&](const auto& candidate) {
 return candidate.mount_id == actuation.mount_id;
 });
 if (old == previous.control.mount_actuations.end())
 continue;
 actuation.yaw_rad = lerp_angle(old->yaw_rad, actuation.yaw_rad, blend);
 actuation.roll_rad = lerp_angle(old->roll_rad, actuation.roll_rad, blend);
 if (old->base_from_link && actuation.base_from_link) {
 auto& transform = *actuation.base_from_link;
 transform.translation_m.x = lerp(old->base_from_link->translation_m.x,
 transform.translation_m.x, blend);
 transform.translation_m.y = lerp(old->base_from_link->translation_m.y,
 transform.translation_m.y, blend);
 transform.translation_m.z = lerp(old->base_from_link->translation_m.z,
 transform.translation_m.z, blend);
 transform.rotation = interpolate_rotation(old->base_from_link->rotation,
 transform.rotation, blend);
 }
 }
 return output;
 }

 const ScenarioSession& mission_;
 std::optional<MissionSnapshot> interpolated_;
 const MissionSnapshot* snapshot_{nullptr};
};

} // namespace mission
