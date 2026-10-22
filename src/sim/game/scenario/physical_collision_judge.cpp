#include "physical_collision_judge.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mission {
namespace {

sim::Vec2 local_point(const PhysicalCollisionBox& box, sim::Vec2 point) {
 const sim::Vec2 center{(box.min.x + box.max.x) * 0.5,
 (box.min.y + box.max.y) * 0.5};
 const double dx = point.x - center.x;
 const double dy = point.y - center.y;
 const double c = std::cos(box.yaw_rad);
 const double s = std::sin(box.yaw_rad);
 return {c * dx + s * dy, -s * dx + c * dy};
}

double clearance_to_box(const PhysicalCollisionBox& box, sim::Vec2 point,
 double radius_m) {
 const auto local = local_point(box, point);
 const double half_x = (box.max.x - box.min.x) * 0.5;
 const double half_y = (box.max.y - box.min.y) * 0.5;
 const double dx = std::max(std::abs(local.x) - half_x, 0.0);
 const double dy = std::max(std::abs(local.y) - half_y, 0.0);
 return std::hypot(dx, dy) - radius_m;
}

} // namespace

PhysicalCollisionJudge::PhysicalCollisionJudge(
 sim::Vec2 world_size, double robot_radius_m,
 std::vector<PhysicalCollisionBox> obstacles)
 : world_size_(world_size), robot_radius_m_(robot_radius_m),
 obstacles_(std::move(obstacles)) {
 if (!(world_size_.x > 0.0) || !(world_size_.y > 0.0) || robot_radius_m_ < 0.0)
 throw std::invalid_argument("physical collision judge geometry must be positive");
}

PhysicalCollisionOutcome PhysicalCollisionJudge::assess(
 sim::Vec2 position, const sim::platform::RigidVehicleState& rigid_state,
 const std::vector<PhysicalActorTruth>& actors) const {
 PhysicalCollisionOutcome out;
 out.static_clearance_m = std::min({position.x, position.y,
 world_size_.x - position.x, world_size_.y - position.y}) - robot_radius_m_;
 for (const auto& obstacle : obstacles_)
 out.static_clearance_m = std::min(out.static_clearance_m,
 clearance_to_box(obstacle, position, robot_radius_m_));

 // Contact flags come only from the stepped physics provider. They are the
 // authoritative simulated outcome and are never passed back to ROBOT.
 out.static_collision = rigid_state.static_collision;
 for (const auto& actor : actors) {
 const double clearance = std::hypot(position.x - actor.position.x,
 position.y - actor.position.y) - actor.radius_m - robot_radius_m_;
 if (clearance < out.dynamic_clearance_m) {
 out.dynamic_clearance_m = clearance;
 out.nearest_actor_position = actor.position;
 }
 const bool contacted = std::find(rigid_state.actor_collisions.begin(),
 rigid_state.actor_collisions.end(), actor.id) != rigid_state.actor_collisions.end();
 if (!contacted)
 continue;
 if (actor.social)
 ++out.social_contacts;
 else
 out.dynamic_collision = true;
 }
 return out;
}

} // namespace mission
