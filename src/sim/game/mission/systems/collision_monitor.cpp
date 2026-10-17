#include "sim/game/mission/systems/collision_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mission {
namespace {

bool segment_intersects_box(sim::Vec2 from, sim::Vec2 to,
 sim::Vec2 minimum, sim::Vec2 maximum) {
 double enter = 0.0;
 double leave = 1.0;
 const auto clip_axis = [&](double start, double delta, double low, double high,
 double& t_enter, double& t_leave) {
 if (std::abs(delta) < 1e-12)
 return start < low || start > high;
 double first = (low - start) / delta;
 double second = (high - start) / delta;
 if (first > second)
 std::swap(first, second);
 t_enter = std::max(t_enter, first);
 t_leave = std::min(t_leave, second);
 return t_enter > t_leave;
 };
 if (clip_axis(from.x, to.x - from.x, minimum.x, maximum.x, enter, leave) ||
 clip_axis(from.y, to.y - from.y, minimum.y, maximum.y, enter, leave)) {
 return false;
 }
 return leave >= 0.0 && enter <= 1.0;
}

} // namespace

CollisionMonitor::CollisionMonitor(sim::Vec2 world_size, double robot_radius_m,
 std::vector<CollisionBox> static_obstacles)
 : world_size_(world_size),
 robot_radius_m_(robot_radius_m),
 static_obstacles_(std::move(static_obstacles)) {
 if (!(world_size_.x > 0.0) || !(world_size_.y > 0.0) || robot_radius_m_ < 0.0)
 throw std::invalid_argument("collision monitor geometry must be positive");
}

void CollisionMonitor::reset() {
 rigid_static_contact_s_ = 0.0;
}

bool CollisionMonitor::static_motion_free(sim::Vec2 from, sim::Vec2 to) const {
 const auto inside_world = [&](sim::Vec2 point) {
 return point.x > robot_radius_m_ && point.y > robot_radius_m_ &&
 point.x < world_size_.x - robot_radius_m_ &&
 point.y < world_size_.y - robot_radius_m_;
 };
 if (!inside_world(from) || !inside_world(to))
 return false;
 for (const auto& obstacle : static_obstacles_) {
 const sim::Vec2 minimum{
 obstacle.min.x - robot_radius_m_, obstacle.min.y - robot_radius_m_};
 const sim::Vec2 maximum{
 obstacle.max.x + robot_radius_m_, obstacle.max.y + robot_radius_m_};
 if (segment_intersects_box(from, to, minimum, maximum))
 return false;
 }
 return true;
}

double CollisionMonitor::static_clearance(sim::Vec2 position) const {
 double minimum = std::min({position.x, position.y,
 world_size_.x - position.x, world_size_.y - position.y}) - robot_radius_m_;
 for (const auto& obstacle : static_obstacles_) {
 const double dx = std::max({obstacle.min.x - position.x, 0.0,
 position.x - obstacle.max.x});
 const double dy = std::max({obstacle.min.y - position.y, 0.0,
 position.y - obstacle.max.y});
 minimum = std::min(minimum,
 std::hypot(dx, dy) - robot_radius_m_);
 }
 return std::max(0.0, minimum);
}

CollisionAssessment CollisionMonitor::assess(
 const CollisionObservation& observation) {
 CollisionAssessment out;
 out.static_clearance_m = static_clearance(observation.position);
 const bool outside = observation.position.x < 0.0 ||
 observation.position.y < 0.0 ||
 observation.position.x >= world_size_.x ||
 observation.position.y >= world_size_.y;
 if (observation.backend == CollisionBackend::RigidBody) {
 rigid_static_contact_s_ = observation.rigid_static_contact
 ? rigid_static_contact_s_ + std::max(0.0, observation.dt_s)
 : 0.0;
 out.static_collision = outside || rigid_static_contact_s_ >= 0.25;
 } else {
 rigid_static_contact_s_ = 0.0;
// The domain plant collides against authored geometry, not a rasterized
// planning costmap. Grid inflation is deliberately conservative and can add
// almost one cell around box corners; treating that approximation as contact
// reports a collision while a circular footprint still has real clearance.
// The occupancy pointer remains available as planning evidence, while the
// exact box/world footprint contract decides physical contact here.
 const bool footprint_outside =
 observation.position.x <= robot_radius_m_ ||
 observation.position.y <= robot_radius_m_ ||
 observation.position.x >= world_size_.x - robot_radius_m_ ||
 observation.position.y >= world_size_.y - robot_radius_m_;
 const bool obstacle_overlap = std::any_of(
 static_obstacles_.begin(), static_obstacles_.end(), [&](const auto& obstacle) {
 const double nearest_x = std::clamp(
 observation.position.x, obstacle.min.x, obstacle.max.x);
 const double nearest_y = std::clamp(
 observation.position.y, obstacle.min.y, obstacle.max.y);
 return std::hypot(observation.position.x - nearest_x,
 observation.position.y - nearest_y) <= robot_radius_m_;
 });
out.static_collision = outside || footprint_outside || obstacle_overlap;
 }

 for (const auto& actor : observation.actors) {
 const double clearance = std::hypot(
 observation.position.x - actor.state.p.x,
 observation.position.y - actor.state.p.y) - actor.state.radius - robot_radius_m_;
 if (clearance < out.dynamic_clearance_m) {
 out.dynamic_clearance_m = clearance;
 out.nearest_actor_position = actor.state.p;
 }
 const bool contact = observation.backend == CollisionBackend::RigidBody
 ? actor.rigid_contact
 : clearance < 0.0;
 if (!contact)
 continue;
 if (actor.social)
 ++out.social_contacts;
 else
 out.dynamic_collision = true;
 }
 return out;
}

} // namespace mission
