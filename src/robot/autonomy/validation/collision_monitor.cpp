#include "robot/autonomy/validation/collision_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sim::autonomy {
namespace {

constexpr double kLidarSafetyInvalidClearanceM = -1.0;

bool lidar_scan_is_valid(const Scan& scan) {
 if (!(scan.n_rays > 0) ||
 scan.ranges.size() != static_cast<std::size_t>(scan.n_rays) ||
 scan.hit.size() != static_cast<std::size_t>(scan.n_rays) ||
 !(scan.max_range > 0.0) ||
 !std::isfinite(scan.angle_min_rad) ||
 !std::isfinite(scan.angle_increment_rad) ||
 std::abs(scan.angle_increment_rad) < 1e-9) {
 return false;
 }
 return true;
}

bool segment_intersects_box(Vec2 from, Vec2 to,
 Vec2 minimum, Vec2 maximum) {
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

Vec2 box_local_point(const CollisionBox& box, Vec2 point) {
 const Vec2 center{(box.min.x + box.max.x) * 0.5,
 (box.min.y + box.max.y) * 0.5};
 const double dx = point.x - center.x;
 const double dy = point.y - center.y;
 const double cosine = std::cos(box.yaw_rad);
 const double sine = std::sin(box.yaw_rad);
 return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

Vec2 box_half_extents(const CollisionBox& box) {
 return {(box.max.x - box.min.x) * 0.5, (box.max.y - box.min.y) * 0.5};
}

} // namespace

CollisionMonitor::CollisionMonitor(Vec2 world_size, double robot_radius_m,
 std::vector<CollisionBox> static_obstacles)
 : world_size_(world_size),
 robot_radius_m_(robot_radius_m),
 static_obstacles_(std::move(static_obstacles)) {
 if (!(world_size_.x > 0.0) || !(world_size_.y > 0.0) || robot_radius_m_ < 0.0)
 throw std::invalid_argument("collision monitor geometry must be positive");
}

void CollisionMonitor::reset() {
}

bool CollisionMonitor::static_motion_free(Vec2 from, Vec2 to,
 double execution_margin_m) const {
 const auto motion_free_at_radius = [&](double radius_m) {
 const auto inside_world = [&](Vec2 point) {
 return point.x > radius_m && point.y > radius_m &&
 point.x < world_size_.x - radius_m &&
 point.y < world_size_.y - radius_m;
 };
 if (!inside_world(from) || !inside_world(to))
 return false;
 for (const auto& obstacle : static_obstacles_) {
 const auto local_from = box_local_point(obstacle, from);
 const auto local_to = box_local_point(obstacle, to);
 const auto half = box_half_extents(obstacle);
 if (segment_intersects_box(local_from, local_to,
 {-half.x - radius_m, -half.y - radius_m},
 {half.x + radius_m, half.y + radius_m}))
 return false;
 }
 return true;
 };

 if (!motion_free_at_radius(robot_radius_m_))
 return false;
 const double margin = std::max(0.0, execution_margin_m);
 if (margin <= 0.0)
 return true;
 const double initial_clearance = static_clearance(from);
 // The strict band applies only while the body is already clear of the whole
 // reserve. At the boundary of the reserve, admitting only non-worsening
 // segments is what lets a braking body stop instead of rejecting every
 // candidate and continuing to coast under its previous command.
 if (initial_clearance > margin + 1e-9)
 return motion_free_at_radius(robot_radius_m_ + margin);

 // If tracking/localization has already put the centre inside the execution
 // reserve, preserve exact physical collision checking and admit only
 // non-worsening recovery segments until the reserve has been regained.
 const Vec2 midpoint{(from.x + to.x) * 0.5, (from.y + to.y) * 0.5};
 return static_clearance(midpoint) + 1e-9 >= initial_clearance &&
 static_clearance(to) + 1e-9 >= initial_clearance;
}

double CollisionMonitor::static_clearance(Vec2 position) const {
 double minimum = std::min({position.x, position.y,
 world_size_.x - position.x, world_size_.y - position.y}) - robot_radius_m_;
 for (const auto& obstacle : static_obstacles_) {
 const auto local = box_local_point(obstacle, position);
 const auto half = box_half_extents(obstacle);
 const double dx = std::max(std::abs(local.x) - half.x, 0.0);
 const double dy = std::max(std::abs(local.y) - half.y, 0.0);
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
 {
 // A ROBOT decision uses its own footprint and deployment geometry. This is
 // intentionally independent of the simulator's physical contact result.
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
 const auto local = box_local_point(obstacle, observation.position);
 const auto half = box_half_extents(obstacle);
 const double dx = std::max(std::abs(local.x) - half.x, 0.0);
 const double dy = std::max(std::abs(local.y) - half.y, 0.0);
 return std::hypot(dx, dy) <= robot_radius_m_;
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
 const bool contact = clearance < 0.0;
 if (!contact)
 continue;
 if (actor.social)
 ++out.social_contacts;
 else
 out.dynamic_collision = true;
 }
 return out;
}

LidarSafetyAssessment CollisionMonitor::assess_lidar(
 const LidarSafetyObservation& observation,
 const LidarSafetyConfig& config) const {
 LidarSafetyAssessment out;
 if (observation.scan == nullptr) {
 out.stop_required = true;
 out.measurement_valid = false;
 out.forward_clearance_m = kLidarSafetyInvalidClearanceM;
 return out;
 }
 const Scan& scan = *observation.scan;
 out.measurement_valid = observation.measurement_valid &&
 observation.measurement_fresh && lidar_scan_is_valid(scan);
 out.forward_clearance_m = kLidarSafetyInvalidClearanceM;
 if (!out.measurement_valid) {
 out.stop_required = true;
 return out;
 }

 const double speed_mps = std::max(0.0, observation.forward_speed_mps);
 const double braking_distance_m =
 speed_mps * std::max(0.0, config.reaction_time_s) +
 speed_mps * speed_mps /
 (2.0 * std::max(1e-6, config.braking_decel_mps2));
 out.braking_distance_m = braking_distance_m;
 // Body relative motion over one scan. The scalar model is conservative for
 // a rotating chassis: it enlarges the allowed travel instead of hiding a
 // return that moved because of the scan's acquisition delay.
 const double scan_clearance_m =
 speed_mps * std::max(0.0, config.range_growth_m_per_mps) *
 (std::max(0.0, scan.scan_period_s) +
 std::max(0.0, config.reaction_time_s));
 out.required_forward_clearance_m =
 braking_distance_m + std::max(0.0, config.sensor_standoff_m) +
 scan_clearance_m;

 const double half_angle_rad = std::clamp(
 config.half_angle_rad, 0.0, kPi * 0.5);
 double nearest_body_clearance_m = std::numeric_limits<double>::infinity();
 for (int ray = 0; ray < scan.n_rays; ++ray) {
 const auto index = static_cast<std::size_t>(ray);
 if (!scan.hit[index] || !std::isfinite(scan.ranges[index])) {
 continue;
 }
 if (!scan.ray_gravity_heights_m.empty() &&
 index < scan.ray_gravity_heights_m.size() &&
 std::isfinite(scan.ray_gravity_heights_m[index]) &&
 scan.ray_gravity_heights_m[index] < config.ground_return_min_height_m) {
 continue;
 }
 const double range_m = scan.ranges[index];
 if (!(range_m > 0.0) || range_m >= scan.max_range) {
 continue;
 }
 const double angle_in_sensor = scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(ray);
 const double angle_in_body = observation.yaw_in_base_rad + angle_in_sensor;
 if (std::abs(std::remainder(angle_in_body, 2.0 * kPi)) >
 half_angle_rad) {
 continue;
 }
 const Vec2 local_point{
 observation.position_in_base.x + range_m * std::cos(angle_in_body),
 observation.position_in_base.y + range_m * std::sin(angle_in_body),
 };
 if (local_point.x <= config.zero_range_m) {
 // A return behind the sensor origin cannot be reached by a forward track.
 continue;
 }
 // The sensor is mounted on the chassis; a range is not yet body clearance.
 // Use the circular footprint in the same body frame that owns the mount so
 // the threshold is independent of where the LiDAR sits on the deck.
 const double body_clearance_m =
 std::hypot(local_point.x, local_point.y) - robot_radius_m_;
 if (body_clearance_m < nearest_body_clearance_m) {
 nearest_body_clearance_m = body_clearance_m;
 out.nearest_target_in_base = local_point;
 }
 }
 out.target_in_path = std::isfinite(nearest_body_clearance_m);
 out.forward_clearance_m = out.target_in_path
 ? nearest_body_clearance_m
 : std::max(0.0, scan.max_range -
 std::hypot(observation.position_in_base.x,
 observation.position_in_base.y) -
 robot_radius_m_);
 out.stop_required =
 out.target_in_path &&
 out.forward_clearance_m < out.required_forward_clearance_m;
 return out;
}

} // namespace sim::autonomy
