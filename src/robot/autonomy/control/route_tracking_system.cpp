#include "robot/autonomy/control/route_tracking_system.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace sim {
namespace {

std::optional<Vec2> first_blocked_route_point(
 const std::vector<RoutePoint>& path, std::size_t begin, Vec2 current,
 const GridMap& map, double maximum_lookahead_m) {
 if (path.empty() || begin >= path.size() || !map.isFree(current.x, current.y))
 return std::nullopt;
 double inspected_m = 0.0;
 Vec2 from = current;
 const double sample_spacing_m = std::max(0.01, 0.5 * map.res());
 for (std::size_t index = begin;
 index < path.size() && inspected_m <= maximum_lookahead_m; ++index) {
 const Vec2 to = path[index].position;
 const double length_m = std::hypot(to.x - from.x, to.y - from.y);
 const int samples = std::max(1, static_cast<int>(std::ceil(
 length_m / sample_spacing_m)));
 for (int sample = 1; sample <= samples; ++sample) {
 const double alpha = static_cast<double>(sample) /
 static_cast<double>(samples);
 const Vec2 point{
 from.x + (to.x - from.x) * alpha,
 from.y + (to.y - from.y) * alpha,
 };
 if (!map.isFree(point.x, point.y))
 return point;
 }
 inspected_m += length_m;
 from = to;
 }
 return std::nullopt;
}

} // namespace

RouteTrackingSystem::RouteTrackingSystem(double kp, double ki, double kd, double output_limit, double integral_limit)
    : heading_pid_(std::make_unique<sim::PID>(kp, ki, kd, output_limit, integral_limit)) {}

void RouteTrackingSystem::reset() {
 guide_state_ = {};
 heading_pid_->reset();
 static_route_blocked_s_ = 0.0;
}

sim::GuideOut RouteTrackingSystem::follow_route(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate,
 double lookahead_m,
                                          double cruise_mps, double dt_s, const sim::GridMap& planning_map,
 const sim::ChassisCaps& chassis_caps) {
 if (chassis_caps.command_space == sim::ChassisCommandSpace::SteeringAngle) {
 return sim::ackermann_guide_step(path, guide_state_, estimate.position.x,
 estimate.position.y, estimate.heading_rad, chassis_caps.geometry.steering_wheel_base_m,
 chassis_caps.geometry.max_steer_rad, lookahead_m,
 cruise_mps, dt_s, planning_map);
 }
 return sim::guide_step(path, guide_state_, estimate.position.x, estimate.position.y, estimate.heading_rad,
                        *heading_pid_, lookahead_m, cruise_mps, dt_s, planning_map, chassis_caps);
}

sim::TravelDirection RouteTrackingSystem::travel_direction(const std::vector<sim::RoutePoint>& path) const {
 return sim::path_travel_direction(path, guide_state_.idx);
}

bool RouteTrackingSystem::should_replan(
 const std::vector<sim::RoutePoint>& path, const sim::EstimatedState& estimate,
 const sim::GridMap& local_map,
 const std::vector<sim::DynamicTrack>& dynamic_tracks, double dt_s) {
 const auto blocked = first_blocked_route_point(
 path, guide_state_.idx, estimate.position, local_map, 8.0);
 if (!blocked) {
 static_route_blocked_s_ = 0.0;
 return false;
 }
 const bool explained_by_dynamic_track = std::any_of(
 dynamic_tracks.begin(), dynamic_tracks.end(), [&](const DynamicTrack& track) {
 const double dynamic_envelope_m =
 track.state.radius + 2.0 * local_map.res();
 return std::hypot(track.state.p.x - blocked->x,
 track.state.p.y - blocked->y) <= dynamic_envelope_m;
 });
 if (explained_by_dynamic_track) {
 static_route_blocked_s_ = 0.0;
 return false;
 }
 static_route_blocked_s_ += std::max(0.0, dt_s);
 constexpr double kPersistentStaticBlockageS = 1.0;
 return static_route_blocked_s_ + 1e-9 >= kPersistentStaticBlockageS;
}

} // namespace sim
