#include "robot/autonomy/mapping/raster_traversal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim {
namespace {

constexpr double kGravityMps2 = 9.81;
constexpr double kMaximumContinuousGrade = 0.85;
constexpr double kMinimumSecantDistanceM = 0.05;

struct GridPoint {
 double local_x{0.0};
 double local_y{0.0};
 double cell_x{0.0};
 double cell_y{0.0};
};

std::optional<GridPoint> local_point(const runtime::MapGrid& grid, Vec2 world) {
 if (grid.width == 0 || grid.height == 0 || !(grid.resolution_m > 0.0) ||
 !std::isfinite(world.x) || !std::isfinite(world.y)) {
 return std::nullopt;
 }
 const double dx = world.x - grid.origin_m.x;
 const double dy = world.y - grid.origin_m.y;
 const double cosine = std::cos(grid.origin_yaw_rad);
 const double sine = std::sin(grid.origin_yaw_rad);
 const double local_x = cosine * dx + sine * dy;
 const double local_y = -sine * dx + cosine * dy;
 const double cell_x = local_x / grid.resolution_m - 0.5;
 const double cell_y = local_y / grid.resolution_m - 0.5;
 if (cell_x < -0.5 || cell_y < -0.5 ||
 cell_x > static_cast<double>(grid.width) - 0.5 ||
 cell_y > static_cast<double>(grid.height) - 0.5) {
 return std::nullopt;
 }
 return GridPoint{local_x, local_y, cell_x, cell_y};
}

template <typename T>
double bilinear(const runtime::MapGrid& grid, const std::vector<T>& layer,
 GridPoint point, double fallback) {
 if (layer.size() != static_cast<std::size_t>(grid.width) * grid.height)
 return fallback;
 const auto clamp_index = [](int value, std::uint32_t limit) {
 return std::clamp(value, 0, static_cast<int>(limit) - 1);
 };
 const int x0 = clamp_index(static_cast<int>(std::floor(point.cell_x)), grid.width);
 const int y0 = clamp_index(static_cast<int>(std::floor(point.cell_y)), grid.height);
 const int x1 = clamp_index(x0 + 1, grid.width);
 const int y1 = clamp_index(y0 + 1, grid.height);
 const double tx = std::clamp(point.cell_x - std::floor(point.cell_x), 0.0, 1.0);
 const double ty = std::clamp(point.cell_y - std::floor(point.cell_y), 0.0, 1.0);
 double weighted = 0.0;
 double weight_sum = 0.0;
 const auto add = [&](int x, int y, double weight) {
 const T value = layer[static_cast<std::size_t>(y) * grid.width + x];
 if (!std::isfinite(static_cast<double>(value)))
 return;
 weighted += static_cast<double>(value) * weight;
 weight_sum += weight;
 };
 add(x0, y0, (1.0 - tx) * (1.0 - ty));
 add(x1, y0, tx * (1.0 - ty));
 add(x0, y1, (1.0 - tx) * ty);
 add(x1, y1, tx * ty);
 return weight_sum > 0.0 ? weighted / weight_sum : fallback;
}

} // namespace

RasterTraversalModel::RasterTraversalModel(RasterTraversalConfig config)
 : config_(std::move(config)) {}

void RasterTraversalModel::set_config(RasterTraversalConfig config) {
 config_ = std::move(config);
}

void RasterTraversalModel::bind(std::shared_ptr<const runtime::MapSnapshot> map) {
 map_ = std::move(map);
 roughness_beliefs_.clear();
 latest_observation_time_s_ = 0.0;
 have_observation_time_ = false;
}

void RasterTraversalModel::observe_motion(
 const EstimatedState& estimate, const ImuObservation& imu, double dt_s) {
 if (!ready() || !(dt_s > 0.0))
 return;
 // Sensor time advances even while stationary. Keeping the clock current lets
 // old per-cell evidence decay instead of remaining "fresh" indefinitely.
 const double stamp = std::isfinite(imu.header.stamp_s) && imu.header.stamp_s > 0.0
 ? imu.header.stamp_s : estimate.timestamp_s;
 if (std::isfinite(stamp)) {
 if (!have_observation_time_ || stamp > latest_observation_time_s_)
 latest_observation_time_s_ = stamp;
 have_observation_time_ = true;
 }
 if (!std::isfinite(estimate.linear_velocity_mps) ||
 std::abs(estimate.linear_velocity_mps) < config_.roughness_observation_min_speed_mps ||
 !std::isfinite(imu.up_acceleration_mps2) ||
 !(config_.roughness_belief_time_constant_s > 0.0) ||
 !(config_.roughness_belief_full_confidence_s > 0.0) ||
 !(config_.roughness_observation_ceiling_m > 0.0)) {
 return;
 }
 const auto cell = runtime::world_to_grid(*map_->grid, estimate.position);
 if (!cell)
 return;

 // The accelerometer measures specific force. Remove the gravity projection
 // from the ROBOT attitude estimate; the remaining vertical excitation is
 // caused by motion over relief, attitude change and sensor noise. A small
 // deadband keeps ordinary noise from turning a smooth floor into a rough
 // surface while the estimator gathers confidence.
 const double roll = estimate.attitude_valid ? estimate.roll_rad : 0.0;
 const double pitch = estimate.attitude_valid ? estimate.pitch_rad : 0.0;
 const double gravity_expected = 9.81 * std::cos(roll) * std::cos(pitch);
 const double residual = std::max(
 0.0, std::abs(imu.up_acceleration_mps2 - gravity_expected) -
 std::max(config_.roughness_observation_deadband_mps2,
 3.0 * std::abs(imu.acceleration_noise_sigma_mps2)));
 const double support_base = std::max(0.05, config_.roughness_support_base_m);
 const double wavenumber = 2.0 * kPi / support_base;
 const double speed_sq = std::max(0.04, estimate.linear_velocity_mps *
 estimate.linear_velocity_mps);
 const double measured = std::clamp(
 residual / std::max(1e-9, wavenumber * wavenumber * speed_sq),
 0.0, config_.roughness_observation_ceiling_m);

 const std::uint64_t key =
 (static_cast<std::uint64_t>(cell->y) << 32) | cell->x;
 auto [entry, inserted] = roughness_beliefs_.try_emplace(key);
 auto& belief = entry->second;
 if (inserted) {
 double confidence = 0.0;
 const double prior = prior_roughness(estimate.position, &confidence);
 belief.roughness_m = confidence > 0.0 ? prior : config_.default_roughness_m;
 }
 double alpha = 1.0 - std::exp(-dt_s / config_.roughness_belief_time_constant_s);
 // Rising relief needs to reach the safety limit quickly; returning to a
 // smooth belief is slower so one quiet sample cannot erase a real bump.
 if (measured > belief.roughness_m)
 alpha = std::max(alpha, 1.0 - std::exp(-dt_s /
 (0.35 * config_.roughness_belief_time_constant_s)));
 belief.roughness_m = (1.0 - alpha) * belief.roughness_m + alpha * measured;
 belief.confidence = std::min(1.0,
 belief.confidence + dt_s / config_.roughness_belief_full_confidence_s);
 belief.observed_at_s = stamp;
}

double RasterTraversalModel::local_elevation(double local_x, double local_y) const {
 if (!ready() || map_->grid->elevation_m.empty())
 return 0.0;
 const auto point = local_point(*map_->grid, {
 map_->grid->origin_m.x + std::cos(map_->grid->origin_yaw_rad) * local_x -
 std::sin(map_->grid->origin_yaw_rad) * local_y,
 map_->grid->origin_m.y + std::sin(map_->grid->origin_yaw_rad) * local_x +
 std::cos(map_->grid->origin_yaw_rad) * local_y,
 });
 return point ? bilinear(*map_->grid, map_->grid->elevation_m, *point, 0.0) : 0.0;
}

double RasterTraversalModel::local_roughness(double local_x, double local_y) const {
 if (!ready())
 return config_.default_roughness_m;
 const Vec2 position{
 map_->grid->origin_m.x + std::cos(map_->grid->origin_yaw_rad) * local_x -
 std::sin(map_->grid->origin_yaw_rad) * local_y,
 map_->grid->origin_m.y + std::sin(map_->grid->origin_yaw_rad) * local_x +
 std::cos(map_->grid->origin_yaw_rad) * local_y,
 };
 double prior = config_.default_roughness_m;
 double prior_confidence = 0.0;
 prior = prior_roughness(position, &prior_confidence);
 double online = 0.0;
 double online_confidence = 0.0;
 if (const auto cell = runtime::world_to_grid(*map_->grid, position)) {
 const auto it = roughness_beliefs_.find(
 (static_cast<std::uint64_t>(cell->y) << 32) | cell->x);
 if (it != roughness_beliefs_.end()) {
 online = std::clamp(it->second.roughness_m, 0.0,
 config_.roughness_observation_ceiling_m);
 const double age = have_observation_time_
 ? std::max(0.0, latest_observation_time_s_ - it->second.observed_at_s)
 : config_.roughness_belief_max_age_s;
 const double stale_decay = age <= config_.roughness_belief_max_age_s
 ? 1.0 : std::exp(-(age - config_.roughness_belief_max_age_s) /
 std::max(0.1, config_.roughness_belief_time_constant_s));
 online_confidence = it->second.confidence * stale_decay;
 }
 }
 if (online_confidence <= 0.0)
 return prior_confidence > 0.0 ? prior : config_.default_roughness_m;
 if (prior_confidence <= 0.0) {
 const double online_weight = std::clamp(online_confidence, 0.0, 1.0);
 return online_weight * online +
 (1.0 - online_weight) * config_.default_roughness_m;
 }
 const double online_weight = online_confidence /
 std::max(1e-9, online_confidence + prior_confidence);
 return online_weight * online + (1.0 - online_weight) * prior;
}

double RasterTraversalModel::prior_roughness(
 Vec2 position, double* confidence) const {
 if (confidence)
 *confidence = 0.0;
 if (!ready() || map_->grid->roughness_m.empty() ||
 map_->grid->roughness_confidence.size() != map_->grid->roughness_m.size())
 return config_.default_roughness_m;
 const auto cell = runtime::world_to_grid(*map_->grid, position);
 if (!cell)
 return config_.default_roughness_m;
 const auto index = static_cast<std::size_t>(cell->y) * map_->grid->width + cell->x;
 const double value = map_->grid->roughness_m[index];
 const double value_confidence = map_->grid->roughness_confidence[index];
 if (!std::isfinite(value) || !std::isfinite(value_confidence) || value_confidence <= 0.0)
 return config_.default_roughness_m;
 if (confidence)
 *confidence = std::clamp(value_confidence, 0.0, 1.0);
 return std::max(0.0, value);
}

TerrainSample RasterTraversalModel::sample(Vec2 position) const {
 if (!ready())
 return {0.0, 0.0, 0.0, config_.default_roughness_m};
 const auto point = local_point(*map_->grid, position);
 if (!point)
 return {0.0, 0.0, 0.0, config_.default_roughness_m};
 const double step = std::max(0.05, 0.5 * map_->grid->resolution_m);
 const double height = local_elevation(point->local_x, point->local_y);
 const double grade_x = map_->grid->elevation_m.empty() ? 0.0 :
 (local_elevation(point->local_x + step, point->local_y) -
 local_elevation(point->local_x - step, point->local_y)) /
 (2.0 * step);
 const double grade_y = map_->grid->elevation_m.empty() ? 0.0 :
 (local_elevation(point->local_x, point->local_y + step) -
 local_elevation(point->local_x, point->local_y - step)) /
 (2.0 * step);
 const double cosine = std::cos(map_->grid->origin_yaw_rad);
 const double sine = std::sin(map_->grid->origin_yaw_rad);
 return {
 height,
 cosine * grade_x - sine * grade_y,
 sine * grade_x + cosine * grade_y,
 local_roughness(point->local_x, point->local_y),
 };
}

double RasterTraversalModel::friction_prior(Vec2 position) const {
 if (!ready() || map_->grid->surface_class_properties.empty())
 return config_.default_surface_friction_mu;
 const auto cell = runtime::world_to_grid(*map_->grid, position);
 if (!cell)
 return config_.default_surface_friction_mu;
 const auto index = static_cast<std::size_t>(cell->y) * map_->grid->width + cell->x;
 const auto class_index = map_->grid->surface_class.empty()
 ? 0U : map_->grid->surface_class[index];
 if (class_index >= map_->grid->surface_class_properties.size())
 return config_.default_surface_friction_mu;
 return map_->grid->surface_class_properties[class_index].nominal_friction_mu;
}

double RasterTraversalModel::rolling_prior(Vec2 position) const {
 if (!ready() || map_->grid->surface_class_properties.empty())
 return 0.0;
 const auto cell = runtime::world_to_grid(*map_->grid, position);
 if (!cell)
 return 0.0;
 const auto index = static_cast<std::size_t>(cell->y) * map_->grid->width + cell->x;
 const auto class_index = map_->grid->surface_class.empty()
 ? 0U : map_->grid->surface_class[index];
 return class_index < map_->grid->surface_class_properties.size()
 ? map_->grid->surface_class_properties[class_index].rolling_resistance : 0.0;
}

TraversalContactEstimate RasterTraversalModel::contact(Vec2 position) const {
 const double surface_mu = std::max(0.0, friction_prior(position));
 const double effective_mu = config_.tire_friction_mu * surface_mu /
 std::max(0.05, config_.reference_surface_friction_mu);
 const double grip_fraction = std::clamp(surface_mu /
 std::max(0.05, config_.reference_surface_friction_mu), 0.0, 1.0);
 return {
 effective_mu,
 config_.tire_rolling_resistance + std::max(0.0, rolling_prior(position)),
 std::min(config_.max_wheel_accel_mps2,
 config_.brake_decel_mps2 * (0.5 + 0.5 * grip_fraction)),
 config_.tire_contact_time_constant_s,
 config_.tire_contact_damping_ratio,
 };
}

double RasterTraversalModel::traversal_cost(Vec2 from, Vec2 to,
 TerrainCapability capability) const {
 const double dx = to.x - from.x;
 const double dy = to.y - from.y;
 const double distance = std::hypot(dx, dy);
 if (distance < 1e-12)
 return 1.0;
 const auto from_terrain = sample(from);
 const auto to_terrain = sample(to);
 const double segment_grade = distance < kMinimumSecantDistanceM
 ? longitudinal_grade(from_terrain, std::atan2(dy, dx))
 : (to_terrain.height_m - from_terrain.height_m) / distance;
 if (std::abs(segment_grade) > kMaximumContinuousGrade)
 return std::numeric_limits<double>::infinity();
 const double heading = std::atan2(dy, dx);
 const TerrainSample terrain = sample({(from.x + to.x) * 0.5, (from.y + to.y) * 0.5});
 const double forward_grade = std::max(longitudinal_grade(terrain, heading), segment_grade);
 const double cross_grade = lateral_grade(terrain, heading);
 if (std::atan(std::abs(cross_grade)) > capability.max_cross_slope_rad)
 return std::numeric_limits<double>::infinity();
 capability.friction_mu = contact({(from.x + to.x) * 0.5,
 (from.y + to.y) * 0.5}).friction_mu;
 const double slope_rad = std::atan(forward_grade);
 const double required_force = capability.mass_kg * kGravityMps2 *
 std::max(0.0, std::sin(slope_rad));
 const double traction_force = capability.mass_kg * kGravityMps2 *
 std::max(0.0, capability.friction_mu) * std::cos(slope_rad);
 const double usable_force = std::min(capability.available_drive_force_N, traction_force);
 if (required_force > usable_force * 1.25 + 1e-9)
 return std::numeric_limits<double>::infinity();
 return 1.0 + capability.uphill_cost_weight * std::max(0.0, forward_grade) +
 capability.cross_slope_cost_weight * std::abs(cross_grade) +
 capability.roughness_cost_weight * std::max(0.0, terrain.roughness);
}

} // namespace sim
