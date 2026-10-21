#include "robot/autonomy/mapping/online_slam.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::Scan scan_with_wall(double stamp_s, std::uint64_t sequence, double wall_x) {
 sim::Scan scan;
 scan.n_rays = 180;
 scan.max_range = 8.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.scan_period_s = 0.18;
 scan.time_increment_s = scan.scan_period_s / scan.n_rays;
 scan.header = {stamp_s, sequence, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.lidar"};
 for (int i = 0; i < scan.n_rays; ++i) {
 const double angle = scan.angle_min_rad + scan.angle_increment_rad * i;
 const double cosine = std::cos(angle);
 double range = scan.max_range;
 if (cosine > 1e-6) {
 const double candidate = wall_x / cosine;
 if (candidate > 0.05 && candidate < scan.max_range)
 range = candidate;
 }
 scan.ranges.push_back(range);
 scan.hit.push_back(range < scan.max_range ? 1 : 0);
 scan.intensities.push_back(1.0);
 }
 return scan;
}

sim::ObservationFrame frame(sim::Scan scan) {
 sim::ObservationFrame observations;
 observations.time_s = scan.header.stamp_s;
 observations.lidar = std::move(scan);
 return observations;
}

// A corridor wall at wall_x with an optional round post in front of it. The
// post is a transient return: it must not survive in the published submap
// after the scans stop seeing it.
sim::Scan scan_with_post(double stamp_s, std::uint64_t sequence, double wall_x,
 double post_x, bool post_present) {
 sim::Scan scan;
 scan.n_rays = 180;
 scan.max_range = 8.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.scan_period_s = 0.1;
 scan.time_increment_s = scan.scan_period_s / scan.n_rays;
 scan.header = {stamp_s, sequence, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.transient-lidar"};
 for (int i = 0; i < scan.n_rays; ++i) {
 const double angle = scan.angle_min_rad + scan.angle_increment_rad * i;
 const double cosine = std::cos(angle);
 double range = scan.max_range;
 if (cosine > 1e-6) {
 const double wall = wall_x / cosine;
 if (wall > 0.05 && wall < scan.max_range)
 range = wall;
 }
 if (post_present && cosine > 1e-6) {
 const double post = post_x / cosine;
 // 0.10 m half-width post, expressed as the lateral offset of the
 // intersection point with the post's plane.
 const double lateral = std::abs(post * std::sin(angle));
 if (lateral <= 0.10 && post > 0.05 && post < range)
 range = post;
 }
 scan.ranges.push_back(range);
 scan.hit.push_back(range < scan.max_range ? 1 : 0);
 scan.intensities.push_back(1.0);
 }
 return scan;
}

} // namespace

int main() {
 sim::autonomy::OnlineSlamConfig config;
 config.width = 160;
 config.height = 160;
 config.resolution_m = 0.10;
 config.maximum_range_m = 8.0;
 config.translation_search_m = 0.20;
 config.translation_step_m = 0.10;
 config.heading_search_rad = 0.10;
 config.heading_step_rad = 0.10;
 config.maximum_matching_returns = 32;
 sim::autonomy::OnlineSlamMapBuilder builder(config);

 sim::EstimatedState prior;
 prior.position = {10.0, -4.0};
 prior.heading_rad = 0.0;
 prior.linear_velocity_mps = 0.15;
 prior.angular_velocity_radps = 0.08;
 prior.confidence = 1.0;
 const auto first_scan = scan_with_wall(1.0, 1, 5.0);
 const auto first = builder.update(frame(first_scan), prior,
 sim::runtime::TimePoint::from_seconds(1.0));
 CHECK(first.state == sim::autonomy::MappingState::Mapping);
 CHECK(first.published_map);
 CHECK(first.published_map->provenance.source_kind == "slam");
 CHECK(first.map_revision == 1);
 CHECK(first.pose.position.x == 0.0);
 CHECK(first.pose.position.y == 0.0);
 CHECK(first.published_map->grid);
 bool saw_occupied = false;
 bool saw_free = false;
 for (const auto cell : first.published_map->grid->occupancy) {
 saw_occupied = saw_occupied || cell >= 50;
 saw_free = saw_free || cell == 0;
 }
 CHECK(saw_occupied);
 CHECK(saw_free);

 prior.position = {10.25, -4.0};
 const auto second_scan = scan_with_wall(2.0, 2, 4.75);
 const auto second = builder.update(frame(second_scan), prior,
 sim::runtime::TimePoint::from_seconds(2.0));
 CHECK(second.state == sim::autonomy::MappingState::Mapping);
 CHECK(second.map_revision == 2);
 CHECK(second.published_map->revision == 2);
 CHECK(second.pose.header.source_id == "robot.online-slam");
 CHECK(second.pose.position.x >= 0.15 && second.pose.position.x <= 0.35);

 // A pose-graph correction is fed back through the same provider boundary as
 // a normal update. The next prior must remain in the corrected map frame and
 // the raster must continue publishing instead of silently reverting to the
 // pre-correction frame.
 builder.apply_map_correction({0.20, -0.10}, 0.05);
 const auto corrected = builder.update(
 frame(scan_with_wall(2.1, 3, 4.75)), prior,
 sim::runtime::TimePoint::from_seconds(2.1));
 CHECK(corrected.state == sim::autonomy::MappingState::Mapping);
 CHECK(corrected.map_revision == 3);
 CHECK(corrected.published_map && corrected.published_map->grid);
 CHECK(std::isfinite(corrected.pose.position.x));
 CHECK(std::isfinite(corrected.pose.position.y));

 const auto stale = builder.update(frame(second_scan), prior,
 sim::runtime::TimePoint::from_seconds(1.5));
 CHECK(stale.state == sim::autonomy::MappingState::Degraded);
 CHECK(stale.diagnostic == "stale or invalid SLAM state");

 sim::ObservationFrame empty;
 empty.time_s = 3.0;
 const auto missing = builder.update(empty, prior,
 sim::runtime::TimePoint::from_seconds(3.0));
 CHECK(missing.state == sim::autonomy::MappingState::Degraded);
 CHECK(missing.diagnostic == "missing or stale LiDAR observation");

 // A multi-beam provider is reduced to the planar layer only after the
 // ROBOT-side height and timing contract is applied.  Ground returns must not
 // become obstacles, while the selected obstacle returns retain their firing
 // offsets for deskew.
 sim::autonomy::OnlineSlamConfig cloud_config = config;
 cloud_config.sensor_height_m = 0.30;
 cloud_config.ground_rejection_band_m = 0.06;
 cloud_config.translation_search_m = 0.0;
 cloud_config.heading_search_rad = 0.0;
 sim::autonomy::OnlineSlamMapBuilder cloud_builder(cloud_config);
 sim::LidarPointCloud cloud;
 cloud.rings = 16;
 cloud.azimuth_samples = 360;
 cloud.max_range_m = 8.0;
 cloud.scan_period_s = 0.10;
 cloud.header = {4.0, 4, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.16line"};
 cloud.points = {
 {3.0, -0.15, 0.0, 1.0, -0.10, 0, 0},
 {3.0, 0.15, 0.0, 1.0, -0.05, 1, 0},
 {1.0, 0.0, -0.30, 1.0, -0.025, 2, 0}, // ground return: reject
 {2.0, 0.0, 0.35, 1.0, 0.0, 3, 0},
 };
 sim::ObservationFrame cloud_frame;
 cloud_frame.time_s = 4.0;
 cloud_frame.lidar_point_cloud = cloud;
 prior.position = {0.0, 0.0};
 prior.heading_rad = 0.0;
 const auto cloud_update = cloud_builder.update(
 cloud_frame, prior, sim::runtime::TimePoint::from_seconds(4.0));
 CHECK(cloud_update.state == sim::autonomy::MappingState::Mapping);
 CHECK(cloud_update.published_map);
 bool cloud_obstacle = false;
 bool cloud_elevation = false;
 for (const auto cell : cloud_update.published_map->grid->occupancy)
 cloud_obstacle = cloud_obstacle || cell >= 50;
 CHECK(cloud_obstacle);
 CHECK(cloud_update.published_map->grid->elevation_m.size() ==
 cloud_update.published_map->grid->occupancy.size());
 for (const auto elevation : cloud_update.published_map->grid->elevation_m)
 cloud_elevation = cloud_elevation || std::isfinite(elevation);
 CHECK(cloud_elevation);

 // Ground rejection is gravity-referenced when ROBOT has a valid attitude.
 // With a 0.2 rad nose-up pitch and 0.15 rad roll, this return is on level
 // ground even though
 // its sensor-frame z value alone would look like a raised obstacle. The
 // elevated return must remain an occupied endpoint and publish its world
 // height using the same gravity transform.
 sim::autonomy::OnlineSlamMapBuilder inclined_builder(cloud_config);
 sim::EstimatedState inclined_prior = prior;
 inclined_prior.attitude_valid = true;
 inclined_prior.pitch_rad = 0.20;
 inclined_prior.roll_rad = 0.15;
 sim::LidarPointCloud inclined_cloud = cloud;
 inclined_cloud.header.stamp_s = 4.2;
 inclined_cloud.header.sequence = 42;
 inclined_cloud.points = {
 {1.0, 0.0, -0.105, 1.0, -0.02, 0, 0}, // gravity height ~= -0.30 m: ground
 {2.0, 0.0, 0.50, 1.0, -0.01, 1, 0}, // elevated obstacle
 };
 sim::ObservationFrame inclined_frame;
 inclined_frame.time_s = 4.2;
 inclined_frame.lidar_point_cloud = inclined_cloud;
 const auto inclined_update = inclined_builder.update(
 inclined_frame, inclined_prior, sim::runtime::TimePoint::from_seconds(4.2));
 CHECK(inclined_update.state == sim::autonomy::MappingState::Mapping);
 bool inclined_obstacle = false;
 bool inclined_height = false;
 for (std::size_t index = 0; index < inclined_update.published_map->grid->occupancy.size(); ++index) {
 if (inclined_update.published_map->grid->occupancy[index] >= 50) {
 inclined_obstacle = true;
 if (index < inclined_update.published_map->grid->elevation_m.size() &&
 std::isfinite(inclined_update.published_map->grid->elevation_m[index]) &&
 std::abs(inclined_update.published_map->grid->elevation_m[index] - 0.39F) < 0.08F)
 inclined_height = true;
 }
 }
 CHECK(inclined_obstacle);
 CHECK(inclined_height);

 // Multi-frame point-cloud replay: a stable elevated return should converge
 // to one uncertain-but-valid terrain height, while a return that disappears
 // is cleared by subsequent free-space rays. This exercises the complete
 // ROBOT-side projection, occupancy evidence and elevation publication path.
 const auto replay_cell = [&](const sim::runtime::MapSnapshot& snapshot,
 sim::Vec2 point) {
 const auto cell = sim::runtime::world_to_grid(*snapshot.grid, point);
 if (!cell)
 return std::size_t{0};
 return static_cast<std::size_t>(cell->y) * snapshot.grid->width + cell->x;
 };
 sim::LidarPointCloud replay_cloud = cloud;
 replay_cloud.points = {
 {2.0, -0.02, 0.35, 1.0, -0.04, 3, 0}, // stable elevated return
 {1.0, -1.02, 0.30, 1.0, -0.03, 4, 0}, // transient return
 };
 std::shared_ptr<const sim::runtime::MapSnapshot> replay_map;
 for (int frame_index = 0; frame_index < 4; ++frame_index) {
 replay_cloud.header.stamp_s = 4.1 + 0.1 * frame_index;
 replay_cloud.header.sequence = 5 + frame_index;
 sim::ObservationFrame replay_frame;
 replay_frame.time_s = replay_cloud.header.stamp_s;
 replay_frame.lidar_point_cloud = replay_cloud;
 replay_map = cloud_builder.update(replay_frame, prior,
 sim::runtime::TimePoint::from_seconds(replay_cloud.header.stamp_s)).published_map;
 }
 CHECK(replay_map && replay_map->grid);
 const auto stable_index = replay_cell(*replay_map, {2.0, -0.02});
 const auto transient_index = replay_cell(*replay_map, {1.0, -1.02});
 bool stable_found = false;
 bool transient_found = false;
 for (int dy = -1; dy <= 1; ++dy) {
 for (int dx = -1; dx <= 1; ++dx) {
 const auto stable = stable_index + static_cast<std::ptrdiff_t>(dy) *
 replay_map->grid->width + dx;
 const auto transient = transient_index + static_cast<std::ptrdiff_t>(dy) *
 replay_map->grid->width + dx;
 if (replay_map->grid->occupancy[stable] >= 50 &&
 std::isfinite(replay_map->grid->elevation_m[stable]) &&
 std::abs(replay_map->grid->elevation_m[stable] - 0.65F) < 0.08F &&
 std::isfinite(replay_map->grid->elevation_sigma_m[stable]) &&
 replay_map->grid->elevation_sigma_m[stable] < 0.50F)
 stable_found = true;
 if (replay_map->grid->occupancy[transient] >= 50)
 transient_found = true;
 }
 }
 CHECK(stable_found);
 CHECK(transient_found);
 replay_cloud.points.resize(1); // remove only the transient return
 for (int frame_index = 0; frame_index < 20; ++frame_index) {
 replay_cloud.header.stamp_s = 4.5 + 0.1 * frame_index;
 replay_cloud.header.sequence = 9 + frame_index;
 sim::ObservationFrame replay_frame;
 replay_frame.time_s = replay_cloud.header.stamp_s;
 replay_frame.lidar_point_cloud = replay_cloud;
 replay_map = cloud_builder.update(replay_frame, prior,
 sim::runtime::TimePoint::from_seconds(replay_cloud.header.stamp_s)).published_map;
 }
 CHECK(replay_map && replay_map->grid);
 bool transient_cleared = true;
 bool stable_retained = false;
 for (int dy = -1; dy <= 1; ++dy) {
 for (int dx = -1; dx <= 1; ++dx) {
 const auto transient = transient_index + static_cast<std::ptrdiff_t>(dy) *
 replay_map->grid->width + dx;
 const auto stable = stable_index + static_cast<std::ptrdiff_t>(dy) *
 replay_map->grid->width + dx;
 transient_cleared = transient_cleared && replay_map->grid->occupancy[transient] == 0;
 stable_retained = stable_retained || replay_map->grid->occupancy[stable] >= 50;
 }
 }
 CHECK(transient_cleared);
 CHECK(stable_retained);

 // Transient filtering: a post that stops being observed must leave the live
 // submap, while the wall behind it stays. Two wrong answers are rejected
 // here: a map that keeps burned-in phantom obstacles, and a map that lets a
 // single stray ray delete real geometry.
 sim::autonomy::OnlineSlamConfig transient_config = config;
 transient_config.translation_search_m = 0.0;
 transient_config.heading_search_rad = 0.0;
 sim::autonomy::OnlineSlamMapBuilder transient_builder(transient_config);
 sim::EstimatedState fixed_prior;
 fixed_prior.position = {0.0, 0.0};
 fixed_prior.heading_rad = 0.0;
 fixed_prior.confidence = 1.0;
 std::shared_ptr<const sim::runtime::MapSnapshot> transient_map;
 std::uint64_t sequence = 1;
 double stamp = 1.0;
 for (int scan_index = 0; scan_index < 4; ++scan_index) {
 transient_map = transient_builder.update(
 frame(scan_with_post(stamp, sequence++, 5.0, 2.5, true)), fixed_prior,
 sim::runtime::TimePoint::from_seconds(stamp)).published_map;
 stamp += 0.1;
 }
 CHECK(transient_map && transient_map->grid);
 const auto cell_index = [&](double x, double y) {
 const auto cell = sim::runtime::world_to_grid(*transient_map->grid, {x, y});
 if (!cell)
 return std::size_t{0};
 return static_cast<std::size_t>(cell->y) * transient_map->grid->width + cell->x;
 };
 const auto post_cell = cell_index(2.5, 0.0);
 const auto wall_cell = cell_index(5.0, 3.0);
 // A cell centre may straddle the exact geometric plane of the fixture, so
 // confirmation is checked over the small window the return can land in.
 const auto occupied_near = [&](std::size_t centre, int radius) {
 for (int dy = -radius; dy <= radius; ++dy) {
 for (int dx = -radius; dx <= radius; ++dx) {
 const auto index = centre + static_cast<std::size_t>(dy) *
 transient_map->grid->width + static_cast<std::ptrdiff_t>(dx);
 if (transient_map->grid->occupancy[index] >= 50)
 return true;
 }
 }
 return false;
 };
 CHECK(occupied_near(post_cell, 1));
 CHECK(occupied_near(wall_cell, 1));
 for (int scan_index = 0; scan_index < 20; ++scan_index) {
 transient_map = transient_builder.update(
 frame(scan_with_post(stamp, sequence++, 5.0, 2.5, false)), fixed_prior,
 sim::runtime::TimePoint::from_seconds(stamp)).published_map;
 stamp += 0.1;
 }
 CHECK(transient_map && transient_map->grid);
 int phantom = 0;
 for (int dy = -3; dy <= 3; ++dy) {
 for (int dx = -3; dx <= 3; ++dx) {
 if (transient_map->grid->occupancy[post_cell + static_cast<std::size_t>(dy) *
 transient_map->grid->width + dx] >= 50)
 ++phantom;
 }
 }
 CHECK(phantom == 0);
 CHECK(occupied_near(wall_cell, 1));

 return 0;
}
