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
 return 0;
}
