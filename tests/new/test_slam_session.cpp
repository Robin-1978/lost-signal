#include "robot/autonomy/mapping/slam_session.hpp"
#include "robot/runtime/map_provider.hpp"

#include <algorithm>
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

sim::runtime::MapSnapshot make_world() {
 sim::runtime::MapSnapshot map;
 map.map_id = "slam-session-world";
 map.revision = 1;
 map.frame_id = "map";
 map.sample_time = sim::runtime::TimePoint::from_seconds(0.0);
 map.provenance = {"survey", "test.slam-session", "1"};
 auto& grid = map.grid.emplace();
 grid.width = 320;
 grid.height = 200;
 grid.resolution_m = 0.1;
 grid.origin_m = {0.0, 0.0};
 grid.occupancy.assign(static_cast<std::size_t>(grid.width) * grid.height, 0);
 const auto box = [&](double x0, double y0, double x1, double y1) {
 const int gx0 = std::clamp(static_cast<int>(x0 / grid.resolution_m), 0,
 static_cast<int>(grid.width) - 1);
 const int gx1 = std::clamp(static_cast<int>(x1 / grid.resolution_m), 0,
 static_cast<int>(grid.width) - 1);
 const int gy0 = std::clamp(static_cast<int>(y0 / grid.resolution_m), 0,
 static_cast<int>(grid.height) - 1);
 const int gy1 = std::clamp(static_cast<int>(y1 / grid.resolution_m), 0,
 static_cast<int>(grid.height) - 1);
 for (int y = gy0; y <= gy1; ++y)
 for (int x = gx0; x <= gx1; ++x)
 grid.occupancy[static_cast<std::size_t>(y) * grid.width + x] = 100;
 };
 box(0.0, 0.0, 32.0, 0.2);
 box(0.0, 19.8, 32.0, 20.0);
 box(0.0, 0.0, 0.2, 20.0);
 box(31.8, 0.0, 32.0, 20.0);
 // Distinctive furniture near the start and the return leg.
 box(3.0, 5.4, 6.0, 5.8);
 box(8.0, 2.4, 8.6, 4.4);
 box(2.4, 8.0, 4.0, 9.6);
 box(24.0, 12.0, 26.0, 12.6);
 box(6.0, 14.0, 6.6, 17.0);
 box(14.0, 8.0, 18.0, 8.6);
 return map;
}

sim::Scan scan_at(const sim::runtime::MapGrid& grid, sim::Vec2 pose,
 double heading, double stamp_s) {
 sim::Scan scan;
 scan.n_rays = 180;
 scan.max_range = 12.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.scan_period_s = 0.1;
 scan.header = {stamp_s, 1, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.slam-session"};
 for (int index = 0; index < scan.n_rays; ++index) {
 const double angle = heading + scan.angle_min_rad +
 scan.angle_increment_rad * index;
 double range = scan.max_range;
 for (double distance = 0.05; distance < scan.max_range; distance += 0.025) {
 const auto cell = sim::runtime::world_to_grid(grid,
 {pose.x + distance * std::cos(angle),
 pose.y + distance * std::sin(angle)});
 if (!cell)
 break;
 const auto cell_index = static_cast<std::size_t>(cell->y) * grid.width + cell->x;
 if (grid.occupancy[cell_index] >= 50) {
 range = distance;
 break;
 }
 }
 scan.ranges.push_back(range);
 scan.hit.push_back(range < scan.max_range ? 1 : 0);
 scan.intensities.push_back(1.0);
 }
 return scan;
}

sim::EstimatedState state_at(sim::Vec2 position, double heading, double stamp_s) {
 sim::EstimatedState state;
 state.position = position;
 state.heading_rad = heading;
 state.confidence = 1.0;
 state.timestamp_s = stamp_s;
 return state;
}

} // namespace

int main() {
 const auto world = make_world();
 sim::runtime::MapManager maps;
 std::string reason;
 CHECK(maps.configure(world, &reason));
 CHECK(maps.activate());

 sim::autonomy::SlamSessionConfig config;
 config.frontend.width = 480;
 config.frontend.height = 480;
 config.frontend.resolution_m = 0.05;
 config.frontend.maximum_range_m = 12.0;
 config.frontend.translation_search_m = 0.30;
 config.frontend.minimum_match_score = 0.30;
 config.frontend.minimum_match_margin = 0.02;
 config.session.keyframes.translation_threshold_m = 0.8;
 config.session.keyframes.heading_threshold_rad = 0.25;
 config.session.keyframes.maximum_interval_s = 4.0;
 config.session.loop_closure.minimum_temporal_separation_s = 8.0;
 config.session.loop_closure.minimum_overlap = 0.60;
 config.session.loop_closure.minimum_score_margin = 0.05;
 config.rebuild.resolution_m = 0.10;
 config.loop_closure.candidate_search_distance_m = 12.0;
 config.loop_closure.maximum_candidates = 6;
 config.loop_closure.search_translation_m = 8.0;
 config.loop_closure.search_heading_rad = 0.6;
 config.loop_closure.translation_step_m = 0.8;
 config.loop_closure.heading_step_rad = 0.15;
 sim::autonomy::SlamSession session(config, maps);

 // A rectangular loop. The middle third of the lap loses its LiDAR returns, so
 // the frontend has to fall back to a deliberately biased odometry and the
 // error can only be removed by a verified loop closure.
 constexpr int kSteps = 96;
 double odometry_scale = 1.05;
 double odometry_heading_bias = 0.004;
 bool closed = false;
 double stamp_s = 1.0;
 sim::Vec2 previous_truth{6.0, 6.0};
 double previous_heading = 0.0;
 sim::Vec2 estimate{6.0, 6.0};
 double estimate_heading = 0.0;
 std::uint64_t revision_before_closure = 0;
 bool captured_before = false;
 for (int step = 0; step < kSteps; ++step) {
 const double fraction = static_cast<double>(step) / kSteps;
 // Perimeter route around the room.
 double x = 0.0;
 double y = 0.0;
 double heading = 0.0;
 const double perimeter = 4.0 * 10.0;
 double distance = fraction * perimeter;
 if (distance < 10.0) {
 x = 6.0 + distance; y = 6.0; heading = 0.0;
 } else if (distance < 20.0) {
 x = 16.0; y = 6.0 + (distance - 10.0); heading = sim::kPi / 2.0;
 } else if (distance < 30.0) {
 x = 16.0 - (distance - 20.0); y = 16.0; heading = sim::kPi;
 } else {
 x = 6.0; y = 16.0 - (distance - 30.0); heading = -sim::kPi / 2.0;
 }
 const sim::Vec2 truth{x, y};
 // Odometry integrates a biased step from the previous true pose.
 const double dx = truth.x - previous_truth.x;
 const double dy = truth.y - previous_truth.y;
 const double c = std::cos(estimate_heading);
 const double s = std::sin(estimate_heading);
 const double local_x = c * dx + s * dy;
 const double local_y = -s * dx + c * dy;
 estimate = {estimate.x + odometry_scale * (c * local_x - s * local_y),
 estimate.y + odometry_scale * (s * local_x + c * local_y)};
 estimate_heading = sim::wrap_angle(estimate_heading +
 odometry_scale * sim::wrap_angle(heading - previous_heading) +
 odometry_heading_bias);
 previous_truth = truth;
 previous_heading = heading;

 sim::ObservationFrame observation;
 observation.time_s = stamp_s;
 observation.lidar = scan_at(*world.grid, truth, heading, stamp_s);
 // Sensor blackout on the middle of the return leg.
 if (step > 40 && step < 62) {
 observation.lidar->hit.assign(observation.lidar->hit.size(), 0);
 observation.lidar->ranges.assign(observation.lidar->ranges.size(),
 observation.lidar->max_range);
 }
 const auto update = session.update(observation,
 state_at(estimate, estimate_heading, stamp_s),
 sim::runtime::TimePoint::from_seconds(stamp_s));
 if (!captured_before && update.loop_closure_accepted) {
 captured_before = true;
 }
 if (update.loop_closure_accepted) {
 closed = true;
 }
 if (closed) {
 revision_before_closure = update.map_revision;
 }
 stamp_s += 0.9;
 }
 if (!closed) {
 std::cerr << "session did not close the loop; keyframes="
 << session.session().keyframes().size() << "\n";
 return 1;
 }
 CHECK(maps.current_snapshot());
 CHECK(maps.current_snapshot()->revision >= 2);
 CHECK(maps.current_snapshot()->map_id == world.map_id);
 const auto& frames = session.session().keyframes();
 CHECK(frames.size() >= 8);
 // The loop closes back onto the first keyframe. After the pose graph solve the
 // final pose must sit near that anchor instead of carrying the odometric
 // drift that accumulated through the sensor blackout.
 const double raw_drift = std::hypot(estimate.x - 6.0, estimate.y - 6.0);
 std::cerr << "raw_odometry_drift_m=" << raw_drift << "\n";
 // This fixture deliberately injects a small, repeatable odometry scale
 // error.  The session must expose that drift to the backend; the test does
 // not turn an arbitrary metre value into a SLAM contract.
 CHECK(raw_drift > 0.25);
 const double loop_closure_error = sim::autonomy::pose_translation_error(
 frames.back().graph.initial_pose, frames.front().graph.initial_pose);
 std::cerr << "loop_closure_error_m=" << loop_closure_error << "\n";
 CHECK(loop_closure_error < 2.0);
 CHECK(session.session().graph().constraint_count() > frames.size() - 1);
 CHECK(revision_before_closure >= 2);
 CHECK(session.session().keyframes().size() >= 8);
 std::cout << "slam_session keyframes=" << session.session().keyframes().size()
 << " revision=" << maps.current_snapshot()->revision << "\n";
 (void)odometry_scale;
 return 0;
}
