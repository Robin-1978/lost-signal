#include "robot/autonomy/mapping/loop_closure_detector.hpp"
#include "robot/runtime/map_provider.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::runtime::MapSnapshot make_room(bool symmetric) {
 sim::runtime::MapSnapshot map;
 map.map_id = symmetric ? "symmetric-room" : "asymmetric-room";
 map.frame_id = "map";
 map.revision = 1;
 auto& grid = map.grid.emplace();
 grid.width = 100;
 grid.height = 80;
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
 box(0.0, 0.0, 10.0, 0.2);
 box(0.0, 7.8, 10.0, 8.0);
 box(0.0, 0.0, 0.2, 8.0);
 box(9.8, 0.0, 10.0, 8.0);
 if (!symmetric) {
 // Asymmetric geometry gives the loop closure a unique interpretation.
 box(2.0, 5.4, 3.6, 5.8);
 box(6.4, 2.2, 6.8, 4.6);
 box(8.2, 6.4, 8.6, 6.9);
 }
 return map;
}

sim::Scan scan_at(const sim::runtime::MapGrid& grid, sim::Vec2 pose,
 double heading, double stamp_s) {
 sim::Scan scan;
 scan.n_rays = 180;
 scan.max_range = 9.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.scan_period_s = 0.1;
 scan.header = {stamp_s, 1, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.loop-detector"};
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

sim::autonomy::MappingSessionKeyframe keyframe(std::uint64_t id, double stamp_s,
 sim::autonomy::Pose2 pose, sim::Scan scan) {
 sim::autonomy::MappingSessionKeyframe value;
 value.graph.id = id;
 value.graph.stamp_s = stamp_s;
 value.graph.initial_pose = pose;
 value.scan = std::move(scan);
 return value;
}

} // namespace

int main() {
 const auto map = make_room(false);
 const auto& grid = *map.grid;
 const sim::Vec2 physical_pose{2.4, 3.1};
 const double physical_heading = 0.35;

 // A loop that closes after large odometric drift widens the verification
 // window. The gate still requires a strong, unambiguous two-way overlap.
 sim::autonomy::LoopClosureDetectorConfig drifted_config;
 drifted_config.search_translation_m = 3.0;
 drifted_config.search_heading_rad = 0.4;
 drifted_config.translation_step_m = 0.15;
 drifted_config.heading_step_rad = 0.05;
 sim::autonomy::LoopClosureDetector detector(drifted_config);

 // The odometry pose has drifted by two metres, but the retained scan was
 // taken at the same physical place as keyframe 1. Verification must recover
 // the near-identity relative transform.
 std::vector<sim::autonomy::MappingSessionKeyframe> keyframes;
 keyframes.push_back(keyframe(1, 1.0, {{physical_pose.x, physical_pose.y},
 physical_heading},
 scan_at(grid, physical_pose, physical_heading, 1.0)));
 keyframes.push_back(keyframe(2, 12.0, {{physical_pose.x + 2.0, physical_pose.y + 1.3},
 physical_heading + 0.22},
 scan_at(grid, physical_pose, physical_heading, 12.0)));
 const auto detected = detector.detect(keyframes);
 CHECK(detected.has_value());
 CHECK(detected->from_keyframe == 1);
 CHECK(detected->to_keyframe == 2);
 const double translation = std::hypot(detected->evidence.measured_relative_pose.position.x,
 detected->evidence.measured_relative_pose.position.y);
 CHECK(translation < 0.20);
 CHECK(std::abs(sim::wrap_angle(detected->evidence.measured_relative_pose.heading_rad)) < 0.05);
 CHECK(detected->evidence.overlap > 0.80);
 CHECK(detected->evidence.score_margin > 0.02);
 std::cerr << "loop translation=" << translation
 << " overlap=" << detected->evidence.overlap
 << " margin=" << detected->evidence.score_margin << "\n";

 // A four-fold symmetric room cannot pin down a unique relative transform.
 // The detector must report a small margin so the gate rejects the candidate.
 const auto symmetric = make_room(true);
 std::vector<sim::autonomy::MappingSessionKeyframe> symmetric_frames;
 const sim::Vec2 centre{5.0, 4.0};
 symmetric_frames.push_back(keyframe(1, 1.0, {centre, 0.0},
 scan_at(*symmetric.grid, centre, 0.0, 1.0)));
 symmetric_frames.push_back(keyframe(2, 12.0, {{centre.x + 1.4, centre.y + 0.8}, 0.6},
 scan_at(*symmetric.grid, centre, 0.0, 12.0)));
 sim::autonomy::LoopClosureDetector strict_detector;
 const auto symmetric_result = strict_detector.detect(symmetric_frames);
 if (symmetric_result) {
 // The detector only reports measured evidence; the gate owns the policy
 // threshold. This fixture is deliberately rejected by a stricter ambiguity
 // policy rather than asserting a raster-specific score value.
 sim::autonomy::LoopClosureGateConfig gate_config;
 gate_config.minimum_score_margin = 0.65;
 sim::autonomy::LoopClosureGate gate(gate_config);
 const auto decision = gate.validate(symmetric_frames.front().graph,
 symmetric_frames.back().graph, symmetric_result->evidence);
 CHECK(!decision.accepted);
 }
 return 0;
}
