#include "robot/autonomy/mapping/slam_map_publication.hpp"

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

sim::Scan scan(double stamp_s, std::uint64_t sequence, double range) {
 sim::Scan value;
 value.n_rays = 36;
 value.max_range = 12.0;
 value.angle_min_rad = -sim::kPi;
 value.angle_increment_rad = 2.0 * sim::kPi / value.n_rays;
 value.ranges.assign(value.n_rays, value.max_range);
 value.hit.assign(value.n_rays, 0);
 value.intensities.assign(value.n_rays, 1.0);
 for (int index = 0; index < value.n_rays; ++index) {
 if (index % 3 == 0) {
 value.ranges[static_cast<std::size_t>(index)] = range;
 value.hit[static_cast<std::size_t>(index)] = 1;
 }
 }
 value.header = {stamp_s, sequence, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.slam-publication"};
 return value;
}

sim::runtime::MapSnapshot base_map(double stamp_s) {
 sim::runtime::MapSnapshot snapshot;
 snapshot.map_id = "slam-session";
 snapshot.revision = 3;
 snapshot.frame_id = "map";
 snapshot.sample_time = sim::runtime::TimePoint::from_seconds(stamp_s);
 snapshot.provenance = {"slam", "test.fixture", "3"};
 sim::runtime::MapGrid grid;
 grid.width = 8;
 grid.height = 8;
 grid.resolution_m = 0.05;
 grid.origin_m = {-0.2, -0.2};
 grid.occupancy.assign(64, -1);
 snapshot.grid = grid;
 return snapshot;
}

std::size_t occupied_cells(const sim::runtime::MapSnapshot& snapshot) {
 if (!snapshot.grid)
 return 0;
 return static_cast<std::size_t>(std::count_if(
 snapshot.grid->occupancy.begin(), snapshot.grid->occupancy.end(),
 [](std::int8_t value) { return value >= 50; }));
}

} // namespace

int main() {
 sim::autonomy::MappingSessionConfig session_config;
 session_config.keyframes.translation_threshold_m = 0.4;
 session_config.keyframes.heading_threshold_rad = 0.3;
 session_config.keyframes.maximum_interval_s = 5.0;
 session_config.keyframes.maximum_keyframes = 16;
 session_config.loop_closure.minimum_temporal_separation_s = 2.0;
 sim::autonomy::MappingSession session(session_config);
 CHECK(session.ingest({{0.0, 0.0}, 0.0}, scan(1.0, 1, 2.5), 1.0).created_keyframe);
 CHECK(session.ingest({{0.6, 0.0}, 0.0}, scan(2.0, 2, 2.6), 2.0).created_keyframe);
 CHECK(session.ingest({{0.6, 0.6}, 1.57}, scan(3.0, 3, 2.4), 3.0).created_keyframe);
 // Deliberate odometry drift: the return leg lands short of the origin.
 CHECK(session.ingest({{0.0, 0.65}, 3.05}, scan(4.0, 4, 2.5), 4.0).created_keyframe);

 sim::autonomy::LoopClosureEvidence evidence;
 evidence.match_score = 0.94;
 evidence.overlap = 0.90;
 evidence.score_margin = 0.25;
 // Ground-truth closure: the fourth keyframe is really back at the origin.
 evidence.measured_relative_pose = sim::autonomy::relative_pose(
 sim::autonomy::Pose2{{0.0, 0.0}, 0.0},
 sim::autonomy::Pose2{{0.0, 0.0}, 0.0});
 evidence.information = sim::autonomy::pose_graph_information(0.05, 0.03);
 const auto closure = session.add_loop_closure(4, 1, evidence);
 CHECK(closure.accepted);

 const auto uncorrected = session.keyframes();
 sim::autonomy::KeyframeMapRebuildConfig rebuild_config;
 rebuild_config.width = 200;
 rebuild_config.height = 200;
 rebuild_config.resolution_m = 0.05;
 rebuild_config.map_id = "slam-session";
 sim::autonomy::KeyframeMapRebuilder rebuilder(rebuild_config);

 sim::runtime::MapManager maps;
 std::string reason;
 CHECK(maps.configure(base_map(0.0), &reason));
 CHECK(maps.activate());

 sim::autonomy::SlamMapPublication publisher;
 const auto publication = publisher.optimize_rebuild_publish(
 session, rebuilder, maps, 5.0);
 CHECK(publication.success);
 CHECK(publication.graph_optimized);
 CHECK(publication.map_result == sim::runtime::MapUpdateResult::Accepted);
 CHECK(publication.revision == 4);
 CHECK(maps.current_snapshot());
 CHECK(maps.current_snapshot()->revision == 4);
 CHECK(maps.current_snapshot()->provenance.producer_id == "robot.keyframe-map-rebuilder");
 CHECK(occupied_cells(*maps.current_snapshot()) > 0);

 // The session now carries the optimized poses, so a later rebuild of the
 // same keyframes reproduces the published geometry.
 const auto corrected_pose = session.keyframes().back().graph.initial_pose;
 CHECK(sim::autonomy::pose_translation_error(corrected_pose,
 uncorrected.back().graph.initial_pose) > 0.05);
 CHECK(sim::autonomy::pose_translation_error(uncorrected.back().graph.initial_pose,
 sim::autonomy::Pose2{{0.0, 0.0}, 0.0}) > 0.50);
 const auto naive = rebuilder.rebuild(uncorrected, 4, 5.0);
 CHECK(naive.success);
 CHECK(naive.snapshot->grid->occupancy != maps.current_snapshot()->grid->occupancy);

 // A second accepted closure must publish a strictly newer revision.
 const auto again = publisher.optimize_rebuild_publish(session, rebuilder, maps, 5.0);
 CHECK(again.success);
 CHECK(again.revision == 5);
 CHECK(maps.current_snapshot()->revision == 5);

 // An inactive provider must not consume an optimization or a revision.
 sim::runtime::MapManager inactive;
 CHECK(inactive.configure(base_map(0.0), &reason));
 const auto rejected = publisher.optimize_rebuild_publish(session, rebuilder, inactive, 6.0);
 CHECK(!rejected.success);
 CHECK(rejected.diagnostic == "map manager is not active");
 CHECK(inactive.current_snapshot()->revision == 3);

 // A session without retained scans cannot publish anything.
 sim::autonomy::MappingSession empty_session(session_config);
 const auto empty = publisher.optimize_rebuild_publish(empty_session, rebuilder, maps, 6.0);
 CHECK(!empty.success);
 CHECK(empty.diagnostic == "mapping session has no keyframes");
 CHECK(maps.current_snapshot()->revision == 5);

 // Pose transfer must reject a mismatched graph instead of relabelling nodes.
 sim::autonomy::PoseGraphBackend foreign;
 const auto foreign_result = session.apply_optimized_poses(foreign, &reason);
 CHECK(!foreign_result);
 CHECK(reason == "optimized graph does not match this session");

 // A non-monotonic publication time is a contract error, not a stale map.
 const auto stale_time = publisher.optimize_rebuild_publish(session, rebuilder, maps, -1.0);
 CHECK(!stale_time.success);
 CHECK(stale_time.diagnostic == "invalid map publication timestamp");
 CHECK(maps.current_snapshot()->revision == 5);
 return 0;
}
