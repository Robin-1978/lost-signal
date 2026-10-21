#include "robot/autonomy/mapping/keyframe_map_rebuilder.hpp"

#include <algorithm>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::Scan scan(double stamp_s, std::uint64_t sequence, double wall_range) {
 sim::Scan value;
 value.n_rays = 8;
 value.max_range = 6.0;
 value.angle_min_rad = -sim::kPi;
 value.angle_increment_rad = 2.0 * sim::kPi / value.n_rays;
 value.ranges.assign(value.n_rays, value.max_range);
 value.hit.assign(value.n_rays, 0);
 value.intensities.assign(value.n_rays, 1.0);
 // The two forward-facing returns form a wall at x=wall_range.
 value.ranges[3] = wall_range;
 value.ranges[4] = wall_range;
 value.hit[3] = 1;
 value.hit[4] = 1;
 value.header = {stamp_s, sequence, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.rebuilder"};
 return value;
}

} // namespace

int main() {
 sim::autonomy::MappingSession session;
 auto first = session.ingest({{0.0, 0.0}, 0.0}, scan(1.0, 1, 3.0), 1.0);
 CHECK(first.created_keyframe);
 auto second = session.ingest({{0.5, 0.0}, 0.0}, scan(2.0, 2, 2.5), 2.0);
 CHECK(second.created_keyframe);

 sim::autonomy::KeyframeMapRebuildConfig config;
 config.width = 160;
 config.height = 160;
 config.resolution_m = 0.05;
 config.map_id = "rebuilt-map";
 sim::autonomy::KeyframeMapRebuilder rebuilder(config);
 const auto rebuilt = rebuilder.rebuild(session.keyframes(), 7, 3.0);
 CHECK(rebuilt.success);
 CHECK(rebuilt.snapshot);
 CHECK(rebuilt.snapshot->revision == 7);
 CHECK(rebuilt.snapshot->provenance.producer_id == "robot.keyframe-map-rebuilder");
 CHECK(rebuilt.snapshot->grid);
 CHECK(std::count_if(rebuilt.snapshot->grid->occupancy.begin(),
 rebuilt.snapshot->grid->occupancy.end(),
 [](std::int8_t value) { return value >= 50; }) > 0);
 CHECK(std::count_if(rebuilt.snapshot->grid->occupancy.begin(),
 rebuilt.snapshot->grid->occupancy.end(),
 [](std::int8_t value) { return value == 0; }) > 0);
 auto empty = rebuilder.rebuild({}, 8, 4.0);
 CHECK(!empty.success);
 CHECK(empty.diagnostic == "cannot rebuild a map without keyframes");
 return 0;
}
