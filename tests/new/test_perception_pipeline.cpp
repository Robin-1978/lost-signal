#include "autonomy/perception/perception_pipeline.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::Scan scan(double stamp_s, bool foreground) {
 sim::Scan value;
 value.n_rays = 4;
 value.max_range = 6.0;
 value.angle_min_rad = 0.0;
 value.angle_increment_rad = sim::kPi * 0.5;
 value.scan_period_s = 0.1;
 value.ranges.assign(4, value.max_range);
 value.hit.assign(4, 0);
 value.intensities.assign(4, 0.0);
 value.header = {stamp_s, static_cast<std::uint64_t>(std::llround(stamp_s * 10.0)),
 "lidar_link", 0.0, 1, "test.pipeline"};
 if (foreground) {
 value.ranges[0] = 2.0;
 value.hit[0] = 1;
 value.intensities[0] = 0.8;
 }
 return value;
}

} // namespace

int main() {
 sim::GridMap static_map(40, 40, 0.25);
 static_map.setOccupied(0.625, 0.625);
 sim::PerceptionPipeline pipeline(static_map, 40, 40, 0.25, 17, {0.5, 0.2});

 auto measured = scan(1.0, true);
 auto reference = scan(1.0, false);
 const auto frame = pipeline.process(measured, reference, {{1.0, 1.0}, 0.0, 0.0, 0.0}, {}, 1.0);
 CHECK(frame.hits.size() == 1);
 CHECK(frame.obstacle_rays.size() == 4);
 CHECK(frame.obstacle_rays.front().hit);
 CHECK(pipeline.obstacle_layer().active_obstacle_cells(1.0) == 1);

 // Composition keeps immutable authored occupancy and adds the transient
 // endpoint at the projected sensor return.
 const auto composed = pipeline.compose_local_map(static_map, 1.0, 0.0);
 CHECK(!composed.isFree(0.625, 0.625));
 CHECK(!composed.isFree(3.0, 1.0));

 // A newer clear scan removes the transient return without erasing authored
 // geometry. The same state is used by simulation, replay, and hardware input.
 measured = scan(1.1, false);
 reference = scan(1.1, false);
 pipeline.process(measured, reference, {{1.0, 1.0}, 0.0, 0.0, 0.0}, {}, 1.0);
 const auto cleared = pipeline.compose_local_map(static_map, 1.1, 0.0);
 CHECK(cleared.isFree(3.0, 1.0));
 CHECK(!cleared.isFree(0.625, 0.625));

 // Acquisition time is authoritative. An out-of-order scan cannot rewind
 // transient occupancy even if transport delivers it later.
 bool rejected = false;
 try {
 auto stale = scan(1.05, true);
 auto stale_reference = scan(1.05, false);
 pipeline.process(stale, stale_reference, {{1.0, 1.0}, 0.0, 0.0, 0.0}, {}, 1.0);
 } catch (const std::invalid_argument&) {
 rejected = true;
 }
 CHECK(rejected);

 pipeline.reset(17);
 CHECK(pipeline.obstacle_layer().active_obstacle_cells(2.0) == 0);
 CHECK(pipeline.dynamic_obstacles().active_tracks(2.0, 1.0).empty());
 return 0;
}
