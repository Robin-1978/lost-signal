#include "robot/autonomy/pipeline/autonomy_pipeline.hpp"

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
 value.header = {stamp_s, 1, "lidar_link", 0.0, 1, "test.autonomy"};
 if (foreground) {
 value.ranges[0] = 2.0;
 value.hit[0] = 1;
 }
 return value;
}

} // namespace

int main() {
 sim::GridMap static_map(80, 80, 0.25);
 sim::AutonomyPipeline pipeline(static_map, 80, 80, 0.25, 19,
 {0.5, 0.2});

 const auto perception = pipeline.process_lidar(
 scan(1.0, true), scan(1.0, false),
 {{1.0, 1.0}, 0.0, 0.0, 0.0}, {}, 1.0);
 CHECK(perception.hits.size() == 1);
 CHECK(!pipeline.compose_local_map(static_map, 1.0, 0.0)
 .isFree(3.0, 1.0));

 sim::LocalPlanningRequest request;
 request.position = {1.0, 2.0};
 request.target = {3.0, 2.0};
 request.max_speed_mps = 0.75;
 request.max_yaw_rate_radps = 1.5;
 request.max_accel_mps2 = 2.0;
 request.max_yaw_accel_radps2 = 4.0;
 request.prediction_horizon_s = 1.0;
 request.step_s = 0.1;
 request.robot_radius_m = 0.3;
 const auto plan = pipeline.plan_local(request, static_map, {});
 CHECK(plan.selection.validation.accepted());
 CHECK(plan.selection.trajectory.command.valid);

 const sim::TrajectoryTrackingLimits limits{0.25, 0.25, 0.5};
 CHECK(pipeline.accept_trajectory(
 2.0, plan.selection.trajectory, limits) == 1);
 const auto tracking = pipeline.observe_trajectory(
 2.0, request.position, 0.0, 0.0);
 CHECK(tracking.active);
 CHECK(tracking.within_envelope);
 CHECK(tracking.sequence == 1);

 pipeline.reset(19);
 CHECK(pipeline.dynamic_obstacles().active_tracks(2.0, 1.0).empty());
 CHECK(!pipeline.observe_trajectory(2.0, {}, 0.0, 0.0).active);
 return 0;
}
