#include <cassert>
#include <cstdio>

#include "robot/app/robot_application.hpp"

int main() {
  using namespace lost_signal::robot;

  app::RobotApplication app(nullptr);

  // No observation yet -> must fail closed to safe stop.
  iface::MotionCommand cmd0 = app.tick(1'000'000);
  assert(cmd0.is_safe_stop());

  // Invalid observation must not advance state.
  iface::ObservationFrame bad;
  app.publish_observation(bad);
  iface::MotionCommand cmd1 = app.tick(2'000'000);
  assert(cmd1.is_safe_stop());

  // Valid observation still safe-stops in v1 skeleton, but is accepted.
  iface::ObservationFrame good;
  good.sequence = 1;
  good.stamp_ns = 3'000'000;
  good.source = "test.lidar";
  good.frame = core::Frame::kLidarLink;
  good.scan_valid = true;
  good.ranges_m = {3.0f};
  assert(iface::valid_observation_frame(good));
  app.publish_observation(good);
  iface::MotionCommand cmd2 = app.tick(3'000'000);
  assert(cmd2.is_safe_stop());
  assert(iface::valid_motion_command(cmd2, 3'000'000));

  // Stale sequence must be rejected (state stays at seq 1).
  iface::ObservationFrame stale = good;
  stale.sequence = 1;
  stale.stamp_ns = 4'000'000;
  app.publish_observation(stale);
  iface::MotionCommand cmd3 = app.tick(4'000'000);
  assert(cmd3.is_safe_stop());

  // Task intent crosses as typed request only.
  iface::TaskRequest req;
  req.request_id = "t1";
  req.place_id = "dock";
  assert(app.submit_task(req));
  assert(app.task_status().state == iface::TaskState::kActive);

  std::printf("robot_cycle ok\n");
  return 0;
}
