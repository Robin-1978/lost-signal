#include <cassert>
#include <cstdio>

#include "robot/app/robot_application.hpp"

int main() {
  using namespace lost_signal::robot;

  app::RobotApplication app(nullptr);
  assert(app.register_provider({"test.lidar", iface::EndpointRole::kLidar,
                                iface::ProviderBackend::kNative, "lidar_link"}));

  // Pre-bringup tick: safe stop without processing (modules not Active).
  assert(app.bringup_state() == lifecycle::ModuleState::kUnconfigured);
  iface::MotionCommand pre = app.tick(500'000);
  assert(pre.is_safe_stop());

  // Ordered bringup.
  assert(app.configure());
  assert(app.bringup_state() == lifecycle::ModuleState::kInactive);
  assert(app.activate());
  assert(app.bringup_state() == lifecycle::ModuleState::kActive);

  // No observation yet -> must fail closed to safe stop.
  iface::MotionCommand cmd0 = app.tick(1'000'000);
  assert(cmd0.is_safe_stop());

  // Invalid observation must not advance state.
  iface::ObservationFrame bad;
  assert(!app.publish_observation(bad));
  iface::MotionCommand cmd1 = app.tick(2'000'000);
  assert(cmd1.is_safe_stop());

  // Unregistered source must never enter the decision path.
  iface::ObservationFrame rogue;
  rogue.sequence = 1;
  rogue.stamp_ns = 2'500'000;
  rogue.source = "rogue.lidar";
  rogue.frame = core::Frame::kLidarLink;
  rogue.scan_valid = true;
  rogue.ranges_m = {3.0f};
  assert(!app.publish_observation(rogue));

  // Valid observation still safe-stops in v1 skeleton, but is accepted.
  iface::ObservationFrame good;
  good.sequence = 1;
  good.stamp_ns = 3'000'000;
  good.source = "test.lidar";
  good.frame = core::Frame::kLidarLink;
  good.scan_valid = true;
  good.ranges_m = {3.0f};
  assert(iface::valid_observation_frame(good));
  assert(app.publish_observation(good));
  iface::MotionCommand cmd2 = app.tick(3'000'000);
  assert(cmd2.is_safe_stop());
  assert(iface::valid_motion_command(cmd2, 3'000'000));

  // Stale sequence must be rejected (state stays at seq 1).
  iface::ObservationFrame stale = good;
  stale.sequence = 1;
  stale.stamp_ns = 4'000'000;
  assert(!app.publish_observation(stale));
  iface::MotionCommand cmd3 = app.tick(4'000'000);
  assert(cmd3.is_safe_stop());

  // Take path enforces expiry: command from t=4ms is dead at t=1s.
  iface::MotionCommand taken = app.take_command(4'000'000 + 100'000'000);
  assert(taken.is_safe_stop());
  iface::MotionCommand expired = app.take_command(2'000'000'000);
  assert(expired.is_safe_stop());

  // Task intent crosses as typed request only.
  iface::TaskRequest req;
  req.request_id = "t1";
  req.place_id = "dock";
  assert(app.submit_task(req));
  assert(app.task_status().state == iface::TaskState::kActive);

  // Shutdown returns modules to Unconfigured; tick holds again.
  app.shutdown();
  assert(app.bringup_state() == lifecycle::ModuleState::kUnconfigured);
  iface::MotionCommand post = app.tick(5'000'000);
  assert(post.is_safe_stop());

  std::printf("robot_cycle ok\n");
  return 0;
}
