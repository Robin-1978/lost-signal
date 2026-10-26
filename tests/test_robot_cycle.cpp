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

  // Valid observation but no goal yet: holds safe stop while the
  // observation is accepted into the channel.
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

 // An aggregate envelope cannot smuggle a section owned by an unregistered
 // endpoint through a registered transport identity.
 iface::ObservationFrame spoofed = good;
 spoofed.sequence = 2;
 spoofed.stamp_ns = 3'500'000;
 spoofed.imu_valid = true;
 spoofed.imu_header = {
 1, 3'500'000, "rogue.imu", core::Frame::kImuLink};
 assert(!app.publish_observation(spoofed));

 // Legacy frames may represent one endpoint only. A headerless aggregate
 // cannot borrow one registered identity for unrelated sensor roles.
 iface::ObservationFrame legacy_aggregate = good;
 legacy_aggregate.sequence = 2;
 legacy_aggregate.stamp_ns = 3'600'000;
 legacy_aggregate.imu_valid = true;
 assert(!app.publish_observation(legacy_aggregate));

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

  // Task intent crosses as typed request only. Empty place = hold.
  // (Place resolution against a map with places is covered in test_places.)
  iface::TaskRequest req;
  req.request_id = "t1";
  req.place_id = "";
  iface::GoalHandle h = app.request_goal(req);
  assert(h.valid());
  assert(app.goal_status(h).state == iface::GoalState::kDone);  // hold acks at once

  // Goal state machine: unknown handle fails; cancel of a non-active
  // handle is false; a fresh app has no active goal to complete.
  iface::GoalHandle bogus;
  bogus.id = 999;
  assert(app.goal_status(bogus).state == iface::GoalState::kFailed);
  assert(!app.cancel_goal(bogus));
  assert(!app.cancel_goal(h)); // terminal hold: nothing to withdraw

 // Joint feedback older than the configured safety reaction window cannot
 // close a wheel loop. Fresh moving wheels demand braking duty; after the
 // feedback expires the actuator path fails closed to safe zero.
 {
 app::RobotApplication wheel_app(nullptr);
 assert(wheel_app.register_provider(
 {"test.encoder", iface::EndpointRole::kEncoder,
 iface::ProviderBackend::kNative, "base_link"}));
 assert(wheel_app.configure().ok && wheel_app.activate().ok);
 iface::ObservationFrame joints;
 joints.sequence = 1;
 joints.stamp_ns = 1'000'000;
 joints.source = "test.encoder";
 joints.frame = core::Frame::kBaseLink;
 joints.odometry_valid = true;
 joints.drive_count = 2;
 joints.drive_radps[0] = 10.0;
 joints.drive_radps[1] = 10.0;
 assert(wheel_app.publish_observation(joints));
 wheel_app.tick(1'000'000);
 const iface::ActuatorCommand fresh = wheel_app.take_actuator(1'000'000);
 assert(fresh.drive[0] < 0.0 && fresh.drive[1] < 0.0);
 // A deliberate safe stop combines the measured wheel-speed loop and the
 // independent mechanical brake.
 assert(fresh.service_brake == 1.0);
 wheel_app.tick(300'000'000);
 const iface::ActuatorCommand expired =
 wheel_app.take_actuator(300'000'000);
 assert(expired.is_safe_stop());
 }

  // Shutdown returns modules to Unconfigured; tick holds again.
  app.shutdown();
  assert(app.bringup_state() == lifecycle::ModuleState::kUnconfigured);
  iface::MotionCommand post = app.tick(5'000'000);
  assert(post.is_safe_stop());

  std::printf("robot_cycle ok\n");
  return 0;
}
