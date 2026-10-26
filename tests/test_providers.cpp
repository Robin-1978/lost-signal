#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "robot/app/robot_application.hpp"
#include "sim/eval/drive_profile_io.hpp"
#include "sim/providers/sensor_suite.hpp"
#include "sim/providers/static_map.hpp"
#include "sim/world/motor.hpp"
#include "sim/world/world.hpp"

int main() {
  using namespace lost_signal;

#ifdef LOST_SIGNAL_WITH_MUJOCO
 // The 1 kHz path inspects only a typed model revision + actuator layout.
 // Serialization and compilation happen once, then only after a model input
 // changes. Dynamic environment/place edits stay outside the compiled plant.
 {
 sim::world::World w;
 robot::iface::ActuatorCommand zero;
 zero.mode = robot::iface::ActuatorMode::kDiffDuty;
 w.apply(zero);
 w.prepare();
 const auto built = w.mujoco_build_stats();
 assert(built.sync_requests == 1);
 assert(built.signature_serializations == 1);
 assert(built.rebuild_attempts == 1);
 assert(built.successful_rebuilds == 1);

 for (int i = 0; i < 100; ++i) w.step(0.001);
 const auto unchanged = w.mujoco_build_stats();
 assert(unchanged.sync_requests == built.sync_requests + 100);
 assert(unchanged.fast_path_hits == built.fast_path_hits + 100);
 assert(unchanged.signature_serializations ==
 built.signature_serializations);
 assert(unchanged.rebuild_attempts == built.rebuild_attempts);

 w.set_environment({{0.0, {{"default", 0.8}}}});
 w.set_places({{"dock", 4.0, 2.0, 0.0}});
 w.prepare();
 const auto dynamic_edit = w.mujoco_build_stats();
 assert(dynamic_edit.signature_serializations ==
 unchanged.signature_serializations);
 assert(dynamic_edit.rebuild_attempts == unchanged.rebuild_attempts);

 const std::vector<sim::world::StaticBox> boxes = {
 {100.0, 100.0, 101.0, 101.0, 1.0}};
 w.set_static_boxes(boxes);
 w.prepare();
 const auto geometry_edit = w.mujoco_build_stats();
 assert(geometry_edit.signature_serializations ==
 dynamic_edit.signature_serializations + 1);
 assert(geometry_edit.rebuild_attempts ==
 dynamic_edit.rebuild_attempts + 1);
 assert(geometry_edit.successful_rebuilds ==
 dynamic_edit.successful_rebuilds + 1);

 // Replacing configuration with identical content advances neither survey
 // nor model revision, so even the one-time serializer is skipped.
 const std::uint64_t geometry_version = w.geometry_version();
 w.set_static_boxes(boxes);
 w.set_motor_params(w.motor_params());
 w.prepare();
 const auto identical_edit = w.mujoco_build_stats();
 assert(w.geometry_version() == geometry_version);
 assert(identical_edit.signature_serializations ==
 geometry_edit.signature_serializations);
 assert(identical_edit.rebuild_attempts == geometry_edit.rebuild_attempts);
 }
#endif

  sim::world::World world;
 sim::world::MotorParams brake_motor = world.motor_params();
 brake_motor.brake_max_torque_Nm = 5.0;
 world.set_motor_params(brake_motor);
  sim::providers::SimSensorSuite suite(&world);
  robot::app::RobotApplication app(nullptr);
  suite.register_all(app);
  assert(app.configure().ok && app.activate().ok);

  // Envelope carries all three sections from typed messages, no hand-pack.
  world.step(0.02);
  robot::iface::ObservationFrame obs;
  assert(suite.poll_envelope(obs));
 assert(obs.odometry_valid && obs.scan_valid && obs.imu_valid);
 assert(obs.motion_header.valid() && obs.motion_header.source == "sim.motion");
 assert(obs.scan_header.valid() && obs.scan_header.source == "sim.lidar2d");
 assert(obs.imu_header.valid() && obs.imu_header.source == "sim.imu");
 assert(obs.motion_header.stamp_ns == obs.scan_header.stamp_ns);
 assert(obs.scan_header.stamp_ns == obs.imu_header.stamp_ns);
 // SIM publishes the same raw sensor contract as a real lidar. It cannot
 // attach an actor-free reference rendering to a ROBOT observation.
 assert(!obs.scan_static_valid && obs.static_ranges_m.empty());
  assert(obs.ranges_m.size() == 72);
  assert(robot::iface::valid_observation_frame(obs));
  assert(app.publish_observation(obs));

  // Actuator endpoint closes the loop: duty -> first-order plant -> wheels.
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 20'000'000;
  cmd.expiry_ns = 220'000'000;
  cmd.drive[0] = 1.0;
  cmd.drive[1] = 1.0;
  assert(robot::iface::valid_actuator_command(cmd, 20'000'000));
  suite.consume(cmd);
  double x0 = world.truth().body_x_m;
  world.step(0.02);
  // Engine pulls immediately: small but strictly positive first step.
  const double dx1 = world.truth().body_x_m - x0;
  assert(dx1 > 0.0 && dx1 < 0.01);
  assert(world.truth().drive_radps[0] > 0.0);
  // Settles at the motor-vs-drag equilibrium (~2.90 on the engine),
  // never above free speed.
  for (int i = 0; i < 300; ++i) world.step(0.02);
  assert(world.truth().forward_mps > 2.80 && world.truth().forward_mps <= 3.0);

  // Feedback reflects the plant, and the envelope carries it through.
  // Encoder velocities differentiate over the poll cadence (native-driver
  // parity): poll at rate, like the live loop does — a sparse poll reads
  // the honest time-average, never an instantaneous sample.
  robot::iface::ObservationFrame obs2;
  for (int i = 0; i < 5; ++i) {
    world.step(0.02);
    assert(suite.poll_envelope(obs2));
  }
  assert(obs2.drive_count == 2);
  assert(obs2.forward_mps > 2.80 && obs2.forward_mps <= 3.0);
  // Return-strength channel rides the envelope parallel to ranges
  // (measurement metadata; the ROBOT never maps it back to materials).
  assert(obs2.scan_valid && obs2.intensity_valid);
  assert(obs2.intensities.size() == obs2.ranges_m.size());
  // Rolling-shutter offsets ride parallel too (scan-end frame).
  assert(obs2.time_offset_valid);
  assert(obs2.time_offsets_s.size() == obs2.ranges_m.size());
  assert(obs2.time_offsets_s.front() < 0.0f);

 // The deployable actuator contract carries an independent mechanical
 // brake request through the same endpoint as motor duty. The SIM brake
 // has finite apply lag and is passive, but stops a coasting plant.
 const robot::iface::ActuatorCommand stop =
 robot::iface::ActuatorCommand::SafeStop(
 world.sim_time_ns(), robot::iface::ActuatorMode::kDiffDuty);
 assert(robot::iface::valid_actuator_command(stop, world.sim_time_ns()));
 suite.consume(stop);
 double minimum_forward_mps = world.truth().forward_mps;
 for (int i = 0; i < 200; ++i) {
 world.step(0.02);
 minimum_forward_mps =
 std::min(minimum_forward_mps, world.truth().forward_mps);
 }
 assert(std::abs(world.truth().forward_mps) < 0.05);
 assert(minimum_forward_mps > -0.05); // passive brake never drives backward

 // Independent provider deadlines must not manufacture fresh data for the
 // other sections. One transport sequence remains monotonic while the
 // section validity flags describe acquisition on this tap.
 const auto seq0 = obs2.sequence;
 const auto stamp0 = obs2.stamp_ns;
 robot::iface::ObservationFrame scan_only;
 world.step(0.02);
 assert(suite.poll_envelope(true, false, false, scan_only));
 assert(scan_only.sequence > seq0 && scan_only.stamp_ns >= stamp0);
 assert(scan_only.scan_valid && scan_only.intensity_valid);
 assert(scan_only.scan_header.source == "sim.lidar2d");
 assert(!scan_only.odometry_valid && !scan_only.imu_valid);
 assert(scan_only.drive_count == 0 && scan_only.motion_header.empty());

 robot::iface::ObservationFrame motion_only;
 world.step(0.02);
 assert(suite.poll_envelope(false, true, false, motion_only));
 assert(motion_only.sequence > scan_only.sequence);
 assert(!motion_only.scan_valid && !motion_only.intensity_valid);
 assert(motion_only.odometry_valid && !motion_only.imu_valid);
 assert(motion_only.motion_header.source == "sim.motion");
 assert(motion_only.stamp_ns >= scan_only.stamp_ns);

 robot::iface::ObservationFrame imu_only;
 world.step(0.02);
 assert(suite.poll_envelope(false, false, true, imu_only));
 assert(imu_only.sequence > motion_only.sequence);
 assert(!imu_only.scan_valid && !imu_only.odometry_valid);
 assert(imu_only.imu_valid);
 assert(imu_only.imu_header.source == "sim.imu");
 assert(imu_only.stamp_ns >= motion_only.stamp_ns);
  // Traction flags stay SIM-side (WorldTruth, offline labels only — the
  // ROBOT side only ever sees what its own estimators can compute).
  // Quiet in normal driving...
  assert(!world.truth().drive_slipping[0] &&
         !world.truth().drive_slipping[1]);
 // ...and firing once the resolved low-mu contacts actually slide. A hard
 // launch can briefly unload a wheel, so this is intentionally a bounded
 // physical observation window instead of assuming two outer ticks of
 // uninterrupted contact.
  sim::world::World wslip;
  sim::world::MotorParams low_mu = wslip.motor_params();
  low_mu.tire_friction_mu = 0.05;
  wslip.set_motor_params(low_mu);
  robot::iface::ActuatorCommand c2;
  c2.mode = robot::iface::ActuatorMode::kDiffDuty;
  c2.stamp_ns = 20'000'000;
  c2.expiry_ns = 220'000'000;
  c2.drive[0] = 1.0;
  c2.drive[1] = 1.0;
  wslip.apply(c2);
 bool low_mu_slip_seen = false;
 for (int i = 0; i < 100; ++i) {
 wslip.step(0.02);
 low_mu_slip_seen |= wslip.truth().drive_slipping[0] ||
 wslip.truth().drive_slipping[1];
 }
 assert(low_mu_slip_seen);
 // Once drive effort is withdrawn, the same contact-response accumulator
 // must drain again; a historical slip event is not a permanent state.
 wslip.apply(robot::iface::ActuatorCommand::SafeZero(
 wslip.sim_time_ns(), robot::iface::ActuatorMode::kDiffDuty));
 for (int i = 0; i < 100; ++i) wslip.step(0.02);
 assert(!wslip.truth().drive_slipping[0] &&
 !wslip.truth().drive_slipping[1]);

 // A 500 m x 400 m survey at 0.25 m resolution is 3.2 million cells.
 // Sparse obstacle rasterization must preserve exact geometry and cache the
 // immutable revision instead of rebuilding or copying it on every poll.
 {
 sim::world::World large_world;
 large_world.set_static_boxes({
 {10.0, 20.0, 20.0, 30.0, 1.5},
 {300.0, 100.0, 301.0, 350.0, 2.0},
 });
 sim::providers::StaticMapProvider provider(
 &large_world, 0.0, 0.0, 500.0, 400.0, 0.25);
 const auto& map = provider.active_snapshot();
 assert(map.width == 2000 && map.height == 1600);
 assert(map.occupancy.size() == 3'200'000);
 assert(map.elevation_m.size() == map.occupancy.size());
 assert(map.blocked(15.0, 25.0));
 const auto cell_at = [&](double x, double y) {
 const int gx = static_cast<int>(
 std::floor((x - map.origin_x_m) / map.resolution_m));
 const int gy = static_cast<int>(
 std::floor((y - map.origin_y_m) / map.resolution_m));
 return map.occupancy[static_cast<std::size_t>(gy) * map.width + gx];
 };
 assert(cell_at(100.0, 100.0) < 0);
 assert(map.elevation_at(15.0, 25.0) == 1.5f);
 assert(map.elevation_at(100.0, 100.0) == 0.0f);
 const auto* occupancy_data = map.occupancy.data();
 const std::uint64_t revision = map.revision;
 const auto& cached = provider.active_snapshot();
 assert(cached.revision == revision);
 assert(cached.occupancy.data() == occupancy_data);
 }

  // Drive profile file: overrides merge over factory defaults; malformed
  // files and unknown kinds fail closed without touching the output.
  {
    namespace eval = sim::eval;
    const std::string path =
        (std::filesystem::temp_directory_path() / "test_drive_profile.json")
            .string();
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out << "{\"kind\":\"differential\",\"free_speed_mps\":4.0}";
    }
    robot::platform::DriveProfile prof;
    std::string err;
    assert(eval::load_drive_profile(path, prof, &err));
    assert(prof.chassis_kind == "differential");
    assert(prof.free_speed_mps == 4.0);
    assert(prof.wheel_radius_m == 0.1);  // default kept
    // Partial override drives the plant: higher free speed retunes settling.
    sim::world::World tuned;
    tuned.set_motor_params(sim::world::motor_params_from_profile(prof));
    robot::iface::ActuatorCommand cmd;
    cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
    cmd.stamp_ns = 0;
    cmd.expiry_ns = 1'000'000'000;
    cmd.drive[0] = cmd.drive[1] = 1.0;
    tuned.apply(cmd);
    for (int i = 0; i < 200; ++i) tuned.step(0.02);
    // Settles at the retuned equilibrium (free 4.0: 54(1-v/4) = 1.18+0.6v
    // -> v ~= 3.75), not at the drag-free 4 m/s.
    assert(tuned.truth().forward_mps > 3.70 && tuned.truth().forward_mps < 3.80);
    assert(!eval::load_drive_profile(path + ".missing", prof, &err));
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out << "{\"kind\":\"nope\"}";
    }
    assert(!eval::load_drive_profile(path, prof, &err) && !err.empty());
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out << "{\"kind\":\"differential\",\"free_speed_mps\":-1.0}";
    }
    assert(!eval::load_drive_profile(path, prof, &err));
    std::filesystem::remove(path);
  }

  std::printf("providers ok\n");
  return 0;
}
