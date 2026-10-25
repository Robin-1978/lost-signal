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

  sim::world::World world;
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
  // ...and firing when stall force exceeds mu * load (low mu, standstill).
  // Confirmation hysteresis: one transient step never latches the label.
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
  wslip.step(0.02);
  assert(!wslip.truth().drive_slipping[0] &&
         !wslip.truth().drive_slipping[1]);
  wslip.step(0.02);
  assert(wslip.truth().drive_slipping[0] &&
         wslip.truth().drive_slipping[1]);

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
