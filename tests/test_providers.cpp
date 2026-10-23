#include <cassert>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "robot/app/robot_application.hpp"
#include "sim/eval/drive_profile_io.hpp"
#include "sim/providers/sensor_suite.hpp"
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
  assert(obs.odometry_valid && obs.scan_valid && obs.scan_static_valid &&
         obs.imu_valid);
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
