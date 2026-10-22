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
  // Force integration: one 20 ms step at stall accel (54 N / 3 kg) reaches
  // 0.36 m/s, so displacement is small but strictly positive.
  const double dx1 = world.truth().body_x_m - x0;
  assert(dx1 > 0.005 && dx1 < 0.02);
  assert(world.truth().drive_radps[0] > 0.0);
  // Settles near free speed after several time constants.
  for (int i = 0; i < 100; ++i) world.step(0.02);
  assert(world.truth().forward_mps > 2.9);

  // Feedback reflects the plant, and the envelope carries it through.
  robot::iface::ObservationFrame obs2;
  assert(suite.poll_envelope(obs2));
  assert(obs2.drive_count == 2);
  assert(obs2.forward_mps > 2.9);

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
    assert(tuned.truth().forward_mps > 3.9);  // settles at 4 m/s, not 3
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
