#include <cassert>
#include <cstdio>

#include "robot/app/robot_application.hpp"
#include "sim/providers/sensor_suite.hpp"
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
  cmd.stamp_ns = 20'000'000;
  cmd.expiry_ns = 220'000'000;
  cmd.left = 1.0;
  cmd.right = 1.0;
  assert(robot::iface::valid_actuator_command(cmd, 20'000'000));
  suite.consume(cmd);
  double x0 = world.truth().body_x_m;
  world.step(0.02);
  // First-order lag: one 20 ms step at tau=0.1 s reaches ~18% of the
  // 3 m/s free speed, so displacement is small but strictly positive.
  const double dx1 = world.truth().body_x_m - x0;
  assert(dx1 > 0.005 && dx1 < 0.02);
  assert(world.truth().left_radps > 0.0);
  // Settles near free speed after several time constants.
  for (int i = 0; i < 100; ++i) world.step(0.02);
  assert(world.truth().forward_mps > 2.9);

  // Feedback reflects the plant, and the envelope carries it through.
  robot::iface::ObservationFrame obs2;
  assert(suite.poll_envelope(obs2));
  assert(obs2.wheels_valid);
  assert(obs2.forward_mps > 2.9);

  std::printf("providers ok\n");
  return 0;
}
