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
  assert(obs.odometry_valid && obs.scan_valid && obs.imu_valid);
  assert(obs.ranges_m.size() == 72);
  assert(robot::iface::valid_observation_frame(obs));
  assert(app.publish_observation(obs));

  // Motion endpoint closes the loop: command -> plant -> feedback.
  robot::iface::MotionCommand cmd;
  cmd.space = robot::iface::CommandSpace::kYawRate;
  cmd.stamp_ns = 20'000'000;
  cmd.expiry_ns = 220'000'000;
  cmd.forward_mps = 1.0;
  suite.consume(cmd);
  double x0 = world.truth().body_x_m;
  world.step(0.02);
  assert(world.truth().body_x_m - x0 > 0.019);

  // Feedback reflects the plant, and the envelope carries it through.
  robot::iface::ObservationFrame obs2;
  assert(suite.poll_envelope(obs2));
  assert(obs2.forward_mps == 1.0);

  std::printf("providers ok\n");
  return 0;
}
