#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "robot/iface/message.hpp"
#include "robot/providers/real_lidar2d.hpp"
#include "sim/providers/lidar2d.hpp"
#include "sim/providers/sim_imu.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

// WorldTruth is SIM-owned; tests set poses directly (never a ROBOT input).
sim::world::WorldTruth& mutable_truth(sim::world::World& w) {
  return const_cast<sim::world::WorldTruth&>(w.truth());
}

int argmin_range(const robot::iface::Lidar2DScan& scan) {
  int best = 0;
  for (std::size_t i = 1; i < scan.ranges_m.size(); ++i) {
    if (scan.ranges_m[i] < scan.ranges_m[static_cast<std::size_t>(best)]) {
      best = static_cast<int>(i);
    }
  }
  return best;
}

}  // namespace

int main() {
  // Sim2D: true 3D raycast from the sensor pose (body position + attitude
  // composed with mount extrinsics) against the engine scene. The engine
  // model builds on the first step, so arm a diff drive first.
  sim::world::World w;
  robot::iface::ActuatorCommand enable;
  enable.mode = robot::iface::ActuatorMode::kDiffDuty;
  w.apply(enable);
  w.step(0.02);
  sim::world::StaticBox box{2.0, -0.5, 3.0, 0.5, 1.0};  // dead ahead, 2 m out
  w.set_static_boxes({box});
  w.step(0.02);  // rebuild with the new geometry
  sim::providers::Sim2DLidarProvider sim2d(&w, "sim.lidar2d", 72, 10.0f);
  assert(sim2d.descriptor().backend == robot::iface::ProviderBackend::kSim);
  robot::iface::Lidar2DScan scan;
  assert(sim2d.poll(scan));
  assert(scan.valid());
  assert(scan.ranges_m.size() == 72);
  assert(scan.seq == 1);
  // Box center bearing 0 -> beam 36 (angle_min -pi + 36*2pi/72 = 0).
  const int hit = argmin_range(scan);
  assert(hit >= 35 && hit <= 37);
  assert(scan.ranges_m[static_cast<std::size_t>(hit)] < 3.0f);
  assert(scan.ranges_m[0] == 10.0f);  // behind: max range
  // Static-only pass renders geometry without actors.
  sim::world::Actor actor;
  actor.x = 1.0;
  actor.y = 0.1;
  actor.radius_m = 0.3;  // near edge at ~0.7: occludes the box behind it
  w.add_actor(actor);
  robot::iface::Lidar2DScan with_actor, static_only;
  assert(sim2d.poll(with_actor, true));
  assert(sim2d.poll(static_only, false));
  // Foreground vs background: the actor occludes the box behind it.
  assert(with_actor.ranges_m[static_cast<std::size_t>(hit)] <
         static_only.ranges_m[static_cast<std::size_t>(hit)]);
  assert(std::abs(static_only.ranges_m[static_cast<std::size_t>(hit)] - 2.0f) <
         0.05f);  // box face at x = 2.0
  assert(std::abs(with_actor.ranges_m[static_cast<std::size_t>(hit)] - 0.7f) <
         0.15f);  // actor disc near edge occludes it
  // Yaw-aware: facing north (+pi/2) moves the eastern dip by -18 beams.
  // (A world-frame fan would leave it at 36: the loop-closure deadlock.)
  mutable_truth(w).body_yaw_rad = M_PI_2;
  robot::iface::Lidar2DScan turned;
  assert(sim2d.poll(turned, false));
  const int hit2 = argmin_range(turned);
  assert(hit2 >= 17 && hit2 <= 21);
  mutable_truth(w).body_yaw_rad = 0.0;
  // Determinism: same truth, same rays (only the sequence advances).
  robot::iface::Lidar2DScan again;
  assert(sim2d.poll(again, false));
  assert(again.ranges_m == static_only.ranges_m);
  // Mount extrinsics: forward-mounted lidar sees the box face sooner.
  {
    const double xyz[3] = {0.3, 0.0, 0.0};
    const double rpy[3] = {0.0, 0.0, 0.0};
    sim2d.set_mount(xyz, rpy);
    robot::iface::Lidar2DScan moved;
    assert(sim2d.poll(moved, false));
    assert(std::abs(moved.ranges_m[static_cast<std::size_t>(hit)] - 1.7f) <
           0.05f);  // face at 2.0 - 0.3 mount
    // Yaw-mounted lidar rotates the scan: +pi/2 mount yaws every bearing.
    const double yaw_rpy[3] = {0.0, 0.0, M_PI_2};
    const double zero_xyz[3] = {0.0, 0.0, 0.0};
    sim2d.set_mount(zero_xyz, yaw_rpy);
    robot::iface::Lidar2DScan yawed;
    assert(sim2d.poll(yawed, false));
    const int hit3 = argmin_range(yawed);
    assert(hit3 >= 17 && hit3 <= 21);  // same -18 beam shift as body yaw
    // High mount over a low box: rays fly over, max range (2.5D for free).
    const double high_xyz[3] = {0.0, 0.0, 2.0};
    const double zero_rpy[3] = {0.0, 0.0, 0.0};
    sim2d.set_mount(high_xyz, zero_rpy);
    robot::iface::Lidar2DScan over;
    assert(sim2d.poll(over, false));
    assert(over.ranges_m[static_cast<std::size_t>(hit)] == 10.0f);
    // Non-finite mounts are rejected, previous mount kept.
    const double bad_xyz[3] = {std::numeric_limits<double>::quiet_NaN(), 0.0,
                               0.0};
    sim2d.set_mount(bad_xyz, zero_rpy);
    robot::iface::Lidar2DScan kept;
    assert(sim2d.poll(kept, false));
    assert(kept.ranges_m == over.ranges_m);
    sim2d.set_mount(zero_xyz, zero_rpy);
  }
  sim::providers::Sim2DLidarProvider null_world(nullptr, "x", 72, 10.0f);
  assert(!null_world.is_ready());
  assert(!null_world.poll(scan));

  // SimImu: truth yaw rate + specific force, exact by default.
  sim::providers::SimImuProvider imu(&w);
  mutable_truth(w).yaw_rate_radps = 0.5;
  robot::iface::ImuMessage m;
  assert(imu.poll(m));
  assert(m.valid());
  assert(m.gyro_z_radps == 0.5);
  assert(m.accel_z_mps2 == 9.81);
  // Realism switch: bias offsets exactly.
  sim::providers::ImuRealismConfig cfg;
  cfg.seed = 1;
  cfg.gyro_bias_radps = 0.1;
  imu.set_realism(cfg);
  assert(imu.poll(m));
  assert(m.gyro_z_radps == 0.6);
  // Mount extrinsics: orientation rotates body rates into the sensor.
  {
    sim::providers::SimImuProvider mounted(&w);
    const double xyz[3] = {0.0, 0.0, 0.0};
    const double yaw90[3] = {0.0, 0.0, M_PI_2};  // yaw commutes: z unchanged
    mounted.set_mount(xyz, yaw90);
    mutable_truth(w).yaw_rate_radps = 0.5;
    robot::iface::ImuMessage mm;
    assert(mounted.poll(mm));
    assert(mm.gyro_z_radps == 0.5);
    assert(mm.gyro_x_radps == 0.0 && mm.gyro_y_radps == 0.0);
    // Roll-mounted 90 deg: body yaw rate appears on the sensor y axis
    // (tolerances: rotation is exact math, floating-point application).
    const double roll90[3] = {M_PI_2, 0.0, 0.0};
    mounted.set_mount(xyz, roll90);
    assert(mounted.poll(mm));
    assert(std::abs(mm.gyro_y_radps - 0.5) < 1e-12);
    assert(std::abs(mm.gyro_z_radps) < 1e-12);
    assert(std::abs(mm.gyro_x_radps) < 1e-12);
    // Rotation preserves the rate norm for any mount.
    const double tilt[3] = {0.3, -0.2, 1.1};
    mounted.set_mount(xyz, tilt);
    assert(mounted.poll(mm));
    const double norm = std::sqrt(mm.gyro_x_radps * mm.gyro_x_radps +
                                  mm.gyro_y_radps * mm.gyro_y_radps +
                                  mm.gyro_z_radps * mm.gyro_z_radps);
    assert(std::abs(norm - 0.5) < 1e-12);
  }
  // Lever arm: offset mount + yaw acceleration reads centripetal + Euler.
  {
    sim::providers::SimImuProvider lever(&w);
    const double off[3] = {1.0, 0.0, 0.0};
    const double no_rot[3] = {0.0, 0.0, 0.0};
    lever.set_mount(off, no_rot);
    mutable_truth(w).yaw_rate_radps = 1.0;
    robot::iface::ImuMessage lm;
    assert(lever.poll(lm));  // first poll: no angular acceleration yet
    assert(lever.poll(lm));  // same rate: alpha = 0, centripetal only
    assert(std::abs(lm.accel_x_mps2 - -1.0) < 1e-9);  // -w^2 * rx
    assert(std::abs(lm.accel_y_mps2) < 1e-9);
    assert(std::abs(lm.accel_z_mps2 - 9.81) < 1e-9);
    w.step(0.02);  // advance the stamp so angular acceleration is observable
    mutable_truth(w).yaw_rate_radps = 0.5;  // alpha = (0.5-1)/0.02 = -25
    assert(lever.poll(lm));
    // Euler: -alpha*ry = 0 on x; alpha*rx = -25 on y. Centripetal -0.25.
    assert(std::abs(lm.accel_x_mps2 - -0.25) < 1e-9);
    assert(std::abs(lm.accel_y_mps2 - -25.0) < 1e-6);
    mutable_truth(w).yaw_rate_radps = 0.5;
  }

  // Real2D: same ISensorProvider seam, native backend, replay-fed.
  robot::providers::Real2DLidarProvider real("ro1.lidar",
                                             std::vector<float>(36, 4.0f),
                                             0.1f);
  assert(real.descriptor().backend == robot::iface::ProviderBackend::kNative);
  robot::iface::Lidar2DScan rscan;
  assert(!real.poll(rscan));  // nothing fed: no fabricated scan
  real.feed(200'000'000);
  assert(real.poll(rscan));
  assert(rscan.valid());
  assert(rscan.ranges_m.size() == 36);

  // Polymorphism: typed messages behind IMessage-compatible use.
  robot::iface::IMessage* msg = &scan;
  assert(msg->valid() && msg->sequence() >= 1);

  std::printf("lidar ok\n");
  return 0;
}
