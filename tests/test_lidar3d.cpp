#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "robot/iface/message.hpp"
#include "sim/providers/lidar3d.hpp"
#include "sim/providers/sensor_suite.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

// WorldTruth is SIM-owned; tests set poses directly (never a ROBOT input).
sim::world::WorldTruth& mutable_truth(sim::world::World& w) {
  return const_cast<sim::world::WorldTruth&>(w.truth());
}

void arm(sim::world::World& w) {
  robot::iface::ActuatorCommand enable;
  enable.mode = robot::iface::ActuatorMode::kDiffDuty;
  w.apply(enable);
  w.step(0.02);
}

}  // namespace

int main() {
  // 16-line producer: true 3D raycast per ring/azimuth from the sensor pose
  // against the engine scene. The engine model builds on the first step,
  // so arm a diff drive first.
  sim::world::World w;
  arm(w);
  sim::world::StaticBox box{2.0, -0.5, 3.0, 0.5, 1.0};  // dead ahead, 2 m out
  w.set_static_boxes({box});
  w.step(0.02);  // rebuild with the new geometry
  sim::providers::Sim3DLidarProvider sim3d(&w, "sim.lidar3d", 16, 72, 10.0f);
  assert(sim3d.descriptor().backend == robot::iface::ProviderBackend::kSim);
  assert(sim3d.descriptor().frame_id == "lidar_link");
  robot::iface::Lidar3DCloud cloud;
  assert(sim3d.poll(cloud));
  assert(cloud.valid());
  assert(cloud.ring_count == 16);
  assert(cloud.scan_period_s > 0.0f);
  assert(cloud.seq == 1);
  assert(!cloud.points.empty());
  // Box face at x = 2.0 (sensor at origin, zero mount): some returns land
  // on it; low rings hit the floor first (sensor-frame z < 0).
  bool saw_face = false, saw_floor = false;
  unsigned ring_mask = 0;
  for (const auto& p : cloud.points) {
    assert(p.ring < 16);
    assert(p.time_offset_s <= 0.0f &&
           p.time_offset_s >= -cloud.scan_period_s);
    assert(std::isfinite(p.x_m) && std::isfinite(p.y_m) &&
           std::isfinite(p.z_m));
    ring_mask |= (1u << p.ring);
    if (std::abs(p.x_m - 2.0f) < 0.05f && std::abs(p.y_m) < 0.5f &&
        p.z_m > -0.05f) {
      saw_face = true;
    }
    if (p.z_m < -0.05f) saw_floor = true;  // floor below the sensor
  }
  assert(saw_face && saw_floor);
  int rings_hit = 0;
  for (int j = 0; j < 16; ++j) rings_hit += (ring_mask >> j) & 1;
  assert(rings_hit >= 3);  // face + floor span several rings
  // Determinism: same truth, same points (only the sequence advances).
  robot::iface::Lidar3DCloud again;
  assert(sim3d.poll(again, false));
  assert(again.points.size() == cloud.points.size());
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    assert(again.points[i].x_m == cloud.points[i].x_m);
    assert(again.points[i].ring == cloud.points[i].ring);
  }
  // Static-only vs with-actor: an off-path actor adds returns the static
  // scene cannot explain (isolated bearing, no box there).
  sim::world::Actor actor;
  actor.x = 2.0;
  actor.y = 2.0;  // bearing 45 deg, clear of the box (within +-14 deg)
  actor.radius_m = 0.3;
  w.add_actor(actor);
  robot::iface::Lidar3DCloud with_actor, static_only;
  assert(sim3d.poll(with_actor, true));
  assert(sim3d.poll(static_only, false));
  auto near_actor = [](const robot::iface::Lidar3DCloud& c) {
    int n = 0;
    for (const auto& p : c.points) {
      if (std::hypot(p.x_m - 2.0f, p.y_m - 2.0f) < 0.5f) ++n;
    }
    return n;
  };
  assert(near_actor(with_actor) > 0);
  assert(near_actor(static_only) == 0);
  // Mount extrinsics: forward-mounted lidar sees the box face sooner.
  {
    const double xyz[3] = {0.3, 0.0, 0.0};
    const double rpy[3] = {0.0, 0.0, 0.0};
    sim3d.set_mount(xyz, rpy);
    robot::iface::Lidar3DCloud moved;
    assert(sim3d.poll(moved, false));
    bool saw_shifted = false;
    for (const auto& p : moved.points) {
      if (std::abs(p.x_m - 1.7f) < 0.05f && std::abs(p.y_m) < 0.5f &&
          p.z_m > -0.05f) {
        saw_shifted = true;
      }
    }
    assert(saw_shifted);  // face at 2.0 - 0.3 mount
    // Yaw-mounted lidar rotates the rays in the world: with a +90 deg
    // mount the world +x face lands on sensor-frame -y (points are
    // sensor-frame, like a real driver).
    const double yaw_rpy[3] = {0.0, 0.0, M_PI_2};
    const double zero_xyz[3] = {0.0, 0.0, 0.0};
    sim3d.set_mount(zero_xyz, yaw_rpy);
    robot::iface::Lidar3DCloud yawed;
    assert(sim3d.poll(yawed, false));
    bool saw_rotated = false;
    for (const auto& p : yawed.points) {
      if (p.y_m < -1.9f && p.y_m > -2.1f && std::abs(p.x_m) < 0.3f) {
        saw_rotated = true;
      }
    }
    assert(saw_rotated);
    // High mount over a low box: top rings fly over, fewer returns.
    sim::world::World low;
    arm(low);
    low.set_static_boxes({{2.0, -0.5, 3.0, 0.5, 0.2}});  // 0.2 m plate
    low.step(0.02);
    sim::providers::Sim3DLidarProvider high(&low, "h", 16, 72, 10.0f);
    const double high_xyz[3] = {0.0, 0.0, 2.0};
    const double zero_rpy[3] = {0.0, 0.0, 0.0};
    high.set_mount(high_xyz, zero_rpy);
    robot::iface::Lidar3DCloud over;
    assert(high.poll(over, false));
    sim::providers::Sim3DLidarProvider level(&low, "l", 16, 72, 10.0f);
    robot::iface::Lidar3DCloud flat;
    assert(level.poll(flat, false));
    assert(over.points.size() < flat.points.size());
    // Non-finite mounts are rejected, previous mount kept.
    const double bad_xyz[3] = {std::numeric_limits<double>::quiet_NaN(), 0.0,
                               0.0};
    high.set_mount(bad_xyz, zero_rpy);
    robot::iface::Lidar3DCloud kept;
    assert(high.poll(kept, false));
    assert(kept.points.size() == over.points.size());
    sim3d.set_mount(zero_xyz, zero_rpy);
  }
  // Geometry tuning: fail-closed on bad input, applied on good.
  {
    sim::providers::Sim3DLidarProvider t(&w, "t", 16, 72, 10.0f);
    t.set_rings(0);
    t.set_azimuths(-1);
    t.set_max_range(-2.0f);
    t.set_scan_period(0.0f);
    t.set_vertical_fov(0.5, -0.5);
    robot::iface::Lidar3DCloud c;
    assert(t.poll(c) && c.ring_count == 16);  // all rejected, defaults kept
    t.set_rings(4);
    t.set_azimuths(36);
    assert(t.poll(c) && c.ring_count == 4);
    for (const auto& p : c.points) assert(p.ring < 4);
  }
  sim::providers::Sim3DLidarProvider null_world(nullptr, "x", 16, 72, 10.0f);
  assert(!null_world.is_ready());
  assert(!null_world.poll(cloud));
  // Empty cloud is invalid (misses are dropped, never filled).
  {
    robot::iface::Lidar3DCloud empty;
    assert(!empty.valid());
  }

  // Suite wiring: lidar_link mounts reach the 3D producer.
  {
    sim::world::World sw;
    arm(sw);
    sw.set_static_boxes({box});
    sw.step(0.02);
    sim::providers::SimSensorSuite suite(&sw);
    robot::platform::SensorMount m;
    m.sensor_id = "ro1.lidar3d";
    m.link = "lidar_link";
    m.xyz_m[0] = 0.3;
    suite.set_mounts({m});
    robot::iface::Lidar3DCloud a, b;
    assert(suite.lidar3d().poll(a, false));
    bool shifted = false;
    for (const auto& p : a.points) {
      if (std::abs(p.x_m - 1.7f) < 0.05f && std::abs(p.y_m) < 0.5f) {
        shifted = true;
      }
    }
    assert(shifted);
    (void)b;
  }

  std::printf("lidar3d ok\n");
  return 0;
}
