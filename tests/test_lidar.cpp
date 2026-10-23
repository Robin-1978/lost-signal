#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "robot/iface/message.hpp"
#include "robot/providers/real_lidar2d.hpp"
#include "sim/providers/lidar2d.hpp"
#include "sim/providers/lidar3d.hpp"
#include "sim/providers/reflectivity.hpp"
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
  // Realism switch: trim offsets exactly, per axis.
  sim::providers::ImuRealismConfig cfg;
  cfg.seed = 1;
  cfg.gyro_trim_radps[2] = 0.1;
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
  // Per-axis bias draw + walk (seeded device imperfection, never truth).
  {
    mutable_truth(w).yaw_rate_radps = 0.0;
    sim::providers::SimImuProvider a(&w), b(&w);
    sim::providers::ImuRealismConfig dc;
    dc.seed = 77;
    dc.gyro_bias_stddev_radps = 0.01;
    a.set_realism(dc);
    b.set_realism(dc);
    robot::iface::ImuMessage ma, mb;
    assert(a.poll(ma) && b.poll(mb));
    // Same seed: identical per-axis draws (exact replay, not statistical).
    assert(ma.gyro_x_radps == mb.gyro_x_radps);
    assert(ma.gyro_y_radps == mb.gyro_y_radps);
    assert(ma.gyro_z_radps == mb.gyro_z_radps);
    // Independent per-axis draws (vanishing chance all three coincide).
    assert(ma.gyro_x_radps != ma.gyro_y_radps ||
           ma.gyro_y_radps != ma.gyro_z_radps);
    // Drawn bias is bounded (6 sigma) and rides all axes, not just z.
    assert(std::abs(ma.gyro_x_radps) <= 0.06);
    assert(std::abs(ma.gyro_z_radps) <= 0.06);
    // Random walk accumulates across steps (seeded, deterministic).
    sim::providers::ImuRealismConfig wk = dc;
    wk.gyro_bias_walk_radps_sqrt_s = 0.01;
    a.set_realism(wk);
    b.set_realism(wk);
    assert(a.poll(ma));
    const double z0 = ma.gyro_z_radps;
    for (int i = 0; i < 50; ++i) {
      w.step(0.02);
      assert(a.poll(ma) && b.poll(mb));
      assert(ma.gyro_z_radps == mb.gyro_z_radps);  // streams stay locked
    }
    assert(ma.gyro_z_radps != z0);  // walk moved the bias
    assert(std::abs(ma.gyro_z_radps) <= 6.0 * 0.06);  // bounded drift
    // Invalid config rejected (fail closed): negative walk keeps behavior.
    sim::providers::ImuRealismConfig bad = wk;
    bad.gyro_bias_walk_radps_sqrt_s = -1.0;
    a.set_realism(bad);
    assert(a.poll(ma));
    assert(ma.gyro_z_radps == mb.gyro_z_radps);  // previous stream kept
  }
  // 3D lever arm: z-offset under yaw spin reads no phantom (centripetal
  // and Euler arms vanish along the spin axis); gravity norm is preserved
  // under any mount orientation.
  {
    mutable_truth(w).yaw_rate_radps = 1.0;
    sim::providers::SimImuProvider up(&w);
    const double zoff[3] = {0.0, 0.0, 1.0};
    const double no_rot[3] = {0.0, 0.0, 0.0};
    up.set_mount(zoff, no_rot);
    robot::iface::ImuMessage um;
    assert(up.poll(um));
    assert(up.poll(um));
    assert(std::abs(um.accel_x_mps2) < 1e-9);
    assert(std::abs(um.accel_y_mps2) < 1e-9);
    assert(std::abs(um.accel_z_mps2 - 9.81) < 1e-9);
    // Level at rest: no phantom roll/pitch rates from the attitude delta
    // (quaternion-delta residue is numerically zero, never a live rate).
    mutable_truth(w).yaw_rate_radps = 0.0;
    w.step(0.02);
    assert(up.poll(um));
    assert(std::abs(um.gyro_x_radps) < 1e-9);
    assert(std::abs(um.gyro_y_radps) < 1e-9);
    const double tilt[3] = {0.3, -0.2, 1.1};
    up.set_mount(zoff, tilt);
    assert(up.poll(um));
    const double anorm = std::sqrt(um.accel_x_mps2 * um.accel_x_mps2 +
                                   um.accel_y_mps2 * um.accel_y_mps2 +
                                   um.accel_z_mps2 * um.accel_z_mps2);
    assert(std::abs(anorm - 9.81) < 1e-9);
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

  // Reflectivity (SIM-side synthesis, measurement-only downstream):
  // helper pins first, then provider beams, then the envelope carry.
  {
    using sim::providers::detail::return_intensity;
    using sim::providers::detail::surface_reflectivity;
    sim::world::World wr;
    assert(std::abs(surface_reflectivity(wr, 0.0, 0.0, 0.0) - 0.30) < 1e-12);
    wr.set_materials({{"ice", 0.2, 0.005, 0.006, 1.0, 0.1}});
    wr.set_material_zones({{"slick", -5.0, -5.0, 5.0, 5.0, "ice"}});
    assert(std::abs(surface_reflectivity(wr, 0.0, 0.0, 0.0) - 0.1) < 1e-12);
    assert(std::abs(surface_reflectivity(wr, 9.0, 9.0, 0.0) - 0.30) < 1e-12);
    wr.set_static_boxes({{2.0, -0.5, 3.0, 0.5, 1.0}});
    assert(std::abs(surface_reflectivity(wr, 2.5, 0.0, 0.5) - 0.68) < 1e-12);
    // Attenuation: nearer is brighter; misses read exactly 0. Head-on rays
    // (dir into the surface normal) keep the legacy values exactly.
    const float near =
        return_intensity(wr, 0.0, 0.0, 0.0, 1.0, false, false, 0.0, 0.0, -1.0,
                         0.0, 0.0, 0.0);
    const float far =
        return_intensity(wr, 0.0, 0.0, 0.0, 5.0, false, false, 0.0, 0.0, -1.0,
                         0.0, 0.0, 0.0);
    assert(near > far && far > 0.0f);
    assert(std::abs(near - 0.1f / (1.0f + 0.015f)) < 1e-6);
    assert(return_intensity(wr, 0.0, 0.0, 0.0, 2.0, false, true, 0.0, 0.0,
                            -1.0, 0.0, 0.0, 0.0) == 0.0f);
    const float actor = return_intensity(wr, -0.4, 0.0, 0.5, 2.0, true, false,
                                         1.0, 0.0, 0.0, 0.0, 0.0, 1.5);
    assert(std::abs(actor - 0.52f / (1.0f + 0.015f * 4.0f)) < 1e-6);
    // Incidence (master's angular response, floor 0.15): a level ray
    // grazing flat ground dims to the floor; head-on reads full.
    const float graze = return_intensity(wr, 0.0, 0.0, 0.0, 1.0, false, false,
                                         1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    assert(std::abs(graze - near * 0.15f) < 1e-6);
    // Box faces: head-on front face full, tangent ray floored, top full.
    const float face =
        return_intensity(wr, 2.0, 0.0, 0.5, 2.0, false, false, 1.0, 0.0, 0.0,
                         0.0, 0.0, 0.0);
    assert(std::abs(face - 0.68f / (1.0f + 0.015f * 4.0f)) < 1e-6);
    const float edge =
        return_intensity(wr, 2.0, 0.0, 0.5, 2.0, false, false, 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0);
    assert(std::abs(edge - face * 0.15f) < 1e-6);
    const float top =
        return_intensity(wr, 2.5, 0.0, 1.0, 2.0, false, false, 0.0, 0.0, -1.0,
                         0.0, 0.0, 0.0);
    assert(std::abs(top - 0.68f / (1.0f + 0.015f * 4.0f)) < 1e-6);
    // Actor wall: head-on side full, tangent floored (same floor doctrine).
    const float side =
        return_intensity(wr, 0.4, 0.0, 0.5, 2.0, true, false, 0.0, -1.0, 0.0,
                         0.0, 0.0, 1.5);
    assert(std::abs(side - actor * 0.15f) < 1e-6);
  }
  // Provider beams: box return strong, miss zero, sizes parallel.
  {
    sim::world::World wb;
    robot::iface::ActuatorCommand enable;
    enable.mode = robot::iface::ActuatorMode::kDiffDuty;
    wb.apply(enable);
    wb.step(0.02);
    wb.set_static_boxes({{2.0, -0.5, 3.0, 0.5, 1.0}});
    wb.step(0.02);
    sim::providers::Sim2DLidarProvider sim2d(&wb, "sim.lidar2d", 72, 10.0f);
    robot::iface::Lidar2DScan s2;
    assert(sim2d.poll(s2));
    assert(s2.intensities.size() == s2.ranges_m.size());
    const int hit = argmin_range(s2);
    assert(s2.ranges_m[static_cast<std::size_t>(hit)] < 3.0f);
    const float expect =
        0.68f /
        (1.0f + 0.015f * s2.ranges_m[static_cast<std::size_t>(hit)] *
                    s2.ranges_m[static_cast<std::size_t>(hit)]);
    assert(std::abs(s2.intensities[static_cast<std::size_t>(hit)] - expect) <
           1e-5);
    // A side beam missing everything reads max range with zero intensity.
    assert(s2.intensities[0] == 0.0f);
    // 3D cloud: same model per point (no constant-1.0 fill).
    sim::providers::Sim3DLidarProvider sim3d(&wb, "sim.lidar3d", 16, 72,
                                             10.0f);
    robot::iface::Lidar3DCloud cloud;
    assert(sim3d.poll(cloud));
    assert(!cloud.points.empty());
    bool strong_seen = false;
    for (const auto& p : cloud.points) {
      assert(p.intensity >= 0.0f && p.intensity <= 1.0f);
      strong_seen |= p.intensity > 0.3f;  // box face up close
    }
    assert(strong_seen);
    // Composite carries the winning beam's intensity per azimuth.
    std::vector<float> cr, ci;
    assert(sim3d.poll_composite(cr, ci, true));
    assert(cr.size() == 72 && ci.size() == 72);
    assert(ci[static_cast<std::size_t>(hit)] > 0.3f);
    // 3D device model: dropout 1.0 drops every point — the message-layer
    // representation is exactly an all-miss scene (empty cloud reads
    // invalid per the standing contract, same as legacy with no geometry).
    sim::providers::LidarDeviceModel drop3d;
    drop3d.dropout_probability = 1.0;
    sim3d.set_device_model(drop3d);
    robot::iface::Lidar3DCloud empty;
    assert(!sim3d.poll(empty));
    assert(empty.points.empty());
    // The composite front end (what the suite consumes) stays valid with
    // all-max-range: no-return = blanked, never an invented boundary hit.
    std::vector<float> dr, di;
    assert(sim3d.poll_composite(dr, di, true));
    for (float r : dr) assert(r == 10.0f);
    for (float s : di) assert(s == 0.0f);
  }

  // Rolling shutter (SIM-side generation): offsets ride parallel, beam 0
  // oldest at -period, last beam at the stamp; frozen scans read zeros.
  {
    sim::world::World w;
    robot::iface::ActuatorCommand enable;
    enable.mode = robot::iface::ActuatorMode::kDiffDuty;
    w.apply(enable);
    w.step(0.02);
    w.set_static_boxes({{2.0, -0.5, 3.0, 0.5, 1.0}});
    w.step(0.02);
    sim::providers::Sim2DLidarProvider sim2d(&w, "sim.lidar2d", 72, 10.0f);
    robot::iface::Lidar2DScan still;
    assert(sim2d.poll(still));
    assert(still.time_offsets_s.size() == 72);
    assert(std::abs(still.time_offsets_s[0] + 0.1f) < 1e-6);
    assert(std::abs(still.time_offsets_s[71]) < 0.002f);
    // Spinning truth distorts the scan (same pose, yaw rate only).
    mutable_truth(w).yaw_rate_radps = 2.0;
    robot::iface::Lidar2DScan spin;
    assert(sim2d.poll(spin));
    bool differ = false;
    for (std::size_t i = 0; i < 72; ++i) {
      if (std::abs(spin.ranges_m[i] - still.ranges_m[i]) > 1e-6) {
        differ = true;
        break;
      }
    }
    assert(differ);  // 0.2 rad of shutter skew moves box-edge returns
    mutable_truth(w).yaw_rate_radps = 0.0;
    robot::iface::Lidar2DScan again;
    assert(sim2d.poll(again));
    assert(again.ranges_m == still.ranges_m);  // deterministic
    // Frozen period: legacy behavior, zero offsets, scan matches still.
    sim2d.set_scan_period(0.0);
    robot::iface::Lidar2DScan frozen;
    assert(sim2d.poll(frozen));
    for (float t : frozen.time_offsets_s) assert(t == 0.0f);
    sim2d.set_scan_period(-1.0);  // rejected: previous kept
    robot::iface::Lidar2DScan kept;
    assert(sim2d.poll(kept));
    assert(kept.time_offsets_s == frozen.time_offsets_s);
  }

  // Device observation model (SIM-side synthesis, seeded determinism).
  // Scene above: box dead ahead (~2 m), open sky behind (max range).
  {
    sim::providers::Sim2DLidarProvider dev(&w, "sim.lidar2d", 72, 10.0f);
    dev.set_scan_period(0.0);  // frozen: isolate the device layer
    robot::iface::Lidar2DScan exact;
    assert(dev.poll(exact));
    // Invalid model rejected (fail closed): negative stddev keeps exact.
    sim::providers::LidarDeviceModel bad;
    bad.range_stddev_m = -1.0;
    dev.set_device_model(bad);
    robot::iface::Lidar2DScan still_exact;
    assert(dev.poll(still_exact));
    assert(still_exact.ranges_m == exact.ranges_m);
    // Bias shifts hits, misses stay absent (never filled, never clamped).
    sim::providers::LidarDeviceModel bias;
    bias.range_bias_m = 0.5;
    dev.set_device_model(bias);
    dev.set_noise_seed(42);
    robot::iface::Lidar2DScan biased;
    assert(dev.poll(biased));
    const int hub = argmin_range(exact);
    assert(std::abs(biased.ranges_m[static_cast<std::size_t>(hub)] -
                    (exact.ranges_m[static_cast<std::size_t>(hub)] + 0.5f)) <
           1e-6);
    assert(biased.ranges_m[0] == 10.0f);  // miss stays miss
    // Quantization snaps hits to the grid.
    sim::providers::LidarDeviceModel quant;
    quant.range_resolution_m = 1.0;
    dev.set_device_model(quant);
    dev.set_noise_seed(7);
    robot::iface::Lidar2DScan snapped;
    assert(dev.poll(snapped));
    for (float r : snapped.ranges_m) {
      if (r < 10.0f) assert(std::abs(r - std::round(r)) < 1e-6);
    }
    // White noise: seeded repeat reproduces, stream differs from exact,
    // misses stay absent.
    sim::providers::LidarDeviceModel noisy;
    noisy.range_stddev_m = 0.05;
    dev.set_device_model(noisy);
    dev.set_noise_seed(1234);
    robot::iface::Lidar2DScan n1;
    assert(dev.poll(n1));
    dev.set_noise_seed(1234);
    robot::iface::Lidar2DScan n2;
    assert(dev.poll(n2));
    assert(n1.ranges_m == n2.ranges_m);  // deterministic
    assert(n1.ranges_m != exact.ranges_m);
    assert(n1.ranges_m[0] == 10.0f);
    // Dropout 1.0: every hit becomes a no-return with zero intensity —
    // no obstacle invented at any boundary.
    sim::providers::LidarDeviceModel drop;
    drop.dropout_probability = 1.0;
    dev.set_device_model(drop);
    dev.set_noise_seed(99);
    robot::iface::Lidar2DScan gone;
    assert(dev.poll(gone));
    for (float r : gone.ranges_m) assert(r == 10.0f);
    for (float s : gone.intensities) assert(s == 0.0f);
    // Near-field: a 5 m blind zone blanks the 2 m box, never clamps it
    // to 5 m (no invented obstacle at the boundary).
    sim::providers::LidarDeviceModel blind;
    blind.min_range_m = 5.0;
    dev.set_device_model(blind);
    robot::iface::Lidar2DScan blanked;
    assert(dev.poll(blanked));
    for (float r : blanked.ranges_m) assert(r == 10.0f || r >= 5.0f);
    // Intensity survives device range noise (synthesized at the ideal
    // surface); only driven no-returns zero it.
    dev.set_device_model(noisy);
    dev.set_noise_seed(1234);
    robot::iface::Lidar2DScan ni;
    assert(dev.poll(ni));
    assert(ni.intensities[static_cast<std::size_t>(hub)] > 0.0f);
  }

  std::printf("lidar ok\n");
  return 0;
}
