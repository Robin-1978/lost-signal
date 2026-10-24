// Engine behavior gate: the plant must reproduce deployable behavior.
// Pins are behavioral (moves, stops, holds grade, deterministic); exact
// transients belong to the effort-model unit tests.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <vector>

#include "robot/iface/task.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

sim::world::World make_mujoco_diff() { return sim::world::World(); }

void set_diff_duty(sim::world::World& w, double duty) {
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 0;
  cmd.expiry_ns = 1'000'000'000;
  cmd.drive[0] = cmd.drive[1] = duty;
  w.apply(cmd);
}

}  // namespace

int main() {
#ifdef LOST_SIGNAL_WITH_MUJOCO
  // Spin-up: moves, settles below free speed, level attitude.
  {
    sim::world::World w = make_mujoco_diff();
    set_diff_duty(w, 1.0);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(std::isfinite(w.truth().forward_mps));
    assert(w.truth().forward_mps > 1.0);  // pulls hard
    assert(w.truth().forward_mps < 3.5);  // drag-limited, never above free
    assert(std::abs(w.truth().body_roll_rad) < 0.05);
    assert(std::abs(w.truth().body_pitch_rad) < 0.05);
    assert(w.truth().body_z_m > 0.0);  // off the ground plane, on wheels
 // Passive supports are a real SIM assembly: the wheel position includes
 // configured trail and both joint angles are finite engine read-back.
 sim::world::CasterViz casters[4];
 const int caster_count = w.casters(casters, 4);
 assert(caster_count == 2);
 for (int i = 0; i < caster_count; ++i) {
 assert(std::isfinite(casters[i].swivel_rad));
 assert(std::isfinite(casters[i].roll_rad));
 assert(std::abs(casters[i].radius_m - 0.03) < 1e-12);
 const double ex = casters[i].pivot_x_m - 0.02 *
 std::cos(casters[i].swivel_rad);
 const double ey = casters[i].pivot_y_m - 0.02 *
 std::sin(casters[i].swivel_rad);
 assert(std::abs(casters[i].x_m - ex) < 1e-12);
 assert(std::abs(casters[i].y_m - ey) < 1e-12);
 }
 // A compatible geometry rebuild must not restore the old root height after
 // the wheel radius or authored terrain changes. The base frame is the drive
 // axle frame, so the new minimum is terrain height + new wheel radius.
 {
 sim::world::World w = make_mujoco_diff();
 w.set_terrain_patches({{"raised", -2.0, -2.0, 2.0, 2.0, 0.25,
 0.0, 0.0, 0.0, 0.0}});
 set_diff_duty(w, 0.0);
 w.step(0.02);
 // MuJoCo's compliant contact settles within a sub-millimetre of the
 // canonical plane; the renderer's documented guard covers that tolerance.
 assert(w.truth().body_z_m >= 0.25 + 0.1 - 0.001);
 sim::world::MotorParams rebuilt;
 rebuilt.wheel_radius_m = 0.16;
 w.set_motor_params(rebuilt); // forces a model rebuild with a larger wheel
 w.step(0.02);
 assert(w.truth().body_z_m >= 0.25 + 0.16 - 0.001);
 }

 // Coast: joint damping bleeds speed over tens of seconds; over 100
    // steps it glides. Pin no-energy-creation and no-reversal, not a
    // decay rate — the engine owns the transient.
    set_diff_duty(w, 0.0);
    const double v0 = w.truth().forward_mps;
    for (int i = 0; i < 100; ++i) w.step(0.02);
    assert(w.truth().forward_mps <= v0 + 1e-3);
    assert(w.truth().forward_mps >= 0.0);
  }

  // Slope P1 on the engine (mirrors test_contact, engine tolerances:
  // contact stiction, never exact zeros — the engine owns the creep).
  {
    // Gentle incline, zero duty: bounded creep, never the P1 runaway.
    sim::world::World g = make_mujoco_diff();
    g.set_incline(0.005);
    set_diff_duty(g, 0.0);
    for (int i = 0; i < 200; ++i) g.step(0.02);
    assert(std::abs(g.truth().body_x_m) < 1.0);
    assert(std::abs(g.truth().forward_mps) < 0.5);
    // Steep grade, zero duty: breaks away backward, stays backward.
    sim::world::World steep = make_mujoco_diff();
    steep.set_incline(0.3);
    set_diff_duty(steep, 0.0);
    for (int i = 0; i < 100; ++i) steep.step(0.02);
    assert(steep.truth().body_x_m < -1.0);
    assert(steep.truth().forward_mps < -0.5);
    // Service brake holds the steep grade once its lag ramps in.
    sim::world::MotorParams mp;
    mp.brake_max_torque_Nm = 5.0;
    sim::world::World held = make_mujoco_diff();
    held.set_motor_params(mp);
    held.set_incline(0.3);
    set_diff_duty(held, 0.0);
    held.set_brake_request(0, 1.0);
    held.set_brake_request(1, 1.0);
    for (int i = 0; i < 200; ++i) held.step(0.02);
    assert(held.truth().body_x_m > -0.5);
    assert(std::abs(held.truth().forward_mps) < 0.15);
    // Non-finite grade fails closed to flat, never NaN-poisons.
    held.set_incline(std::numeric_limits<double>::quiet_NaN());
    held.step(0.02);
    assert(std::isfinite(held.truth().body_x_m));
    assert(std::isfinite(held.truth().forward_mps));
    // Slope launch: held on grade by brake, duty on + brake off climbs
    // without sustained rollback.
    sim::world::World launch = make_mujoco_diff();
    launch.set_motor_params(mp);
    launch.set_incline(0.1);
    set_diff_duty(launch, 0.0);
    launch.set_brake_request(0, 1.0);
    launch.set_brake_request(1, 1.0);
    for (int i = 0; i < 100; ++i) launch.step(0.02);
    const double x_pre = launch.truth().body_x_m;
    set_diff_duty(launch, 1.0);
    launch.set_brake_request(0, 0.0);
    launch.set_brake_request(1, 0.0);
    double x_min = x_pre;
    for (int i = 0; i < 200; ++i) {
      launch.step(0.02);
      x_min = std::min(x_min, launch.truth().body_x_m);
    }
    assert(x_min >= x_pre - 0.05);  // no sustained rollback on release
    assert(launch.truth().body_x_m > x_pre + 1.0);  // climbs the grade
    assert(launch.truth().forward_mps > 0.5);
  }

  // Determinism: identical runs, identical truth (bit-identical replay).
  {
    auto run = []() {
      sim::world::World w = make_mujoco_diff();
      set_diff_duty(w, 0.7);
      std::vector<double> xs;
      for (int i = 0; i < 100; ++i) {
        w.step(0.02);
        xs.push_back(w.truth().body_x_m);
      }
      return xs;
    };
    const auto a = run();
    const auto b = run();
    assert(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) assert(a[i] == b[i]);
  }

  // Other chassis on the engine: ackermann left arc, mecanum strafe,
  // swerve straight. Engine-appropriate tolerances throughout (no exact
  // numbers — the engine owns the transients).
  {
    // Ackermann drives a left arc.
    sim::world::World w;
    sim::world::MotorParams mp;
    mp.wheel_base_m = 0.62;
    mp.track_width_m = 0.58;
    w.set_motor_params(mp);
    robot::iface::ActuatorCommand cmd;
    cmd.mode = robot::iface::ActuatorMode::kAckermannDuty;
    cmd.stamp_ns = 0;
    cmd.expiry_ns = 1'000'000'000;
    cmd.drive[0] = cmd.drive[1] = 0.5;
    cmd.steer[0] = cmd.steer[1] = 0.5;
    w.apply(cmd);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(w.truth().forward_mps > 0.5);
    assert(w.truth().yaw_rate_radps > 0.05);  // positive steer -> left arc
    assert(w.truth().body_yaw_rad > 0.0);
    // Mecanum strafes left with ~zero forward.
    sim::world::World m;
    robot::iface::ActuatorCommand mc;
    mc.mode = robot::iface::ActuatorMode::kMecanumDuty;
    mc.stamp_ns = 0;
    mc.expiry_ns = 1'000'000'000;
    mc.drive = {-0.5, 0.5, 0.5, -0.5};
    m.apply(mc);
    for (int i = 0; i < 200; ++i) m.step(0.02);
    assert(std::abs(m.truth().forward_mps) < 0.3);
    assert(m.truth().lateral_mps > 0.3);
    assert(m.truth().body_y_m > 0.1);
    // Swerve drives straight with centered modules.
    sim::world::World s;
    robot::iface::ActuatorCommand sc;
    sc.mode = robot::iface::ActuatorMode::kSwerveDuty;
    sc.stamp_ns = 0;
    sc.expiry_ns = 1'000'000'000;
    sc.drive = {0.5, 0.5, 0.5, 0.5};
    s.apply(sc);
    for (int i = 0; i < 200; ++i) s.step(0.02);
    assert(s.truth().forward_mps > 0.5);
    assert(std::abs(s.truth().yaw_rate_radps) < 0.3);
    // Slip telemetry stays quiet in normal driving (reported, not clamped).
    assert(!w.truth().drive_slipping[0] && !w.truth().drive_slipping[1]);
  }

  // HIL mission on the engine: crossing crosser, hold, resume, arrive.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"dock", 6.0, 0.0, 0.0});
    sim::world::Actor crosser;
    crosser.x = 2.0;
    crosser.y = -1.5;
    crosser.vx_mps = 0.0;
    crosser.vy_mps = 1.0;
    crosser.radius_m = 0.4;
    session.setup_world().add_actor(crosser);
    robot::iface::TaskRequest req;
    req.request_id = "crossing";
    req.place_id = "dock";
    assert(session.request_goal(req).valid());
    std::vector<double> xs;
    for (int i = 0; i < 280; ++i) {
      session.step_once();
      xs.push_back(session.episode().steps().back().truth_after.body_x_m);
    }
    assert(xs[8] > 0.005);  // drove toward the goal initially
    bool held = false;
    for (std::size_t i = 20; i < 120 && i < xs.size(); ++i) {
      if (xs[i] < 0.5) held = true;
    }
    assert(held);  // held while the actor was near
    assert(xs.back() > 2.0);  // resumed after the actor passed
  }

 // Friction-pair audit (drawbar doctrine): full-duty launch from
 // standstill, slip label watched over the run. Stall 54 N against
 // 117.7 N/wheel load: ice (0.15 -> 17.7 N) must slip, the 0.7 floor
 // (82 N) must hold, and a worn tire (0.4 -> 47 N) must slip on both
 // ordinary and grippy ground. The tire geom owns contact priority, so
 // MuJoCo enforces min(tire, ground) instead of its default max rule.
  {
    auto any_slip = [](sim::world::World& w, int steps) {
      bool slipped = false;
      for (int i = 0; i < steps; ++i) {
        w.step(0.02);
        for (bool s : w.truth().drive_slipping) slipped |= s;
      }
      return slipped;
    };
    // Slick zone pulls DOWN through the tire rating (the min() on the
    // tire side; without it the pair would read max(0.7, 0.15) = 0.7).
    {
      sim::world::World w = make_mujoco_diff();
      w.set_materials({{"ice", 0.15, 0.005, 0.006, 1.0, 0.3}});
      w.set_material_zones({{"belt", -2.0, -3.0, 8.0, 3.0, "ice"}});
      set_diff_duty(w, 1.0);
      assert(any_slip(w, 100));
    }
    // Bare floor holds the same launch (control: demand < capacity).
    {
      sim::world::World w = make_mujoco_diff();
      set_diff_duty(w, 1.0);
      assert(!any_slip(w, 200));
    }
    // Worn tire governs off-zone (rating path works where allowed).
    {
      sim::world::World w = make_mujoco_diff();
      sim::world::MotorParams mp;
      mp.tire_friction_mu = 0.4;
      w.set_motor_params(mp);
      set_diff_duty(w, 1.0);
      assert(any_slip(w, 100));
    }
 // Worn tire remains the limit on grippy ground: asphalt cannot repair
 // tire rubber, and the slip evidence follows that same physical pair.
    {
      sim::world::World w = make_mujoco_diff();
      sim::world::MotorParams mp;
      mp.tire_friction_mu = 0.4;
      w.set_motor_params(mp);
      w.set_materials({{"asphalt", 1.0, 0.005, 0.006, 1.0, 0.3}});
      w.set_material_zones({{"pad", -2.0, -3.0, 8.0, 3.0, "asphalt"}});
      set_diff_duty(w, 1.0);
 assert(any_slip(w, 200));
    }
  }

  // Chassis hard geometry (config, never constants): axle offset lands in
  // the wheel table, explicit supports build and stand, bad geometry
  // refuses with frozen truth (same fail-closed contract as bad zones).
  {
    // Axle offset shifts the drive pair (default 0 = legacy plant).
    sim::world::World w = make_mujoco_diff();
    sim::world::MotorParams mp;
    mp.drive_axle_x_m = 0.1;
    w.set_motor_params(mp);
    set_diff_duty(w, 0.5);
    for (int i = 0; i < 10; ++i) w.step(0.02);
    sim::world::WheelViz table[6];
    const int n = w.wheels(table, 6);
    assert(n == 2);
    assert(table[0].x_m == 0.1 && table[1].x_m == 0.1);
    // Explicit single rear support (tripod-ish): builds, stands, drives.
    sim::world::World t = make_mujoco_diff();
    sim::world::MotorParams mt;
    mt.support_count = 1;
    mt.supports[0].kind = "ball";
    mt.supports[0].x_m = -0.2;
    mt.supports[0].radius_m = 0.03;
    mt.supports[0].mass_kg = 0.1;
    mt.supports[0].friction_mu = 0.05;
    mt.supports[0].torsional = 0.005;
    mt.supports[0].rolling = 0.0005;
    t.set_motor_params(mt);
    set_diff_duty(t, 0.6);
    for (int i = 0; i < 50; ++i) t.step(0.02);
    assert(t.truth().body_z_m > 0.05);  // stands, never tips through
    assert(std::abs(t.truth().body_pitch_rad) < 0.1);
    assert(t.truth().body_x_m > 0.1);  // drives, not beached
    // Bad deck (belly at/below the floor): refused, truth frozen.
    sim::world::World bad = make_mujoco_diff();
    sim::world::MotorParams mb;
    mb.body_half_height_m = 0.5;
    bad.set_motor_params(mb);
    set_diff_duty(bad, 0.6);
    for (int i = 0; i < 10; ++i) bad.step(0.02);
    assert(bad.truth().forward_mps == 0.0);
    // Bad support kind: refused, truth frozen.
    sim::world::World bad2 = make_mujoco_diff();
    sim::world::MotorParams mb2;
    mb2.support_count = 1;
    mb2.supports[0].kind = "swivel";
    mb2.supports[0].x_m = -0.2;
    mb2.supports[0].radius_m = 0.03;
    mb2.supports[0].mass_kg = 0.1;
    bad2.set_motor_params(mb2);
    set_diff_duty(bad2, 0.6);
    for (int i = 0; i < 10; ++i) bad2.step(0.02);
    assert(bad2.truth().forward_mps == 0.0);
    // Config switch rebuilds (signature covers every build input): an
    // axle shift on a live world moves the drive pair in the wheel table
    // (previously the stale model survived silently).
    {
      sim::world::World w = make_mujoco_diff();
      set_diff_duty(w, 0.5);
      for (int i = 0; i < 5; ++i) w.step(0.02);
      sim::world::MotorParams ms = w.motor_params();
      ms.drive_axle_x_m = -0.1;
      w.set_motor_params(ms);
      for (int i = 0; i < 5; ++i) w.step(0.02);
      sim::world::WheelViz table[6];
      assert(w.wheels(table, 6) == 2);
      assert(table[0].x_m == -0.1 && table[1].x_m == -0.1);
    }
    // Declared payload (5 kg at 0.3 m): builds, stands, covers less
    // ground in fixed time (same stall force, more mass — the transient
    // lag integrates to a persistent distance gap, deterministic).
    sim::world::World laden = make_mujoco_diff();
    sim::world::MotorParams ml;
    ml.payload_mass_kg = 5.0;
    ml.payload_height_m = 0.3;
    laden.set_motor_params(ml);
    set_diff_duty(laden, 1.0);
    for (int i = 0; i < 100; ++i) laden.step(0.02);
    assert(laden.truth().body_z_m > 0.05);
    sim::world::World bare = make_mujoco_diff();
    set_diff_duty(bare, 1.0);
    for (int i = 0; i < 100; ++i) bare.step(0.02);
    assert(laden.truth().body_x_m < bare.truth().body_x_m);
    assert(bare.truth().body_x_m > 0.5);  // control actually drove
    // COM below the deck (belly breach): refused, truth frozen.
    sim::world::World bad3 = make_mujoco_diff();
    sim::world::MotorParams mb3;
    mb3.com_height_m = 0.04;
    bad3.set_motor_params(mb3);
    set_diff_duty(bad3, 0.6);
    for (int i = 0; i < 10; ++i) bad3.step(0.02);
    assert(bad3.truth().forward_mps == 0.0);
  }
#else
  // MuJoCo disabled: backend exists but reports invalid, never steps.
  {
    sim::world::World w;
    set_diff_duty(w, 1.0);
    w.step(0.02);
    assert(w.truth().forward_mps == 0.0);
  }
#endif

  std::printf("mujoco_backend ok\n");
  return 0;
}
