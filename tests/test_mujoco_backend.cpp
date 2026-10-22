// M1 parity gate: the MuJoCo backend must reproduce deployable behavior,
// not analytic numbers. Pins are behavioral (moves, stops, holds grade,
// deterministic); exact transients belong to the effort-model unit tests.
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

sim::world::World make_mujoco_diff() {
  return sim::world::World(sim::world::PhysicsBackend::kMujoco);
}

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
    assert(w.backend() == sim::world::PhysicsBackend::kMujoco);
    set_diff_duty(w, 1.0);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(std::isfinite(w.truth().forward_mps));
    assert(w.truth().forward_mps > 1.0);  // pulls hard
    assert(w.truth().forward_mps < 3.5);  // drag-limited, never above free
    assert(std::abs(w.truth().body_roll_rad) < 0.05);
    assert(std::abs(w.truth().body_pitch_rad) < 0.05);
    assert(w.truth().body_z_m > 0.0);  // off the ground plane, on wheels
    // Coast: the engine glides (rolling loss ~0 at these speeds, same as
    // main's duty path). Pin no-energy-creation and no-reversal, not a
    // decay rate — the analytic R(v) rate belongs to the analytic plant.
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
  // swerve straight. Mirrors the analytic plant pins, engine-appropriate
  // tolerances (no exact numbers — the engine owns the transients).
  {
    // Ackermann drives a left arc.
    sim::world::World w(sim::world::PhysicsBackend::kMujoco);
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
    sim::world::World m(sim::world::PhysicsBackend::kMujoco);
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
    sim::world::World s(sim::world::PhysicsBackend::kMujoco);
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
    sim::scenario::ScenarioSession session(
        sim::world::PhysicsBackend::kMujoco);
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
#else
  // MuJoCo disabled: backend exists but reports invalid, never steps.
  {
    sim::world::World w(sim::world::PhysicsBackend::kMujoco);
    set_diff_duty(w, 1.0);
    w.step(0.02);
    assert(w.truth().forward_mps == 0.0);
  }
#endif

  std::printf("mujoco_backend ok\n");
  return 0;
}
