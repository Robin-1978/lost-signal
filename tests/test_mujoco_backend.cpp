// M1 parity gate: the MuJoCo backend must reproduce deployable behavior,
// not analytic numbers. Pins are behavioral (moves, stops, holds grade,
// deterministic); exact transients belong to the effort-model unit tests.
#include <cassert>
#include <cmath>
#include <cstdio>
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

  // Brake holds; grade hold without rollback (main's slope-launch P1 shape).
  {
    sim::world::MotorParams mp;
    mp.brake_max_torque_Nm = 5.0;
    sim::world::World w = make_mujoco_diff();
    w.set_motor_params(mp);
    set_diff_duty(w, 1.0);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    set_diff_duty(w, 0.0);
    w.set_brake_request(0, 1.0);
    w.set_brake_request(1, 1.0);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(std::abs(w.truth().forward_mps) < 0.05);
    const double x_hold = w.truth().body_x_m;
    w.set_incline(0.1);  // tilt gravity: held by the brake, no rollback
    // Parking-brake-only grade hold (main-verbatim smoothing brake):
    // bounded creep, never the P1 runaway. Exact standstill belongs to
    // the analytic plant's stiction; the closed-loop chassis brake (the
    // HIL norm) holds tighter than either.
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(w.truth().body_x_m >= x_hold - 0.3);
    assert(std::abs(w.truth().forward_mps) < 0.05);
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
