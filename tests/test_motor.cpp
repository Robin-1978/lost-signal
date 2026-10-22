#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "robot/platform/drive_profile.hpp"
#include "sim/eval/drive_profile_io.hpp"
#include "sim/world/contact.hpp"
#include "sim/world/motor.hpp"
#include "sim/world/service_brake.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

lost_signal::sim::world::World make_diff_world() {
  using namespace lost_signal::sim;
  world::World w;
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 0;
  cmd.expiry_ns = 1'000'000'000;
  cmd.drive[0] = cmd.drive[1] = 1.0;
  w.apply(cmd);
  return w;
}

void set_diff_duty(lost_signal::sim::world::World& w, double duty) {
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 0;
  cmd.expiry_ns = 1'000'000'000;
  cmd.drive[0] = cmd.drive[1] = duty;
  w.apply(cmd);
}

}  // namespace

int main() {
  using namespace lost_signal::sim;

  // DC curve pins (defaults: stall 54 N, free 3 m/s).
  {
    using world::dc_motor_force_N;
    assert(dc_motor_force_N(0.0, 1.0, 54.0, 3.0) == 0.0);  // zero is coast
    assert(dc_motor_force_N(1.0, 0.0, 54.0, 3.0) == 54.0);  // stall at 0 speed
    assert(dc_motor_force_N(1.0, 3.0, 54.0, 3.0) == 0.0);   // zero at free
    assert(dc_motor_force_N(1.0, 5.0, 54.0, 3.0) == 0.0);   // clamped above
    assert(dc_motor_force_N(0.5, 0.0, 54.0, 3.0) == 27.0);
    // Opposing command keeps full electrical-braking authority.
    assert(dc_motor_force_N(-1.0, 1.0, 54.0, 3.0) == -54.0);
    assert(dc_motor_force_N(-1.0, 0.0, 54.0, 3.0) == -54.0);
    assert(dc_motor_force_N(2.0, 0.0, 54.0, 3.0) == 54.0);  // duty clamped
    assert(dc_motor_force_N(1.0, 1.5, 54.0, 3.0) == 27.0);  // linear falloff
  }

  // Brake plant pins: lag, capacity, clamping, fail-closed params.
  {
    using world::ServiceBrakeParams;
    using world::ServiceBrakePlant;
    ServiceBrakeParams p;
    p.maximum_torque_Nm = 10.0;
    ServiceBrakePlant brake(p);
    double cap = brake.step(1.0, 0.02);
    assert(std::abs(brake.telemetry().applied - 0.02 / 0.12) < 1e-9);
    assert(std::abs(cap - 10.0 * 0.02 / 0.12) < 1e-9);
    for (int i = 0; i < 5; ++i) brake.step(1.0, 0.02);
    assert(brake.telemetry().applied == 1.0);  // 6 x 20 ms >= 120 ms apply
    assert(brake.step(1.0, 0.02) == 10.0);
    brake.step(0.0, 0.02);  // release is faster (80 ms)
    assert(std::abs(brake.telemetry().applied - 0.75) < 1e-9);
    brake.step(2.0, 0.02);
    assert(brake.telemetry().requested == 1.0);  // request clamped
    bool threw = false;
    try {
      ServiceBrakeParams bad;
      bad.maximum_torque_Nm = -1.0;
      ServiceBrakePlant b(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    threw = false;
    try {
      brake.step(1.0, -0.02);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }

  // World step response pins stall/mass exactly (a = 54/3 = 18 m/s^2).
  {
    world::World w = make_diff_world();
    w.step(0.02);
    assert(std::abs(w.truth().forward_mps - 0.36) < 1e-9);
    assert(std::abs(w.truth().body_x_m - 0.36 * 0.02) < 1e-9);
    for (int i = 0; i < 400; ++i) w.step(0.02);
    // Drag equilibrium: 54(1-v/3) = 1.18+0.6v -> v ~= 2.84. Below the
    // drag-free no-load speed, never above it.
    assert(w.truth().forward_mps > 2.80 && w.truth().forward_mps < 2.88);
    // Oppose from speed: electrical authority plus drag, summed exactly.
    const double v0 = w.truth().forward_mps;
    world::WheelContactParams cp;
    cp.load_per_wheel_N = 24.0 * 9.81 / 2.0;
    const double drag0 = world::contact_resistance_N(v0, cp);
    set_diff_duty(w, -1.0);
    w.step(0.02);
    const double expect_dv = (-54.0 + drag0) / 3.0 * 0.02;
    assert(std::abs((w.truth().forward_mps - v0) - expect_dv) < 1e-9);
    // Coast: zero duty decays through drag (S2 contact owns this now).
    set_diff_duty(w, 0.0);
    const double vc = w.truth().forward_mps;
    w.step(0.02);
    assert(w.truth().forward_mps < vc && w.truth().forward_mps >= 0.0);
  }

  // Brake hook: dissipative only — stops, holds, never reverses.
  {
    world::MotorParams mp;
    mp.brake_max_torque_Nm = 5.0;  // 50 N/wheel at r = 0.1
    world::World w;
    w.set_motor_params(mp);
    set_diff_duty(w, 1.0);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(w.truth().forward_mps > 2.80);  // drag equilibrium, not free speed
    set_diff_duty(w, 0.0);
    w.set_brake_request(0, 1.0);
    w.set_brake_request(1, 1.0);
    w.set_brake_request(7, 1.0);  // out of range: ignored, no crash
    double min_v = w.truth().forward_mps;
    int stopped_at = -1;
    for (int i = 0; i < 50; ++i) {
      w.step(0.02);
      min_v = std::min(min_v, w.truth().forward_mps);
      if (w.truth().forward_mps == 0.0 && stopped_at < 0) stopped_at = i;
    }
    assert(stopped_at >= 0 && stopped_at < 50);
    assert(min_v >= 0.0);  // never reversed
    // Oversized brake still clamps to an exact stop within a few lag steps
    // and holds (never reverses).
    world::MotorParams big;
    big.brake_max_torque_Nm = 100.0;
    world::World w2;
    w2.set_motor_params(big);
    set_diff_duty(w2, 1.0);
    for (int i = 0; i < 200; ++i) w2.step(0.02);
    set_diff_duty(w2, 0.0);
    w2.set_brake_request(0, 1.0);
    w2.set_brake_request(1, 1.0);
    bool stopped = false;
    for (int i = 0; i < 10; ++i) {
      w2.step(0.02);
      assert(w2.truth().forward_mps >= 0.0);
      if (w2.truth().forward_mps == 0.0) stopped = true;
    }
    assert(stopped);
    // Disabled by default: requests with zero capacity change nothing
    // (a twin world without requests integrates identically).
    world::World with_brake = make_diff_world();
    world::World without_brake = make_diff_world();
    for (int i = 0; i < 100; ++i) {
      with_brake.step(0.02);
      without_brake.step(0.02);
    }
    set_diff_duty(with_brake, 0.0);
    set_diff_duty(without_brake, 0.0);
    with_brake.set_brake_request(0, 1.0);
    with_brake.set_brake_request(1, 1.0);
    for (int i = 0; i < 20; ++i) {
      with_brake.step(0.02);
      without_brake.step(0.02);
      assert(with_brake.truth().forward_mps ==
             without_brake.truth().forward_mps);
    }
  }

  // DriveProfile -> plant chain: one set of numbers, sim and native.
  {
    robot::platform::DriveProfile prof;
    assert(robot::platform::DriveProfile::for_chassis("differential", prof));
    assert(prof.stall_force_N == 54.0 && prof.wheel_equiv_mass_kg == 3.0);
    world::MotorParams mp = world::motor_params_from_profile(prof);
    assert(mp.stall_force_N == 54.0 && mp.wheel_equiv_mass_kg == 3.0 &&
           mp.free_speed_mps == 3.0);
    // File override retunes the plant: doubled stall, doubled first-step.
    const std::string path =
        (std::filesystem::temp_directory_path() / "test_motor_profile.json")
            .string();
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out << "{\"kind\":\"differential\",\"stall_force_N\":108.0}";
    }
    robot::platform::DriveProfile tuned;
    std::string err;
    assert(eval::load_drive_profile(path, tuned, &err));
    assert(tuned.stall_force_N == 108.0);
    assert(tuned.wheel_equiv_mass_kg == 3.0);  // default kept
    world::World w;
    w.set_motor_params(world::motor_params_from_profile(tuned));
    set_diff_duty(w, 1.0);
    w.step(0.02);
    assert(std::abs(w.truth().forward_mps - 0.72) < 1e-9);
    std::filesystem::remove(path);
  }

  std::printf("motor ok\n");
  return 0;
}
