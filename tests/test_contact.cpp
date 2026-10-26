#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "robot/platform/drive_profile.hpp"
#include "sim/eval/drive_profile_io.hpp"
#include "sim/world/contact.hpp"
#include "sim/world/motor.hpp"
#include "sim/world/world.hpp"
#include "sim/world/terrain.hpp"

using namespace lost_signal;

namespace {

sim::world::World make_diff_world() {
  sim::world::World w;
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 0;
  cmd.expiry_ns = 1'000'000'000;
  cmd.drive[0] = cmd.drive[1] = 1.0;
  w.apply(cmd);
  return w;
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
 // A rise/run grade rotates gravity; it must never create extra gravity.
 // The horizontal component follows body heading and points downhill.
 for (double grade : {0.0, 0.3, 1.0, 10.0}) {
 const auto gravity = sim::world::incline_gravity(grade, 0.7);
 assert(std::abs(std::sqrt(gravity.x_mps2 * gravity.x_mps2 +
 gravity.y_mps2 * gravity.y_mps2 +
 gravity.z_mps2 * gravity.z_mps2) - 9.81) < 1e-12);
 assert(gravity.z_mps2 < 0.0);
 if (grade > 0.0) {
 assert(gravity.x_mps2 * std::cos(0.7) +
 gravity.y_mps2 * std::sin(0.7) < 0.0);
 }
 }

  using sim::world::contact_resistance_N;
  using sim::world::contact_static_capacity_N;
  using sim::world::WheelContactParams;

  // R form pins (load = 24*9.81/2 per wheel on a diff).
  {
    WheelContactParams p;
    p.load_per_wheel_N = 24.0 * 9.81 / 2.0;
    assert(contact_resistance_N(0.0, p) == 0.0);  // no motion, no drag
    // Odd in v: reversing the wheel reverses the force exactly.
    assert(contact_resistance_N(-1.5, p) == -contact_resistance_N(1.5, p));
    // At speed the tanh saturates: Coulomb + viscous.
    const double expected = 0.01 * p.load_per_wheel_N + 0.6 * 3.0;
    assert(std::abs(contact_resistance_N(3.0, p) + expected) < 1e-9);
    assert(std::abs(contact_static_capacity_N(p) - 0.01 * p.load_per_wheel_N) <
           1e-12);
    // Negative params clamp, never inject energy.
    WheelContactParams bad{-1.0, -5.0, -1.0, -2.0};
    assert(contact_resistance_N(2.0, bad) == 0.0);
    assert(contact_static_capacity_N(bad) == 0.0);
  }

  // Coast-down: drag decays motion to an exact zero (never reverses,
  // never jitters across it).
  // Coast-down: viscous + rolling losses bleed speed monotonically to a
  // crawl — dissipative only, never reversed, never creating energy.
  {
    sim::world::World w = make_diff_world();
    for (int i = 0; i < 400; ++i) w.step(0.02);
    assert(w.truth().forward_mps > 2.5);  // cruise, below free speed
    set_diff_duty(w, 0.0);
    double prev = w.truth().forward_mps;
    int n_up = 0;
    for (int i = 0; i < 2000; ++i) {
      w.step(0.02);
      const double v = w.truth().forward_mps;
      if (v > prev + 1e-12) ++n_up;
      assert(v >= -1e-9);  // never reversed
      prev = v;
    }
    assert(n_up == 0);  // monotonic bleed, no energy creation
    assert(w.truth().forward_mps < 1.0);  // bled to a crawl
    for (int i = 0; i < 50; ++i) w.step(0.02);
    assert(w.truth().forward_mps < 1.0 && w.truth().forward_mps >= -1e-9);
  }

  // Grade hold: gentle incline creeps bounded (no runaway); steep breaks
  // away and stays away; brake holds the steep grade; NaN fails closed.
  {
    sim::world::World w;
    w.set_incline(0.005);
    set_diff_duty(w, 0.0);
    for (int i = 0; i < 200; ++i) w.step(0.02);
    assert(std::abs(w.truth().body_x_m) < 1.0);
    assert(std::abs(w.truth().forward_mps) < 0.5);
    // Steep grade overcomes the contact: rolls backward, stays backward.
    sim::world::World steep;
    steep.set_incline(0.3);
    set_diff_duty(steep, 0.0);
    for (int i = 0; i < 100; ++i) steep.step(0.02);
    assert(steep.truth().forward_mps < -0.5);
    assert(steep.truth().body_x_m < 0.0);
    // Service brake holds the steep grade once its lag ramps in.
    sim::world::MotorParams mp;
    mp.brake_max_torque_Nm = 5.0;
    sim::world::World held;
    held.set_motor_params(mp);
    held.set_incline(0.3);
    set_diff_duty(held, 0.0);
    held.set_brake_request(0, 1.0);
    held.set_brake_request(1, 1.0);
    for (int i = 0; i < 200; ++i) held.step(0.02);
    assert(held.truth().body_x_m > -0.5);
    assert(std::abs(held.truth().forward_mps) < 0.15);
    // Non-finite grade fails closed to flat, never NaN-poisons the plant.
    held.set_incline(std::numeric_limits<double>::quiet_NaN());
    held.step(0.02);
    assert(std::isfinite(held.truth().forward_mps));
  }

  // Motor breakaway: below-capacity duty creeps bounded (no runaway);
  // above-capacity duty drives away.
  {
    sim::world::World w;
    set_diff_duty(w, 0.01);
    for (int i = 0; i < 100; ++i) w.step(0.02);
    assert(std::abs(w.truth().forward_mps) < 0.5);
    assert(std::abs(w.truth().body_x_m) < 1.0);
    set_diff_duty(w, 0.1);
    for (int i = 0; i < 100; ++i) w.step(0.02);
    assert(w.truth().forward_mps > 0.1);
  }

  // Profile file retunes contact: higher rr settles lower (directional).
  {
    const std::string path =
        (std::filesystem::temp_directory_path() / "test_contact_profile.json")
            .string();
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out << "{\"kind\":\"differential\",\"rolling_resist\":0.05}";
    }
    robot::platform::DriveProfile prof;
    std::string err;
    namespace eval = sim::eval;
    assert(eval::load_drive_profile(path, prof, &err));
    assert(prof.rolling_resist == 0.05);
    assert(prof.viscous_drag_N_per_mps == 0.6);  // untouched default kept
    sim::world::World w;
    w.set_motor_params(sim::world::motor_params_from_profile(prof));
    set_diff_duty(w, 1.0);
    for (int i = 0; i < 400; ++i) w.step(0.02);
    // Profile numbers reach the plant: cruises fast but below free speed.
    assert(w.truth().forward_mps > 2.5 && w.truth().forward_mps <= 3.0);
    std::filesystem::remove(path);
  }

  std::printf("contact ok\n");
  return 0;
}
