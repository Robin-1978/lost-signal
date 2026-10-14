// Wheel-velocity signal audit (inverse-dynamics readiness): measured wheel
// speeds (encoder model) vs true wheel speeds from body twist, plus the
// differentiator noise a feedforward would actually eat: finite differences
// of measured vs true, mapped through wheel_equiv_mass/stall_force into PWM
// authority fraction. Evidence for the FF gate, not a standard.

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"
#include "hw/chassis/diff_drive.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto build = catalog.presentation_for_level("lost_signal").recommended_build;
 game::MissionSimulation mission(catalog, "lost_signal", build);
 CHECK(mission.robot().drive_type == "differential");
 mission.run_to_completion();
 const double base = mission.robot().wheel_base_m;
 const double mass = 3.0;   // wheel_equiv_mass_kg, ro1 (audit constant, printed)
 const double fstall = 54.0; // stall_force_N, ro1 (audit constant, printed)

 int n = 0;
 double sum_vel = 0.0, max_vel = 0.0;
 double sum_acc = 0.0, sum_acc2 = 0.0, max_acc = 0.0;
 double prev_t = -1.0, prev_ml = 0.0, prev_mr = 0.0, prev_tl = 0.0, prev_tr = 0.0;
 bool have_prev = false;
 for (const auto& snapshot : mission.replay()) {
  double tl = 0.0, tr = 0.0;
  sim::DiffDrive::twistToWheels(snapshot.robot.v, snapshot.robot.w, base, tl, tr);
  const double ml = snapshot.control.left_measured_mps;
  const double mr = snapshot.control.right_measured_mps;
  CHECK(std::isfinite(ml) && std::isfinite(mr));
  sum_vel += std::abs(ml - tl) + std::abs(mr - tr);
  max_vel = std::max({max_vel, std::abs(ml - tl), std::abs(mr - tr)});
  n += 2;
  if (have_prev) {
   const double dt = snapshot.time_s - prev_t;
   if (dt > 1e-9) {
    const double al = (ml - prev_ml) / dt, atl = (tl - prev_tl) / dt;
    const double ar = (mr - prev_mr) / dt, atr = (tr - prev_tr) / dt;
    for (double e : {al - atl, ar - atr}) {
     sum_acc += std::abs(e);
     sum_acc2 += e * e;
     max_acc = std::max(max_acc, std::abs(e));
    }
   }
  }
  prev_t = snapshot.time_s;
  prev_ml = ml;
  prev_mr = mr;
  prev_tl = tl;
  prev_tr = tr;
  have_prev = true;
 }
 CHECK(n > 0);
 const double mean_acc = sum_acc / n;
 const double std_acc = std::sqrt(std::max(0.0, sum_acc2 / n - mean_acc * mean_acc));
 std::cerr << "wheel-vel err: mean=" << sum_vel / n << " max=" << max_vel << " (m/s, n=" << n << ")\n";
 std::cerr << "differentiator noise: mean=" << mean_acc << " std=" << std_acc << " max=" << max_acc
           << " (m/s^2)\n";
 std::cerr << "as PWM authority (mass=" << mass << "kg, Fstall=" << fstall
           << "N): mean=" << mass * mean_acc / fstall << " std=" << mass * std_acc / fstall
           << " max=" << mass * max_acc / fstall << " (fraction of full PWM)\n";
 std::cerr << "signal audit recorded\n";
 return 0;
}
