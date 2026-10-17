// Stage-4 offline closed-loop comparison (production untouched):
// a parity-gated mirror of DifferentialMotorDrive::step + mission
// wheel_ground, driven by recorded replay traces, then the T_lead x filter x
// plant-mismatch matrix with synthetic step scenarios.
// Production USED (never re-derived): PID, twistToWheels/wheelsToTwist,
// tune_wheel_pid, feedforward mapping, active_in_level, kPi.
// Mirrored (bit-parity vs recorded traces gates every line): target clamp,
// encoder, wheel force, body integration, ground combine, surface box test.
// A parity failure stops this file (investigate, never loosen silently).

#include "mission/campaign_progress.hpp"
#include "mission/content_catalog.hpp"
#include "scenario/scenario_session.hpp"
#include "core/math.hpp"
#include "autonomy/control/pid.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "hw/feedforward.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

constexpr double kGravity = 9.81;
constexpr double kCoulombWidthMps = 0.05;
constexpr double kViscousNPerMps = 0.6;
constexpr double kBaseLeadTimeS = 1.0 / 6.0;

bool box_inside(sim::Vec2 point, sim::Vec2 lo, sim::Vec2 hi) {
 return point.x >= lo.x && point.x < hi.x && point.y >= lo.y && point.y < hi.y;
}

struct MirrorGround {
 double mu_eff{0.7};
 double rolling_resist{0.01};
 double accel_limit{6.0};
};

struct MirrorLoop {
 sim::MotorDriveParams params;
 sim::PID left_pid;
 sim::PID right_pid;
 double v{0.0};
 double w{0.0};
 double filt_l{0.0};
 double filt_r{0.0};

 void set_state(double v_in, double w_in) {
  v = v_in;
  w = w_in;
 }
 // Per-step outputs (mirrors ChassisTelemetry fields used by the metrics).
 double meas_l{0.0};
 double meas_r{0.0};
 double pwm_l{0.0};
 double pwm_r{0.0};
 double pid_i_l{0.0};
 double pid_i_r{0.0};
 double ff_l{0.0};
 double ff_r{0.0};

 explicit MirrorLoop(sim::MotorDriveParams p, sim::WheelPidGains gains)
     : params(p), left_pid(gains.kp, gains.ki, gains.kd, params.pwm_limit, params.integral_limit,
                           sim::AntiWindupMode::ConditionalIntegration),
       right_pid(gains.kp, gains.ki, gains.kd, params.pwm_limit, params.integral_limit,
                 sim::AntiWindupMode::ConditionalIntegration) {}

 double encoder(double wheel_mps, double dt) const {
  const double circ = 2.0 * sim::kPi * params.wheel_radius_m;
  const double counts =
    std::round(wheel_mps * dt / circ * static_cast<double>(params.encoder_counts_per_revolution));
  return counts * circ / (static_cast<double>(params.encoder_counts_per_revolution) * dt);
 }

 // One mirrored drive step. ff_* = 0 reproduces production bit-exactly
 // (parity gate below); nonzero evaluates the frozen candidate rule
 // pwm = clamp(pid + ff), integrator unconditioned, exactly as specified.
 void step(double target_l, double target_r, const MirrorGround& ground, double dt, double ff_l_in,
           double ff_r_in) {
  target_l = std::clamp(target_l, -params.max_wheel_speed_mps, params.max_wheel_speed_mps);
  target_r = std::clamp(target_r, -params.max_wheel_speed_mps, params.max_wheel_speed_mps);
  double actual_l = 0.0;
  double actual_r = 0.0;
  sim::DiffDrive::twistToWheels(v, w, params.wheel_base_m, actual_l, actual_r);
  meas_l = encoder(actual_l, dt);
  meas_r = encoder(actual_r, dt);
  pwm_l = left_pid.update(target_l - meas_l, dt);
  pwm_r = right_pid.update(target_r - meas_r, dt);
  pid_i_l = left_pid.i();
  pid_i_r = right_pid.i();
  const double total_l = std::clamp(pwm_l + ff_l_in, -params.pwm_limit, params.pwm_limit);
  const double total_r = std::clamp(pwm_r + ff_r_in, -params.pwm_limit, params.pwm_limit);
  pwm_l = total_l;
  pwm_r = total_r;
  const double weight = params.mass_kg * kGravity / 2.0;
  const double traction = std::max(0.0, ground.mu_eff) * weight;
  const double rr = std::max(0.0, ground.rolling_resist);
  double force_l = 0.0;
  double force_r = 0.0;
  for (int side = 0; side < 2; ++side) {
   const double pwm = side == 0 ? total_l : total_r;
   const double wheel = side == 0 ? actual_l : actual_r;
   const double curve = std::max(0.0, 1.0 - std::abs(wheel) / std::max(0.05, params.free_speed_mps));
   double force = pwm * params.stall_force_N * curve;
   force -= rr * weight * std::tanh(wheel / kCoulombWidthMps) + kViscousNPerMps * wheel;
   if (std::abs(force) > traction) {
    force = std::copysign(traction, force);
   }
   if (side == 0) {
    force_l = force;
   } else {
    force_r = force;
   }
  }
  const double accel = (force_l + force_r) / std::max(0.5, params.mass_kg);
  const double alpha =
    (force_r - force_l) * params.wheel_base_m / 2.0 / std::max(0.05, params.yaw_inertia_kgm2);
  const double cap = std::min(params.max_wheel_accel_mps2, std::max(0.5, ground.accel_limit));
  v = v + std::clamp(accel * dt, -cap * dt, cap * dt);
  w = w + alpha * dt;
  double actual_cl = 0.0;
  double actual_cr = 0.0;
  sim::DiffDrive::twistToWheels(v, w, params.wheel_base_m, actual_cl, actual_cr);
  actual_cl = std::clamp(actual_cl, -params.max_wheel_speed_mps, params.max_wheel_speed_mps);
  actual_cr = std::clamp(actual_cr, -params.max_wheel_speed_mps, params.max_wheel_speed_mps);
  sim::DiffDrive::wheelsToTwist(actual_cl, actual_cr, params.wheel_base_m, v, w);
 }
};

struct Metrics {
 int n{0};
 int bad{0};
 double sum_sq_err{0.0};
 double max_err{0.0};
 double pwm_peak{0.0};
 int sat_steps{0};
 double sum_sq_pidi{0.0};
 double sum_sq_ff{0.0};
 double prev_ff_l{0.0};
 double prev_ff_r{0.0};
 double sum_sq_dff{0.0};
 double prev_pwm_l{0.0};
 double prev_pwm_r{0.0};
 double sum_sq_dpwm{0.0};
 void observe(double err_l, double err_r, double pwm_l, double pwm_r, double idi_l, double idi_r, double ff_l,
              double ff_r) {
  ++n;
  sum_sq_err += err_l * err_l + err_r * err_r;
  max_err = std::max({max_err, std::abs(err_l), std::abs(err_r)});
  pwm_peak = std::max({pwm_peak, std::abs(pwm_l), std::abs(pwm_r)});
  if (std::abs(pwm_l) > 0.999 || std::abs(pwm_r) > 0.999) {
   ++sat_steps;
  }
  sum_sq_pidi += idi_l * idi_l + idi_r * idi_r;
  sum_sq_ff += ff_l * ff_l + ff_r * ff_r;
  sum_sq_dff += (ff_l - prev_ff_l) * (ff_l - prev_ff_l) + (ff_r - prev_ff_r) * (ff_r - prev_ff_r);
  sum_sq_dpwm += (pwm_l - prev_pwm_l) * (pwm_l - prev_pwm_l) + (pwm_r - prev_pwm_r) * (pwm_r - prev_pwm_r);
  prev_ff_l = ff_l;
  prev_ff_r = ff_r;
  prev_pwm_l = pwm_l;
  prev_pwm_r = pwm_r;
 }
};

struct TraceFrame {
 double t{0.0};
 double target_l{0.0};
 double target_r{0.0};
 double meas_l{0.0};
 double meas_r{0.0};
 double x{0.0};
 double y{0.0};
};

double mprop(const mission::ContentCatalog& catalog, const std::string& component, const std::string& key) {
 return catalog.components().at(component).behavior.at(key);
}

MirrorGround ground_at(const mission::ContentCatalog& catalog, const std::string& level_id,
                       const std::string& tire_id, sim::Vec2 pos, double motor_max_accel) {
 const double tire_mu = mprop(catalog, tire_id, "friction_mu");
 const double tire_rr = mprop(catalog, tire_id, "rolling_resistance");
 const double brake = mprop(catalog, tire_id, "brake_decel_mps2");
 double ground_mu = 1.0;
 double ground_rr = 0.0;
 double traction = 1.0;
 for (const auto& surface : catalog.map("research_base").surfaces) {
  if (mission::active_in_level(surface.active_in, level_id) && box_inside(pos, surface.min, surface.max)) {
   ground_mu = surface.friction_mu;
   ground_rr = surface.rolling_drag;
   traction = surface.traction_scale;
   break;
  }
 }
 MirrorGround ground;
 ground.mu_eff = tire_mu * ground_mu / 0.8;
 ground.rolling_resist = tire_rr + ground_rr;
 ground.accel_limit = std::min(motor_max_accel, brake * (0.5 + 0.5 * traction));
 return ground;
}

struct RunMetrics {
 double rmse{0.0};
 double max_err{0.0};
 double pwm_peak{0.0};
 double sat_frac{0.0};
 double pid_irms{0.0};
 double ff_rms{0.0};
 double chatter_ff{0.0};
 double chatter_pwm{0.0};
 double overshoot{0.0};
 double settle_s{0.0};
 int bad{0};
};

// Closed-loop replay of recorded wheel targets through the mirror loop.
// ff_scale multiplies T_lead (sensitivity), tau_s filters the FF-path
// measurement only (PID always sees raw encoders, as deployed).
RunMetrics run_trace(const std::vector<TraceFrame>& frames, sim::MotorDriveParams params,
                     sim::WheelPidGains gains, const std::vector<MirrorGround>& grounds,
                     const sim::FeedforwardParams& ff_base, double wheel_base_m, double t_scale, double tau_s,
                     bool want_ff, double* parity_maxdiff = nullptr) {
 MirrorLoop loop(params, gains);
 Metrics m;
 const double t_lead = kBaseLeadTimeS * t_scale;
 for (std::size_t i = 0; i < frames.size(); ++i) {
  const auto& frame = frames[i];
  // Recorded replay cadence (~0.05 s), NOT the physics 0.02 s step: this run
  // is a resampled approximation (targets/ground held per interval), while
  // bit-exactness lives in direct_parity() below. Fidelity vs recorded is
  // measured, not assumed (RMSE gate in main). A first frame stamped at t = 0
  // takes no step (production never steps dt = 0; rest is already right).
  const double dt = frame.t - (i == 0 ? 0.0 : frames[i - 1].t);
  if (dt < 1e-9) {
   continue;
  }
  // Parity inputs mirror production order: targets, ground, then step.
  // FF uses filtered measurement; PID uses raw (deployed separation).
  double ff_l = 0.0;
  double ff_r = 0.0;
  if (want_ff) {
   const double alpha = tau_s <= 0.0 ? 1.0 : 1.0 - std::exp(-dt / tau_s);
   loop.filt_l += (loop.meas_l - loop.filt_l) * alpha;
   loop.filt_r += (loop.meas_r - loop.filt_r) * alpha;
   double tv = 0.0;
   double tw = 0.0;
   sim::DiffDrive::wheelsToTwist(frame.target_l, frame.target_r, wheel_base_m, tv, tw);
   double mv = 0.0;
   double mw = 0.0;
   sim::DiffDrive::wheelsToTwist(loop.filt_l, loop.filt_r, wheel_base_m, mv, mw);
   sim::FeedforwardParams ffp = ff_base;
   ffp.rolling_resist = grounds[i].rolling_resist;
   const auto des = sim::lead_accel(tv, mv, tw, mw, t_lead);
   const auto ff = sim::twist_feedforward(des, mv, mw, wheel_base_m, ffp);
   ff_l = ff.left.pwm_ff;
   ff_r = ff.right.pwm_ff;
  }
  loop.step(frame.target_l, frame.target_r, grounds[i], dt, ff_l, ff_r);
  if (parity_maxdiff && !want_ff) {
   const double diff = std::max(std::abs(loop.meas_l - frame.meas_l), std::abs(loop.meas_r - frame.meas_r));
   *parity_maxdiff = std::max(*parity_maxdiff, diff);
  }
  m.observe(loop.meas_l - frame.target_l, loop.meas_r - frame.target_r, loop.pwm_l, loop.pwm_r, loop.pid_i_l,
            loop.pid_i_r, ff_l, ff_r);
  if (!std::isfinite(loop.meas_l) || !std::isfinite(loop.meas_r) || std::abs(loop.pwm_l) > 1.0 + 1e-12 ||
      std::abs(loop.pwm_r) > 1.0 + 1e-12) {
   ++m.bad;
  }
 }
 RunMetrics out;
 out.rmse = std::sqrt(m.sum_sq_err / std::max(1, 2 * m.n));
 out.max_err = m.max_err;
 out.pwm_peak = m.pwm_peak;
 out.sat_frac = static_cast<double>(m.sat_steps) / std::max(1, m.n);
 out.pid_irms = std::sqrt(m.sum_sq_pidi / std::max(1, 2 * m.n));
 out.ff_rms = std::sqrt(m.sum_sq_ff / std::max(1, 2 * m.n));
 out.chatter_ff = std::sqrt(m.sum_sq_dff / std::max(1, 2 * m.n));
 out.chatter_pwm = std::sqrt(m.sum_sq_dpwm / std::max(1, 2 * m.n));
 out.bad = m.bad;
 return out;
}

struct StepMeta {
 bool active{false};
 double step_time{0.0};
 double final_l{0.0};
 double final_r{0.0};
 double band{0.04};
};

// Same as run_trace but on synthetic wheel-target steps (nominal ground),
// with overshoot/settling. Plant params may mismatch the FF model (load
// change); gains and FF stay nominal — exactly the real-world case.
RunMetrics run_steps(const std::vector<std::pair<double, double>>& targets, double dt,
                     sim::MotorDriveParams params, sim::WheelPidGains gains, const MirrorGround& ground,
                     const sim::FeedforwardParams& ff_base, double wheel_base_m, double t_scale, double tau_s,
                     bool want_ff, const StepMeta& meta = {}) {
 MirrorLoop loop(params, gains);
 Metrics m;
 double overshoot = 0.0;
 double settle = 0.0;
 double t = 0.0;
 for (const auto& [tL, tR] : targets) {
  double ff_l = 0.0;
  double ff_r = 0.0;
  if (want_ff) {
   const double alpha = tau_s <= 0.0 ? 1.0 : 1.0 - std::exp(-dt / tau_s);
   loop.filt_l += (loop.meas_l - loop.filt_l) * alpha;
   loop.filt_r += (loop.meas_r - loop.filt_r) * alpha;
   double tv = 0.0;
   double tw = 0.0;
   sim::DiffDrive::wheelsToTwist(tL, tR, wheel_base_m, tv, tw);
   double mv = 0.0;
   double mw = 0.0;
   sim::DiffDrive::wheelsToTwist(loop.filt_l, loop.filt_r, wheel_base_m, mv, mw);
   sim::FeedforwardParams ffp = ff_base;
   ffp.rolling_resist = ground.rolling_resist;
   const auto des = sim::lead_accel(tv, mv, tw, mw, kBaseLeadTimeS * t_scale);
   const auto ff = sim::twist_feedforward(des, mv, mw, wheel_base_m, ffp);
   ff_l = ff.left.pwm_ff;
   ff_r = ff.right.pwm_ff;
  }
  loop.step(tL, tR, ground, dt, ff_l, ff_r);
  m.observe(loop.meas_l - tL, loop.meas_r - tR, loop.pwm_l, loop.pwm_r, loop.pid_i_l, loop.pid_i_r, ff_l,
            ff_r);
  if (!std::isfinite(loop.meas_l) || !std::isfinite(loop.meas_r) || std::abs(loop.pwm_l) > 1.0 + 1e-12 ||
      std::abs(loop.pwm_r) > 1.0 + 1e-12) {
   ++m.bad;
  }
  if (meta.active && t >= meta.step_time) {
   overshoot = std::max({overshoot, loop.meas_l - meta.final_l, loop.meas_r - meta.final_r,
                         meta.final_l - loop.meas_l, meta.final_r - loop.meas_r});
   if (std::abs(loop.meas_l - meta.final_l) > meta.band || std::abs(loop.meas_r - meta.final_r) > meta.band) {
    settle = t;
   }
  }
  t += dt;
 }
 RunMetrics out;
 out.rmse = std::sqrt(m.sum_sq_err / std::max(1, 2 * m.n));
 out.max_err = m.max_err;
 out.pwm_peak = m.pwm_peak;
 out.sat_frac = static_cast<double>(m.sat_steps) / std::max(1, m.n);
 out.pid_irms = std::sqrt(m.sum_sq_pidi / std::max(1, 2 * m.n));
 out.ff_rms = std::sqrt(m.sum_sq_ff / std::max(1, 2 * m.n));
 out.chatter_ff = std::sqrt(m.sum_sq_dff / std::max(1, 2 * m.n));
 out.chatter_pwm = std::sqrt(m.sum_sq_dpwm / std::max(1, 2 * m.n));
 out.overshoot = std::max(0.0, overshoot);
 out.settle_s = settle;
 out.bad = m.bad;
 return out;
}

void print_metrics(const char* tag, const RunMetrics& m) {
 std::cerr << tag << " rmse=" << m.rmse << " maxerr=" << m.max_err << " pwmpeak=" << m.pwm_peak
           << " satfrac=" << m.sat_frac << " pidrms=" << m.pid_irms << " ffrms=" << m.ff_rms
           << " chff=" << m.chatter_ff << " chpwm=" << m.chatter_pwm;
 if (m.settle_s > 0.0 || m.overshoot > 0.0) {
  std::cerr << " over=" << m.overshoot << " settle=" << m.settle_s;
 }
 std::cerr << "\n";
}

// Direct parity: production DifferentialMotorDrive vs the mirror loop on
// IDENTICAL step inputs (commands, states, grounds, dt incl. off-0.02
// cadences, slip and dry regimes). Bit-exactness here is what certifies the
// mirror; the replay runs below are resampled approximations gated by
// fidelity, never by parity.
double direct_parity(const sim::MotorDriveParams& params, const sim::WheelPidGains& gains) {
 sim::DifferentialMotorDrive truth(params, gains);
 MirrorLoop mirror(params, gains);
 double worst = 0.0;
 sim::DiffState chained{0.0, 0.0, 0.0, 0.0, 0.0};
 auto ground_for = [](int i) {
  MirrorGround ground;
  ground.mu_eff = (i % 3 == 0) ? 0.2 : 0.875;
  ground.rolling_resist = (i % 3 == 0) ? 0.05 : 0.01;
  ground.accel_limit = 6.0;
  return ground;
 };
 for (int i = 0; i < 400; ++i) {
  const double dt = (i % 4 == 0) ? 0.05 : 0.02;
  const double tv = 2.0 * std::sin(0.11 * i) + ((i / 50) % 2 == 0 ? 0.0 : -1.0);
  const double tw = 1.2 * std::sin(0.07 * i + 1.0);
  double wl = 0.0;
  double wr = 0.0;
  sim::DiffDrive::twistToWheels(tv, tw, params.wheel_base_m, wl, wr);
  sim::DiffState state = chained;
  sim::GroundContact contact;
  const MirrorGround mirror_ground = ground_for(i);
  contact.mu_eff = mirror_ground.mu_eff;
  contact.rolling_resist = mirror_ground.rolling_resist;
  contact.accel_limit_mps2 = mirror_ground.accel_limit;
  // Chained trajectories (not fresh random states): both sides advance from
  // the same state, so any transcription error compounds instead of hiding.
  mirror.set_state(state.v, state.w);
  const sim::DiffState out = truth.step(state, {tv, tw}, dt, contact);
  state = out;
  const auto& telemetry = truth.telemetry();
  mirror.step(wl, wr, mirror_ground, dt, 0.0, 0.0);
  worst =
    std::max({worst, std::abs(out.v - mirror.v), std::abs(out.w - mirror.w),
              std::abs(telemetry.left.measured_mps - mirror.meas_l),
              std::abs(telemetry.right.measured_mps - mirror.meas_r),
              std::abs(telemetry.left.pwm - mirror.pwm_l), std::abs(telemetry.right.pwm - mirror.pwm_r)});
 }
 std::cerr << "direct parity maxdiff=" << worst << "\n";
 return worst;
}

} // namespace

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& pres = catalog.presentation_for_level("lost_signal");
 const auto& build = pres.recommended_build;
 const auto& robot = catalog.robot(mission::ScenarioSession::level_robot_id(catalog, "lost_signal", build));
 const std::string motor_id = build.parts.at("motor");
 const std::string tire_id = build.parts.at("tires");
 auto cprop = [&](const std::string& id, const std::string& key) {
  return catalog.components().at(id).behavior.at(key);
 };
 // Factory-identical drive construction (mission_simulation.cpp:136-157).
 sim::MotorDriveParams params;
 params.wheel_base_m = robot.wheel_base_m;
 params.wheel_radius_m = robot.wheel_radius_m;
 params.max_wheel_speed_mps = cprop(motor_id, "max_wheel_speed_mps");
 params.max_wheel_accel_mps2 = std::min(robot.motor.max_wheel_accel_mps2, cprop(tire_id, "brake_decel_mps2"));
 params.stall_force_N = cprop(motor_id, "stall_force_N");
 params.free_speed_mps = cprop(motor_id, "free_speed_mps");
 params.mass_kg = robot.mass_kg;
 params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 params.pwm_limit = robot.motor.pwm_limit;
 params.integral_limit = robot.motor.integral_limit;
 const double wheel_mass = std::max(0.5, robot.mass_kg / 2.0);
 const sim::WheelPidGains gains = sim::tune_wheel_pid(wheel_mass, params.stall_force_N, params.free_speed_mps,
                                                      0.5 * params.max_wheel_speed_mps);
 sim::FeedforwardParams ff_base;
 ff_base.wheel_mass_kg = robot.motor.wheel_equiv_mass_kg;
 ff_base.stall_force_N = params.stall_force_N;
 ff_base.free_speed_mps = params.free_speed_mps;
 ff_base.load_per_unit_N = robot.mass_kg * 9.81 / 2.0;

 // Recorded traces: full pinned finale run.
 mission::ScenarioSession mission(catalog, "lost_signal", build);
 mission.run_to_completion();
 if (mission.result().status != mission::MissionStatus::Succeeded) {
  std::cerr << "recording mission failed: reason=" << mission.result().failure_reason
            << " time=" << mission.result().time_s << " collisions=" << mission.result().collisions
            << " clearance=" << mission.result().min_dynamic_clearance_m << '\n';
 }
 CHECK(mission.result().status == mission::MissionStatus::Succeeded);
 std::vector<TraceFrame> frames;
 std::vector<MirrorGround> grounds;
 for (const auto& snapshot : mission.replay()) {
  TraceFrame frame;
  frame.t = snapshot.time_s;
  frame.target_l = snapshot.control.left_target_mps;
  frame.target_r = snapshot.control.right_target_mps;
  frame.meas_l = snapshot.control.left_measured_mps;
  frame.meas_r = snapshot.control.right_measured_mps;
  frame.x = snapshot.robot.x;
  frame.y = snapshot.robot.y;
  frames.push_back(frame);
  grounds.push_back(
    ground_at(catalog, "lost_signal", tire_id, {frame.x, frame.y}, robot.motor.max_wheel_accel_mps2));
 }
 CHECK(frames.size() > 100);

 // Bit-exactness lives here (identical step inputs), not in the resampled
 // replay below. Mismatch stops the file: mirror bug, never tolerance creep.
 CHECK(direct_parity(params, gains) == 0.0);

 // Parity gate (resampled replay): FF-disabled mirror vs recorded traces.
 // Fidelity bound, not bit-exactness (replay cadence ~= 0.05 s vs physics
 // 0.02 s; targets/ground held per interval). Breach means investigate
 // (uncovered regime or mirror bug), never loosen silently.
 {
  const RunMetrics base =
    run_trace(frames, params, gains, grounds, ff_base, robot.wheel_base_m, 1.0, 0.0, false);
  double replay_rmse = 0.0;
  {
   MirrorLoop loop(params, gains);
   double sum = 0.0;
   int n = 0;
   for (std::size_t i = 0; i < frames.size(); ++i) {
    const double dt = frames[i].t - (i == 0 ? 0.0 : frames[i - 1].t);
    if (dt < 1e-9) {
     continue;
    }
    loop.step(frames[i].target_l, frames[i].target_r, grounds[i], dt, 0.0, 0.0);
    sum += (loop.meas_l - frames[i].meas_l) * (loop.meas_l - frames[i].meas_l) +
           (loop.meas_r - frames[i].meas_r) * (loop.meas_r - frames[i].meas_r);
    n += 2;
   }
   replay_rmse = std::sqrt(sum / n);
  }
  std::cerr << "replay fidelity rmse=" << replay_rmse << " baseline rmse=" << base.rmse << "\n";
  CHECK(replay_rmse < 0.3);
  print_metrics("replay pid-only", base);
  // Determinism rerun.
  const RunMetrics rerun =
    run_trace(frames, params, gains, grounds, ff_base, robot.wheel_base_m, 1.0, 0.0, false);
  CHECK(rerun.rmse == base.rmse);
 }

 // T_lead x filter matrix on the recorded traces (FF enabled).
 const double t_scales[5] = {0.5, 0.75, 1.0, 1.25, 1.5};
 const double taus[3] = {0.0, 0.02, 0.05};
 for (const double t_scale : t_scales) {
  RunMetrics raw_metrics{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (const double tau : taus) {
   const RunMetrics m =
     run_trace(frames, params, gains, grounds, ff_base, robot.wheel_base_m, t_scale, tau, true);
   char tag[64];
   std::snprintf(tag, sizeof(tag), "replay ff T=%.2f tau=%.2f", t_scale, tau);
   print_metrics(tag, m);
   if (tau == 0.0) {
    raw_metrics = m;
   } else if (tau == 0.05) {
    // Filter monotonicity: lagged FF chatters no more than raw FF.
    CHECK(m.chatter_ff <= raw_metrics.chatter_ff + 1e-12);
   }
  }
 }

 // Synthetic steps on nominal ground (paved+grip combine, deterministic).
 MirrorGround nominal;
 nominal.mu_eff = 0.7 * 1.0 / 0.8;
 nominal.rolling_resist = 0.01;
 nominal.accel_limit = 6.0;
 constexpr double kDt = 0.02;
 auto hold = [&](double seconds, double left, double right) {
  std::vector<std::pair<double, double>> out;
  for (double t = 0.0; t < seconds; t += kDt) {
   out.emplace_back(left, right);
  }
  return out;
 };
 struct StepCase {
  const char* name;
  std::vector<std::pair<double, double>> targets;
  StepMeta meta;
 };
 std::vector<StepCase> cases;
 cases.push_back({"start", hold(3.0, 2.0, 2.0), {true, 0.1, 2.0, 2.0, 0.04}});
 cases.push_back({"estop", hold(1.0, 2.0, 2.0), {true, 1.0, 0.0, 0.0, 0.04}});
 {
  auto& estop_targets = cases.back().targets;
  auto tail = hold(2.0, 0.0, 0.0);
  estop_targets.insert(estop_targets.end(), tail.begin(), tail.end());
 }
 {
  std::vector<std::pair<double, double>> curve;
  for (double t = 0.0; t < 0.5; t += kDt) {
   curve.emplace_back(1.0, 1.0);
  }
  double wl = 0.0;
  double wr = 0.0;
  sim::DiffDrive::twistToWheels(1.0, 1.5, robot.wheel_base_m, wl, wr);
  for (double t = 0.5; t < 3.0; t += kDt) {
   curve.emplace_back(wl, wr);
  }
  cases.push_back({"curve", curve, {true, 0.5, wl, wr, 0.04}});
 }
 {
  std::vector<std::pair<double, double>> rev;
  for (double t = 0.0; t < 0.5; t += kDt) {
   rev.emplace_back(1.0, 1.0);
  }
  for (double t = 0.5; t < 4.0; t += kDt) {
   rev.emplace_back(-1.0, -1.0);
  }
  cases.push_back({"reverse", rev, {true, 0.5, -1.0, -1.0, 0.04}});
 }
 for (const double t_scale : t_scales) {
  for (const double tau : taus) {
   const bool want_ff = true;
   for (const auto& step : cases) {
    const RunMetrics m = run_steps(step.targets, kDt, params, gains, nominal, ff_base, robot.wheel_base_m,
                                   t_scale, tau, want_ff, step.meta);
    char tag[80];
    std::snprintf(tag, sizeof(tag), "step %s ff T=%.2f tau=%.2f", step.name, t_scale, tau);
    print_metrics(tag, m);
   }
   // Load change: heavy plant, nominal FF model and gains (the real case).
   sim::MotorDriveParams heavy = params;
   heavy.mass_kg *= 1.5;
   const RunMetrics loaded = run_steps(cases[0].targets, kDt, heavy, gains, nominal, ff_base,
                                       robot.wheel_base_m, t_scale, tau, want_ff, cases[0].meta);
   char tag[80];
   std::snprintf(tag, sizeof(tag), "step start-heavy ff T=%.2f tau=%.2f", t_scale, tau);
   print_metrics(tag, loaded);
  }
 }
 // PID-only step references (same scenarios, no FF).
 for (const auto& step : cases) {
  const RunMetrics m = run_steps(step.targets, kDt, params, gains, nominal, ff_base, robot.wheel_base_m, 1.0,
                                 0.0, false, step.meta);
  char tag[80];
  std::snprintf(tag, sizeof(tag), "step %s pid-only", step.name);
  print_metrics(tag, m);
 }
 std::cerr << "ff sensitivity green\n";
 return 0;
}
