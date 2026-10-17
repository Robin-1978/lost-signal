// RO-2 pocket shadow (observation only, mission untouched).
// Predicted: assess_transit with RO-2 caps (fillet + servo-lag bicycle at
// instant cruise). Actual: production AckermannDrive rollout following speed
// and steering-angle commands derived from the predicted trajectory, through
// drive PID + shared contact physics. The deviation measures exactly the
// documented approximation (instant cruise, no slip) — model-consistency
// evidence, never a release license.
// Estimator shadow: ground-truth stream (latency 0.05 s) through
// EstimateHolder; the episode records status/age/trace + reporting confidence.

#include "robot/autonomy/planning/local/pocket_shadow.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"
#include "robot/autonomy/localization/state_estimator.hpp"
#include "robot/core/math.hpp"
#include "sim/hardware/vehicle/ackermann.hpp"
#include "sim/hardware/vehicle/motor_drive.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

const sim::ChassisExecCaps kRo2{false, 0.0, 0.62, 0.55, 2.0, 0.35};
constexpr double kCruise = 1.0;
constexpr double kDt = 0.02;

sim::AckermannDrive make_ro2() {
  sim::AckermannParams params;
  params.wheel_base_m = 0.62;
  params.wheel_radius_m = 0.1;
  params.track_width_m = 0.58;
  params.max_steer_rad = 0.55;
  params.max_steer_rate_radps = 2.0;
  params.servo_response_s = 0.35;
  params.max_drive_speed_mps = 2.4;
  params.max_drive_accel_mps2 = 6.0;
  params.stall_force_N = 54.0;
  params.free_speed_mps = 3.0;
  params.mass_kg = 26.0;
  params.pwm_limit = 1.0;
  params.integral_limit = 2.0;
  params.encoder_counts_per_revolution = 2048;
  const double wheel_mass = std::max(0.5, params.mass_kg / 2.0);
  const sim::WheelPidGains gains = sim::tune_wheel_pid(wheel_mass, params.stall_force_N,
                                                       params.free_speed_mps, 0.5 * 2.4);
  const double servo_kp = 1.0 / std::max(0.05, params.servo_response_s * params.max_steer_rate_radps);
  return sim::AckermannDrive(params, gains, servo_kp);
}

std::vector<std::vector<sim::DynamicObjectState>> empty_frames(std::size_t n) {
  return std::vector<std::vector<sim::DynamicObjectState>>(n, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
}

// Steering-angle sequence from a timed position reference (finite difference).
std::vector<sim::DiffCmd> derive_commands(const std::vector<sim::TimedPose>& ref) {
  std::vector<sim::DiffCmd> cmds;
  if (ref.size() < 2) {
    return cmds;
  }
  std::vector<double> heading(ref.size(), 0.0);
  for (std::size_t i = 0; i + 1 < ref.size(); ++i) {
    heading[i] = std::atan2(ref[i + 1].p.y - ref[i].p.y, ref[i + 1].p.x - ref[i].p.x);
  }
  heading.back() = heading[heading.size() - 2];
  for (std::size_t i = 0; i + 1 < ref.size(); ++i) {
    const double dt = std::max(1e-6, ref[i + 1].t_s - ref[i].t_s);
    const double dist = std::hypot(ref[i + 1].p.x - ref[i].p.x, ref[i + 1].p.y - ref[i].p.y);
    const double v = dist / dt;
 const double yaw_rate = sim::wrap_angle(heading[i + 1] - heading[i]) / dt;
 const double steer = sim::steer_for_twist(yaw_rate, v, kRo2.wheel_base_m,
 kRo2.max_steer_rad).delta_rad;
 cmds.push_back({v, steer});
  }
  return cmds;
}

// Production rollout: same twist reference through drive PID + contact.
std::vector<sim::TimedPose> roll_production(const std::vector<sim::TimedPose>& ref, sim::Vec2 start,
                                            double start_theta) {
  std::vector<sim::TimedPose> actual;
  if (ref.empty()) {
    return actual;
  }
  sim::AckermannDrive car = make_ro2();
  sim::DiffState state{start.x, start.y, start_theta, 0.0, 0.0};
  const sim::GroundContact ground{0.7, 0.01, 6.0};
  actual.push_back({0.0, {state.x, state.y}});
 const auto cmds = derive_commands(ref);
  double t = 0.0;
  for (const auto& cmd : cmds) {
    state = car.step(state, cmd, kDt, ground);
    t += kDt;
    actual.push_back({t, {state.x, state.y}});
  }
  return actual;
}

sim::PropagatedEstimate estimator_stream_view(const std::vector<sim::TimedPose>& actual, double* age_out) {
  sim::EstimateHolder holder;
  // Ground-truth stream with producer-set latency: pose from the rollout,
  // velocity from finite difference (honest: the stream carries what the
  // body did, not zeros). No filtering here — the holder only manages time.
  std::vector<double> heading(actual.size(), 0.0);
  for (std::size_t i = 0; i + 1 < actual.size(); ++i) {
    heading[i] =
      std::atan2(actual[i + 1].p.y - actual[i].p.y, actual[i + 1].p.x - actual[i].p.x);
  }
  if (!actual.empty()) {
    heading.back() = actual.size() > 1 ? heading[heading.size() - 2] : 0.0;
  }
  for (std::size_t i = 0; i < actual.size(); ++i) {
    const auto& s = actual[i];
    sim::StateEstimate estimate;
    estimate.stamp_s = s.t_s;
    estimate.latency_s = 0.05;
    double vx = 0.0, vy = 0.0, w = 0.0;
    if (i + 1 < actual.size()) {
      const double dt = std::max(1e-6, actual[i + 1].t_s - s.t_s);
      vx = (actual[i + 1].p.x - s.p.x) / dt;
      vy = (actual[i + 1].p.y - s.p.y) / dt;
      w = sim::wrap_angle(heading[i + 1] - heading[i]) / dt;
    }
    estimate.state = {s.p.x, s.p.y, heading[i], vx, vy, w};
    for (std::size_t i = 0; i < 36; ++i) {
      estimate.covariance[i] = (i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) ? 0.01 : 0.0;
    }
    estimate.wheel_valid = true;
    estimate.imu_valid = true;
    estimate.lidar_valid = false;
    holder.push(estimate);
  }
  const double now = actual.empty() ? 0.0 : actual.back().t_s;
  auto view = holder.at(now, 0.1, sim::DropoutPolicy::kHoldLast);
  if (age_out != nullptr) {
    *age_out = 0.0;
  }
  return view;
}

#define RUN_CHECK(condition)                                                                                 \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return sim::PocketShadowEpisode{};                                                                        \
  }                                                                                                          \
 } while (false)

sim::PocketShadowEpisode run_case(const char* name, sim::GridMap& map, sim::Vec2 start, double theta,
                                    sim::Vec2 pocket) {
  const auto frames = empty_frames(161);
  const auto predicted = sim::assess_transit(map, start, theta, pocket, kCruise, kRo2, frames, 0.1, 0.38, 0.5);
  std::cerr << name << ": transit verdict=" << static_cast<int>(predicted.verdict)
            << " reason=" << static_cast<int>(predicted.reason) << " end="
            << predicted.end_s << " gap=" << predicted.min_gap_m
            << " ntraj=" << predicted.exec_traj.size() << "\n";
  RUN_CHECK(predicted.verdict == sim::Feasibility::Verdict::kFeasible);
  RUN_CHECK(!predicted.exec_traj.empty());
  const auto actual = roll_production(predicted.exec_traj, start, theta);
  RUN_CHECK(!actual.empty());
  double age = 0.0;
  const auto view = estimator_stream_view(actual, &age);
  RUN_CHECK(view.status == sim::EstimateStatus::kFresh);
  const auto episode = sim::make_pocket_shadow_episode(predicted, actual, view, age);
  std::cerr << name << ": episode actual_end=" << episode.actual_end_s
            << " max_dev=" << episode.error.max_dev_m << " rmse=" << episode.error.rmse_m
            << " endpoint=" << episode.error.endpoint_err_m << " time_bias=" << episode.error.time_bias_s
            << " conf=" << episode.state_confidence << " trace=" << episode.covariance_trace
            << " verdict=" << sim::shadow_verdict_name(episode.verdict) << "\n";
  RUN_CHECK(std::isfinite(episode.error.max_dev_m) && std::isfinite(episode.error.rmse_m));
  RUN_CHECK(episode.state_confidence > 0.0 && episode.state_confidence <= 1.0);
  RUN_CHECK(episode.estimator_status == sim::EstimateStatus::kFresh);
  RUN_CHECK(episode.verdict == sim::ShadowVerdict::kObservedTracked ||
        episode.verdict == sim::ShadowVerdict::kObservedDeviated);
  return episode;
}

} // namespace

int main() {
  sim::GridMap open(30, 30, 0.1);
  // Pinned finding (deterministic sim, bit-identical): the assessed transit
  // assumes instant cruise, production spools up from standstill through PID +
  // contact — ~0.3 m of start-lag on both geometries. Durations match by
  // construction (same 0.02 s stepping), so the lag appears spatially and
  // time_bias is 0; a future duration-aware harness would move it into time.
  // ObservedDeviated is model-consistency evidence, never a release input.
  const auto straight = run_case("straight", open, {0.05, 0.05}, 0.0, {1.95, 0.05});
  CHECK(straight.verdict == sim::ShadowVerdict::kObservedDeviated);
  CHECK(straight.error.max_dev_m > 0.28 && straight.error.max_dev_m < 0.40);
  CHECK(straight.error.endpoint_err_m > 0.28 && straight.error.endpoint_err_m < 0.40);
  CHECK(straight.error.time_bias_s == 0.0);
  const auto turn = run_case("turn90", open, {0.05, 0.05}, 0.0, {0.05, 1.95});
  CHECK(turn.verdict == sim::ShadowVerdict::kObservedDeviated);
  CHECK(turn.error.max_dev_m > 0.33 && turn.error.max_dev_m < 0.46);
  CHECK(turn.error.endpoint_err_m > 0.33 && turn.error.endpoint_err_m < 0.46);
  sim::GridMap lwall(30, 30, 0.1);
  lwall.addBox(0.9, -1.0, 1.1, 1.0);
  // Tight detour: reported, not pinned to Tracked (swing may deviate).
  {
    const auto frames = empty_frames(161);
    const auto predicted =
      sim::assess_transit(lwall, {0.05, 0.05}, 0.0, {2.05, 2.05}, kCruise, kRo2, frames, 0.1, 0.38, 0.5);
    std::cerr << "detour: transit verdict=" << static_cast<int>(predicted.verdict)
              << " reason=" << static_cast<int>(predicted.reason) << "\n";
    if (predicted.verdict == sim::Feasibility::Verdict::kFeasible) {
      const auto actual = roll_production(predicted.exec_traj, {0.05, 0.05}, 0.0);
      double age = 0.0;
      const auto view = estimator_stream_view(actual, &age);
      const auto episode = sim::make_pocket_shadow_episode(predicted, actual, view, age);
      std::cerr << "detour: episode max_dev=" << episode.error.max_dev_m
                << " endpoint=" << episode.error.endpoint_err_m
                << " verdict=" << sim::shadow_verdict_name(episode.verdict) << "\n";
      CHECK(std::isfinite(episode.error.max_dev_m));
    }
  }
  // Empty-input honesty: Invalid, never Tracked/Deviated.
  {
    sim::LegAssessment empty_pred;
    const std::vector<sim::TimedPose> empty_actual;
    sim::EstimateHolder holder;
    const auto view = holder.at(0.0, 1.0, sim::DropoutPolicy::kHoldLast);
    const auto episode = sim::make_pocket_shadow_episode(empty_pred, empty_actual, view, 0.0);
    CHECK(episode.verdict == sim::ShadowVerdict::kInvalid);
    CHECK(episode.state_confidence == 0.0);
  }
  std::cerr << "ro2 pocket shadow green\n";
  return 0;
}
