#include "robot/autonomy/decision/motion_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace lost_signal::robot::autonomy {

void MotionDecisionPipeline::reset() {
 escape_active_ = false;
 escape_command_ = LocalCandidate{};
 escape_release_count_ = 0;
}

MotionDecisionResult MotionDecisionPipeline::run(
 const EstimatedState& est, const PerceptionResult& perception,
 const LocalMap& local_map, const Route& route,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation,
 const SurveyContext& survey, core::MonotonicNs now_ns,
 IBehaviorPolicy& behavior,
 ILocalPlanner& local_planner, IValidator& validator,
 IController& controller, bool forward_blocked,
 bool observation_motion_requested) {
 MotionDecisionResult out;
 out.command = iface::MotionCommand::SafeStop(now_ns);
 const PredictionBatch prediction = predict_perception(
 perception, now_ns * 1e-9);
 out.prediction_horizon_s = prediction.future_times_s.empty()
 ? 0.0 : prediction.future_times_s.back();
 out.prediction_hypotheses = prediction.hypotheses.size();
 out.behavior = behavior.decide(
 est, perception, prediction, route, behavior_limits);
 out.constraint = out.behavior.must_hold ? "hold" : "speed_cap";

 LocalCandidate hold;
 hold.feasible = true;
 const ValidationResult hold_verdict = validator.validate(
 hold, est, perception, prediction, observation, survey);
 const bool hold_dynamically_unsafe =
 hold_verdict.verdict == TrajectoryVerdict::kDynamicCollision;

 // Keep an escape direction until holding has a stable predicted margin.
 // This prevents one noisy track frame from flipping direction every cycle.
 if (escape_active_) {
 const bool releasable = hold_verdict.accepted &&
 (prediction.hypotheses.empty() ||
 hold_verdict.min_dynamic_clearance_m >=
 config_.escape_release_clearance_m);
 escape_release_count_ = releasable ? escape_release_count_ + 1 : 0;
 if (escape_release_count_ >= std::max<std::size_t>(
 1, config_.escape_release_cycles)) {
 escape_active_ = false;
 escape_release_count_ = 0;
 escape_command_ = LocalCandidate{};
 }
 }

 if (out.behavior.must_hold && hold_verdict.accepted && !escape_active_) {
 out.reason = out.behavior.reason.empty()
 ? hold_verdict.reason
 : out.behavior.reason;
 out.command = controller.track(hold, est, now_ns);
 return out;
 }

 platform::ChassisLimits effective = limits;
 if (out.behavior.max_forward_mps > 0.0) {
 effective.max_forward_mps =
 std::min(effective.max_forward_mps, out.behavior.max_forward_mps);
 }
 const LocalPlan plan = local_planner.plan(
 est, perception, prediction, local_map, route, effective);
 out.route_replan_requested = plan.route_replan_requested;

 struct ValidatedCandidate {
 LocalCandidate candidate;
 ValidationResult verdict;
 std::size_t order = 0;
 };
 std::vector<ValidatedCandidate> planner_valid;
 std::vector<LocalCandidate> planner_candidates = plan.candidates;
 if (planner_candidates.empty() && plan.has_plan && plan.best.feasible) {
 planner_candidates.push_back(plan.best);
 }
 planner_valid.reserve(planner_candidates.size());
 ValidationResult first_rejection;
 bool have_rejection = false;
 for (std::size_t i = 0; i < planner_candidates.size(); ++i) {
 const auto verdict = validator.validate(
 planner_candidates[i], est, perception, prediction, observation, survey);
 if (verdict.accepted) {
 planner_valid.push_back({planner_candidates[i], verdict, i});
 // Candidates arrive in planner rank order and a normal acceptance outranks
 // every minimum-risk escape. Once the first fully accepted trajectory is
 // found, no later proposal can change the selection; validating the rest
 // only repeats raw-scan rollouts inside the control tick.
 if (verdict.verdict == TrajectoryVerdict::kAccepted) break;
 } else if (!have_rejection) {
 first_rejection = verdict;
 have_rejection = true;
 }
 }

 // Safety class first, planner cost second. A rejected top score no longer
 // turns a non-empty dynamic window into a transient stop.
 const auto planner_better = [](const ValidatedCandidate& a,
 const ValidatedCandidate& b) {
 const int ar = a.verdict.verdict == TrajectoryVerdict::kAccepted ? 1 : 0;
 const int br = b.verdict.verdict == TrajectoryVerdict::kAccepted ? 1 : 0;
 return ar != br ? ar > br : a.order < b.order;
 };
 std::stable_sort(planner_valid.begin(), planner_valid.end(), planner_better);

 const bool dynamic_escape_required = escape_active_ ||
 hold_dynamically_unsafe ||
 out.behavior.reason == "route-conflict-no-safe-hold";
 const bool planner_requested = route.valid && !route.points.empty();
 const bool planner_motion_available = std::any_of(
 planner_valid.begin(), planner_valid.end(),
 [](const ValidatedCandidate& candidate) {
 return std::abs(candidate.candidate.forward_mps) > 1e-9 ||
 std::abs(candidate.candidate.yaw_rate_radps) > 1e-9 ||
 std::abs(candidate.candidate.lateral_mps) > 1e-9;
 });
 const bool observation_recovery_required =
 observation_motion_requested && !planner_motion_available;
 const bool recovery_required =
 (planner_requested && planner_valid.empty()) || forward_blocked ||
 dynamic_escape_required || out.behavior.must_hold ||
 observation_recovery_required;
 if (!planner_requested && !recovery_required) {
 out.constraint = "hold";
 out.reason = hold_verdict.accepted ? hold_verdict.reason : "no-route-hold";
 if (hold_verdict.accepted) {
 out.command = controller.track(hold, est, now_ns);
 }
 return out;
 }
 if (!recovery_required && !planner_valid.empty()) {
 auto selected_it = planner_valid.begin();
 if (observation_motion_requested) {
 const auto moving = std::find_if(
 planner_valid.begin(), planner_valid.end(),
 [](const ValidatedCandidate& candidate) {
 return std::abs(candidate.candidate.forward_mps) > 1e-9 ||
 std::abs(candidate.candidate.yaw_rate_radps) > 1e-9 ||
 std::abs(candidate.candidate.lateral_mps) > 1e-9;
 });
 if (moving != planner_valid.end()) selected_it = moving;
 }
 const auto& selected = *selected_it;
 out.validated_static_clearance_m = selected.verdict.min_clearance_m;
 out.validated_dynamic_clearance_m = selected.verdict.min_dynamic_clearance_m;
 out.trajectory_verdict = selected.verdict.verdict;
 out.reason = selected.verdict.reason;
 if (selected.candidate.forward_mps == 0.0 &&
 selected.candidate.yaw_rate_radps == 0.0 &&
 selected.candidate.lateral_mps == 0.0 && route.valid) {
 out.reason = "planner-hold:moving=" +
 std::to_string(plan.moving_candidate_count) +
 ";gear=" + std::to_string(plan.requested_gear) +
 ";route-error=" + std::to_string(plan.route_error_m) +
 ";segment=" + std::to_string(plan.route_segment) +
 ";cusp=" + (plan.gear_change_pending ? "pending" : "none") +
 "/" + (plan.gear_change_braking_armed ? "armed" : "approach") +
 ";cusp-distance=" +
 std::to_string(plan.gear_change_endpoint_distance_m);
 }
 out.command = controller.track(selected.candidate, est, now_ns);
 if (!iface::valid_motion_command(out.command, now_ns)) {
 out.command = iface::MotionCommand::SafeStop(now_ns);
 }
 return out;
 }

 std::array<LocalCandidate, 14> recovery{};
 const double crawl = std::min(
 std::max(0.0, config_.recovery_speed_mps),
 std::max(0.0, effective.max_forward_mps));
 const double turn = std::min(
 std::max(0.0, config_.recovery_yaw_rate_radps),
 std::max(0.0, effective.max_yaw_rate_radps));
 const double recovery_turn = effective.max_curvature_inv_m > 0.0
 ? std::min(turn, crawl * effective.max_curvature_inv_m)
 : turn;
 std::size_t recovery_count = 0;
 auto add_recovery = [&](double forward, double yaw, double lateral) {
 if (recovery_count >= recovery.size()) return;
 if (std::abs(forward) < 1e-9 && std::abs(yaw) < 1e-9 &&
 std::abs(lateral) < 1e-9) return;
 LocalCandidate candidate;
 candidate.forward_mps = forward;
 candidate.yaw_rate_radps = yaw;
 candidate.lateral_mps = lateral;
 candidate.feasible = true;
 recovery[recovery_count++] = candidate;
 };
 add_recovery(-crawl, 0.0, 0.0);
 add_recovery(crawl, 0.0, 0.0);
 if (!(effective.max_curvature_inv_m > 0.0)) {
 add_recovery(0.0, turn, 0.0);
 add_recovery(0.0, -turn, 0.0);
 }
 add_recovery(crawl, recovery_turn, 0.0);
 add_recovery(crawl, -recovery_turn, 0.0);
 add_recovery(-crawl, recovery_turn, 0.0);
 add_recovery(-crawl, -recovery_turn, 0.0);
 if (effective.max_lateral_mps > 0.0) {
 const double slide = std::min(crawl, effective.max_lateral_mps);
 add_recovery(0.0, 0.0, slide);
 add_recovery(0.0, 0.0, -slide);
 add_recovery(-crawl, 0.0, slide);
 add_recovery(-crawl, 0.0, -slide);
 }

 const bool local_evidence =
 local_map.valid && local_map.provenance != LocalMapProvenance::kUnknown;
 ValidatedCandidate best_recovery;
 bool have_recovery = false;
 struct RecoveryQuality {
 bool nominally_safe = false;
 bool clears_forward_block = false;
 double dynamic_clearance_m = -std::numeric_limits<double>::infinity();
 double escape_gain_m = 0.0;
 int continuity_axes = 0;
 double route_distance_m = std::numeric_limits<double>::infinity();
 double static_clearance_m = -std::numeric_limits<double>::infinity();
 } best_quality;
 const auto greater = [](double a, double b) {
 return a > b + 1e-9;
 };
 const auto less = [](double a, double b) {
 return a + 1e-9 < b;
 };
 const auto better_quality = [&](const RecoveryQuality& a,
 const RecoveryQuality& b) {
 if (a.nominally_safe != b.nominally_safe)
 return a.nominally_safe;
 if (a.clears_forward_block != b.clears_forward_block)
 return a.clears_forward_block;
 if (dynamic_escape_required &&
 (greater(a.dynamic_clearance_m, b.dynamic_clearance_m) ||
 greater(b.dynamic_clearance_m, a.dynamic_clearance_m))) {
 return greater(a.dynamic_clearance_m, b.dynamic_clearance_m);
 }
 if (greater(a.escape_gain_m, b.escape_gain_m) ||
 greater(b.escape_gain_m, a.escape_gain_m)) {
 return greater(a.escape_gain_m, b.escape_gain_m);
 }
 if (a.continuity_axes != b.continuity_axes)
 return a.continuity_axes > b.continuity_axes;
 if (less(a.route_distance_m, b.route_distance_m) ||
 less(b.route_distance_m, a.route_distance_m)) {
 return less(a.route_distance_m, b.route_distance_m);
 }
 return greater(a.static_clearance_m, b.static_clearance_m);
 };
 // Recovery is expressed and validated in the current sensor-local frame:
 // raw scan endpoints, ego rollout and tracked objects all undergo the same
 // map-frame transform. Global map covariance therefore cannot make their
 // relative clearance less observable. Requiring a small global sigma here
 // used to create a permanent stop exactly when motion was needed to expose
 // geometry for relocalization. Keep the valid-pose and local-evidence gates;
 // every crawl candidate still passes the independent static, dynamic,
 // braking and terrain validator before it can reach the controller.
 if (recovery_required && est.valid &&
 (local_evidence || forward_blocked)) {
 for (std::size_t i = 0; i < recovery_count; ++i) {
 const ValidationResult verdict = validator.validate(
 recovery[i], est, perception, prediction, observation, survey);
 if (!verdict.accepted) continue;

 RecoveryQuality quality;
 quality.nominally_safe =
 verdict.verdict == TrajectoryVerdict::kAccepted;
 quality.clears_forward_block =
 !forward_blocked || recovery[i].forward_mps <= 1e-6;
 quality.dynamic_clearance_m = verdict.min_dynamic_clearance_m;
 quality.escape_gain_m = verdict.escape_clearance_gain_m;
 quality.static_clearance_m = verdict.min_clearance_m;
 if (escape_active_) {
 const auto same_direction = [](double current, double committed) {
 if (std::abs(committed) < 1e-9) return std::abs(current) < 1e-9;
 return current * committed > 0.0;
 };
 quality.continuity_axes += same_direction(
 recovery[i].forward_mps, escape_command_.forward_mps);
 quality.continuity_axes += same_direction(
 recovery[i].yaw_rate_radps, escape_command_.yaw_rate_radps);
 quality.continuity_axes += same_direction(
 recovery[i].lateral_mps, escape_command_.lateral_mps);
 }
 if (route.valid && !route.points.empty()) {
 const double horizon_s = std::max(
 0.0, config_.recovery_projection_horizon_s);
 const double px = est.x_m +
 (recovery[i].forward_mps * std::cos(est.yaw_rad) -
 recovery[i].lateral_mps * std::sin(est.yaw_rad)) * horizon_s;
 const double py = est.y_m +
 (recovery[i].forward_mps * std::sin(est.yaw_rad) +
 recovery[i].lateral_mps * std::cos(est.yaw_rad)) * horizon_s;
 const auto& target = route.points.back();
 quality.route_distance_m =
 std::hypot(target.x_m - px, target.y_m - py);
 }
 if (!have_recovery || better_quality(quality, best_quality)) {
 have_recovery = true;
 best_quality = quality;
 best_recovery = {recovery[i], verdict, i};
 }
 }
 }

 if (have_recovery) {
 if (dynamic_escape_required) {
 const bool starting_escape = !escape_active_;
 escape_active_ = true;
 if (starting_escape) escape_release_count_ = 0;
 escape_command_ = best_recovery.candidate;
 out.constraint = "dynamic_escape";
 } else if (observation_recovery_required) {
 out.constraint = "localization_observe";
 }
 out.command = controller.track(best_recovery.candidate, est, now_ns);
 out.validated_static_clearance_m = best_recovery.verdict.min_clearance_m;
 out.validated_dynamic_clearance_m =
 best_recovery.verdict.min_dynamic_clearance_m;
 out.trajectory_verdict = best_recovery.verdict.verdict;
 out.reason = "recovery:" + best_recovery.verdict.reason;
 if (planner_candidates.empty()) {
 out.reason += "; planner=no-candidate; gear=" +
 std::to_string(plan.requested_gear) +
 ";route-error=" + std::to_string(plan.route_error_m);
 } else if (have_rejection) {
 out.reason += "; planner=" + first_rejection.reason;
 }
 } else if (!planner_valid.empty() && !forward_blocked &&
 !dynamic_escape_required) {
 const auto& selected = planner_valid.front();
 out.command = controller.track(selected.candidate, est, now_ns);
 out.validated_static_clearance_m = selected.verdict.min_clearance_m;
 out.validated_dynamic_clearance_m = selected.verdict.min_dynamic_clearance_m;
 out.trajectory_verdict = selected.verdict.verdict;
 out.reason = selected.verdict.reason;
 } else if (have_rejection) {
 out.reason = "planner candidates rejected: " + first_rejection.reason;
 } else {
 out.reason = "no-plan; recovery rejected";
 }

 if (!iface::valid_motion_command(out.command, now_ns)) {
 out.command = iface::MotionCommand::SafeStop(now_ns);
 }
 return out;
}

} // namespace lost_signal::robot::autonomy
