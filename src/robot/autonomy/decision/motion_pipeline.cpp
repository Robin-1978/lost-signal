#include "robot/autonomy/decision/motion_pipeline.hpp"

#include <array>
#include <cmath>
#include <limits>

namespace lost_signal::robot::autonomy {

MotionDecisionResult MotionDecisionPipeline::run(
 const EstimatedState& est, const PerceptionResult& perception,
 const LocalMap& local_map, const Route& route,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation,
 const SurveyContext& survey, core::MonotonicNs now_ns,
 IBehaviorPolicy& behavior,
 ILocalPlanner& local_planner, IValidator& validator,
 IController& controller) {
 MotionDecisionResult out;
 out.command = iface::MotionCommand::SafeStop(now_ns);
 out.behavior = behavior.decide(est, perception, behavior_limits);
 out.constraint = out.behavior.must_hold ? "hold" : "speed_cap";

 if (out.behavior.must_hold) {
 LocalCandidate hold;
 hold.feasible = true;
 const ValidationResult verdict =
 validator.validate(hold, est, perception, observation, survey);
 out.reason = verdict.reason;
 if (verdict.accepted) out.command = controller.track(hold, est, now_ns);
 return out;
 }

 platform::ChassisLimits effective = limits;
 if (out.behavior.max_forward_mps > 0.0) {
 effective.max_forward_mps =
 std::min(effective.max_forward_mps, out.behavior.max_forward_mps);
 }
 const LocalPlan plan =
 local_planner.plan(est, perception, local_map, route, effective);
 std::array<LocalCandidate, 12> recovery{};
 const double crawl = std::min(0.25, std::max(0.0, effective.max_forward_mps));
 const double turn = std::min(0.6, std::max(0.0, effective.max_yaw_rate_radps));
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
 // A car-like base cannot pivot in place. Keep pure yaw recovery for
 // differential/holonomic envelopes only; Ackermann recovery uses a moving
 // arc below, and DWA itself enforces the same curvature bound.
 if (!(effective.max_curvature_inv_m > 0.0)) {
 add_recovery(0.0, turn, 0.0);
 add_recovery(0.0, -turn, 0.0);
 }
 add_recovery(crawl, turn, 0.0);
 add_recovery(crawl, -turn, 0.0);
 add_recovery(-crawl, turn, 0.0);
 add_recovery(-crawl, -turn, 0.0);
 if (effective.max_lateral_mps > 0.0) {
 const double slide = std::min(0.25, effective.max_lateral_mps);
 add_recovery(0.0, 0.0, slide);
 add_recovery(0.0, 0.0, -slide);
 }

 bool planner_rejected = false;
 ValidationResult best_recovery;
 LocalCandidate chosen;
 bool have_recovery = false;
 double best_recovery_score = -std::numeric_limits<double>::infinity();
 if (plan.has_plan) {
 best_recovery = validator.validate(
 plan.best, est, perception, observation, survey);
 out.reason = best_recovery.reason;
 if (best_recovery.accepted) {
 out.command = controller.track(plan.best, est, now_ns);
 } else {
 planner_rejected = true;
 }
 }

 const bool recovery_localized = est.valid && est.confidence >= 0.75 &&
 est.position_sigma_m <= 0.5 && est.heading_sigma_rad <= 0.75;
 const bool local_evidence =
 local_map.valid && local_map.provenance != LocalMapProvenance::kUnknown;
 if (planner_rejected && local_evidence && recovery_localized) {
 for (std::size_t i = 0; i < recovery_count; ++i) {
 const ValidationResult verdict = validator.validate(
 recovery[i], est, perception, observation, survey);
 if (!verdict.accepted) continue;
 const double horizon_s = 1.0;
 const double px = est.x_m +
 (recovery[i].forward_mps * std::cos(est.yaw_rad) -
 recovery[i].lateral_mps * std::sin(est.yaw_rad)) * horizon_s;
 const double py = est.y_m +
 (recovery[i].forward_mps * std::sin(est.yaw_rad) +
 recovery[i].lateral_mps * std::cos(est.yaw_rad)) * horizon_s;
 double route_score = 0.0;
 if (route.valid && !route.points.empty()) {
 const auto& target = route.points.back();
 route_score = -std::hypot(target.x_m - px, target.y_m - py);
 }
 const double clearance = std::isfinite(verdict.min_clearance_m)
 ? std::min(2.0, verdict.min_clearance_m) : 2.0;
 const double score = route_score + 0.05 * clearance;
 if (!have_recovery || score > best_recovery_score) {
 have_recovery = true;
 best_recovery_score = score;
 best_recovery = verdict;
 chosen = recovery[i];
 }
 }
 if (have_recovery) {
 out.command = controller.track(chosen, est, now_ns);
 out.reason = "recovery:" + best_recovery.reason;
 } else if (!plan.has_plan) {
 out.reason = "no-plan; recovery rejected";
 } else {
 out.reason = "planner candidate rejected; recovery rejected";
 }
 }
 if (!iface::valid_motion_command(out.command, now_ns)) {
 out.command = iface::MotionCommand::SafeStop(now_ns);
 }
 return out;
}

} // namespace lost_signal::robot::autonomy
