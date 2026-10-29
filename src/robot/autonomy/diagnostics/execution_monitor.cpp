#include "robot/autonomy/diagnostics/execution_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::autonomy::diagnostics {
namespace {

bool finite_nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

double clamp_nonnegative(double value) {
  return finite_nonnegative(value) ? value : 0.0;
}

} // namespace

const char* execution_fault_name(ExecutionFault fault) {
  switch (fault) {
    case ExecutionFault::kNone:
      return "none";
    case ExecutionFault::kFeedbackMissing:
      return "feedback-missing";
    case ExecutionFault::kNoResponse:
      return "no-response";
    case ExecutionFault::kDirectionMismatch:
      return "direction-mismatch";
    case ExecutionFault::kUncommandedMotion:
      return "uncommanded-motion";
  }
  return "unknown";
}

bool ExecutionMonitorConfig::observable() const {
  return finite_nonnegative(velocity_deadband_mps) &&
         finite_nonnegative(yaw_rate_deadband_radps) &&
         finite_nonnegative(response_delay_s) &&
         finite_nonnegative(safety_reaction_time_s) &&
         finite_nonnegative(brake_deceleration_mps2) &&
         finite_nonnegative(confirmation_s) &&
         (velocity_deadband_mps > 0.0 || yaw_rate_deadband_radps > 0.0);
}

void ExecutionMonitor::reset() {
  armed_ = false;
  fault_ = ExecutionFault::kNone;
  pending_fault_ = ExecutionFault::kNone;
  pending_since_ns_ = core::kInvalidTime;
  pending_forward_mps_ = 0.0;
  pending_lateral_mps_ = 0.0;
  pending_yaw_rate_radps_ = 0.0;
  pending_speed_mps_ = 0.0;
}

void ExecutionMonitor::clear_fault() {
  fault_ = ExecutionFault::kNone;
  pending_fault_ = ExecutionFault::kNone;
  pending_since_ns_ = core::kInvalidTime;
  pending_forward_mps_ = 0.0;
  pending_lateral_mps_ = 0.0;
  pending_yaw_rate_radps_ = 0.0;
  pending_speed_mps_ = 0.0;
}

ExecutionMonitorReport ExecutionMonitor::update(
    const iface::MotionCommand& command, const EstimatedState& estimate,
    core::MonotonicNs now_ns) {
  ExecutionMonitorReport out;
  if (!config_.observable() || now_ns == core::kInvalidTime) {
    out.diagnostic = "execution monitor not configured";
    return out;
  }
  out.active = true;
  out.request_forward_mps = command.forward_mps;
  out.request_lateral_mps = command.space == iface::CommandSpace::kHolonomic
                                ? command.lateral_mps
                                : 0.0;
  out.request_yaw_rate_radps = command.yaw_rate_radps;
  out.measured_forward_mps = estimate.forward_mps;
  out.measured_lateral_mps = estimate.lateral_mps;
  out.measured_yaw_rate_radps = estimate.yaw_rate_radps;

  if (fault_ != ExecutionFault::kNone) {
    out.fault = fault_;
    out.stop_required = true;
    out.diagnostic = std::string("latched: ") + execution_fault_name(fault_);
    return out;
  }

  const double velocity_band = std::max(0.0, config_.velocity_deadband_mps);
  const double yaw_band = std::max(0.0, config_.yaw_rate_deadband_radps);
  const double response_window_s =
      std::max(0.0, config_.response_delay_s) +
      std::max(0.0, config_.confirmation_s);
  const bool command_expired =
      command.stamp_ns == core::kInvalidTime || command.expiry_ns <= now_ns;
  const double requested_forward = command_expired ? 0.0 : command.forward_mps;
  const double requested_lateral =
      command_expired ? 0.0 : out.request_lateral_mps;
  const double requested_yaw = command_expired ? 0.0 : command.yaw_rate_radps;

  ExecutionFault observed = ExecutionFault::kNone;
  if (!estimate.valid || estimate.stamp_ns == core::kInvalidTime ||
      estimate.stamp_ns > now_ns) {
    observed = ExecutionFault::kFeedbackMissing;
  } else {
    const double feedback_age_s =
        static_cast<double>(now_ns - estimate.stamp_ns) * 1e-9;
    out.feedback_age_s = feedback_age_s;
    if (feedback_age_s > response_window_s + 1e-9) {
      observed = ExecutionFault::kFeedbackMissing;
    } else {
      const bool forward_requested = std::abs(requested_forward) > velocity_band;
      const bool lateral_requested = std::abs(requested_lateral) > velocity_band;
      const bool translation_requested =
          requested_forward != 0.0 || requested_lateral != 0.0;
      const bool yaw_requested = std::abs(requested_yaw) > yaw_band;
      const bool linear_requested = forward_requested || lateral_requested;
      const bool linear_moving =
          std::abs(estimate.forward_mps) > 2.0 * velocity_band ||
          std::abs(estimate.lateral_mps) > 2.0 * velocity_band;
      const bool yaw_moving = std::abs(estimate.yaw_rate_radps) > 2.0 * yaw_band;

      if (linear_requested) {
        const double request_norm = std::hypot(requested_forward, requested_lateral);
        const double projection =
            (estimate.forward_mps * requested_forward +
             estimate.lateral_mps * requested_lateral) /
            std::max(request_norm, 1e-12);
        if (projection < -velocity_band) {
          observed = ExecutionFault::kDirectionMismatch;
        // The arrival deadband is an observation uncertainty, so apply it on
        // both sides of the request. A small command near that band is still
        // responding when the measured velocity is within one band of it;
        // requiring raw motion above the band falsely stops low-speed crawl.
        // A command materially above the band is considered absent only when
        // the measured projection is itself inside the deadband and the
        // command-to-feedback gap exceeds both sides of that uncertainty.
        } else if (projection <= velocity_band &&
                   request_norm - projection > 2.0 * velocity_band) {
          observed = ExecutionFault::kNoResponse;
        }
      }

      // During a moving arc, yaw response is continuous and lagging; a
      // per-axis yaw failure would misclassify normal curvature tracking.
      // With linear motion requested, command-following is judged on the
      // measured translation. Pure turn-in-place still gets the yaw gate.
      if (observed == ExecutionFault::kNone && !translation_requested &&
          yaw_requested) {
        const double measured_yaw = estimate.yaw_rate_radps;
        if (measured_yaw * requested_yaw < 0.0 &&
            std::abs(measured_yaw) > yaw_band) {
          observed = ExecutionFault::kDirectionMismatch;
        } else if (std::abs(measured_yaw) <= yaw_band &&
                   std::abs(requested_yaw) - std::abs(measured_yaw) >
                       2.0 * yaw_band) {
          observed = ExecutionFault::kNoResponse;
        }
      }

      if (observed == ExecutionFault::kNone && !linear_requested &&
          !yaw_requested && (linear_moving || yaw_moving)) {
        observed = ExecutionFault::kUncommandedMotion;
      }
    }
  }

  if (observed == ExecutionFault::kNone) {
    pending_fault_ = ExecutionFault::kNone;
    pending_since_ns_ = core::kInvalidTime;
    out.diagnostic = "tracking";
    return out;
  }

  // Feedback age has already exceeded the complete response+confirmation
  // window when this class is observed. Requiring another confirmation
  // interval would leave the robot blind and moving for twice the declared
  // reaction budget.
  if (observed == ExecutionFault::kFeedbackMissing) {
    fault_ = observed;
    out.fault = fault_;
    out.stop_required = true;
    out.diagnostic = std::string("latched: ") + execution_fault_name(fault_);
    return out;
  }

  const bool request_changed =
      std::abs(requested_forward - pending_forward_mps_) > velocity_band ||
      std::abs(requested_lateral - pending_lateral_mps_) > velocity_band ||
      std::abs(requested_yaw - pending_yaw_rate_radps_) > yaw_band;
  if (!armed_ || pending_fault_ != observed || request_changed ||
      pending_since_ns_ == core::kInvalidTime || now_ns < pending_since_ns_) {
    armed_ = true;
    pending_fault_ = observed;
    pending_since_ns_ = now_ns;
    pending_forward_mps_ = requested_forward;
    pending_lateral_mps_ = requested_lateral;
    pending_yaw_rate_radps_ = requested_yaw;
    pending_speed_mps_ = clamp_nonnegative(
        std::hypot(estimate.forward_mps, estimate.lateral_mps));
  }

  // The window is the budget for the whole episode, so the braking allowance
  // is derived from the speed at which the episode began. Motion opposite to
  // a new request, or motion under a stop command, is allowed to dissipate
  // under the declared brake deceleration after the declared delay. Deriving
  // the allowance from the current speed instead would shrink it as fast as
  // a healthy stop slows the robot and latch any stop from above
  // brake * (reaction + confirmation). The same rule means a robot that
  // accelerates without a request does not earn more time by speeding up.
  double fault_window_s = response_window_s +
      std::max(0.0, config_.safety_reaction_time_s);
  if ((observed == ExecutionFault::kDirectionMismatch ||
       observed == ExecutionFault::kUncommandedMotion) &&
      config_.brake_deceleration_mps2 > 0.0) {
    const double translational_stop_s =
        pending_speed_mps_ / config_.brake_deceleration_mps2;
    fault_window_s = std::max(
        fault_window_s,
        std::max(0.0, config_.response_delay_s) +
            std::max(0.0, config_.safety_reaction_time_s) +
            translational_stop_s +
            std::max(0.0, config_.confirmation_s));
  }
  const core::MonotonicNs fault_window_ns =
      static_cast<core::MonotonicNs>(std::llround(fault_window_s * 1e9));

  out.pending_fault = pending_fault_;
  out.pending_elapsed_s =
      static_cast<double>(now_ns - pending_since_ns_) * 1e-9;
  if (now_ns - pending_since_ns_ >= fault_window_ns) {
    fault_ = observed;
    out.fault = fault_;
    out.stop_required = true;
    out.diagnostic = std::string("latched: ") + execution_fault_name(fault_);
  } else {
    out.diagnostic = std::string("pending: ") + execution_fault_name(observed);
  }
  return out;
}

} // namespace lost_signal::robot::autonomy::diagnostics
