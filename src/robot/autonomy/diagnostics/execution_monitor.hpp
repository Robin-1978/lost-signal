#pragma once

#include <cstdint>
#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/iface/command.hpp"

namespace lost_signal::robot::autonomy::diagnostics {

// Command-versus-feedback failure classes. They are ROBOT ownership
// conclusions: they use only the accepted body command and the ROBOT's
// measured state. No simulator contact, slip, collision, or actor truth is
// part of this decision.
enum class ExecutionFault {
  kNone = 0,
  kFeedbackMissing,
  kNoResponse,
  kDirectionMismatch,
  kUncommandedMotion,
};

const char* execution_fault_name(ExecutionFault fault);

// The velocity deadbands are existing arrival-policy measurement bands. The
// response window and confirmation interval are the declared actuator delay
// and safety reaction time from the machine description. They are durations
// with physical meaning, not scenario-specific cycle counts.
struct ExecutionMonitorConfig {
  double velocity_deadband_mps = 0.0;
  double yaw_rate_deadband_radps = 0.0;
  double response_delay_s = 0.0;
  double safety_reaction_time_s = 0.0;
  double brake_deceleration_mps2 = 0.0;
  double confirmation_s = 0.0;

  bool observable() const;
};

struct ExecutionMonitorReport {
  bool active = false;
  bool stop_required = false;
  ExecutionFault fault = ExecutionFault::kNone;
  ExecutionFault pending_fault = ExecutionFault::kNone;
  double pending_elapsed_s = 0.0;
  double request_forward_mps = 0.0;
  double request_lateral_mps = 0.0;
  double request_yaw_rate_radps = 0.0;
  double measured_forward_mps = 0.0;
  double measured_lateral_mps = 0.0;
  double measured_yaw_rate_radps = 0.0;
  double feedback_age_s = 0.0;
  std::string diagnostic = "inactive";
};

// Independent execution supervisor. A continuous disagreement between the
// accepted command and measured motion is a fault after the declared
// actuator response delay plus safety reaction interval. Once critical, the
// supervisor demands a mechanical safe stop; the coordinator only sequences
// that verdict after the existing Safety gate. It never rewrites planner or
// estimator policy.
class ExecutionMonitor {
 public:
  explicit ExecutionMonitor(ExecutionMonitorConfig config = {})
      : config_(config) {}

  void reset();
  void clear_fault();
  ExecutionMonitorReport update(const iface::MotionCommand& command,
                                const EstimatedState& estimate,
                                core::MonotonicNs now_ns);

 private:
  ExecutionMonitorConfig config_;
  bool armed_ = false;
  ExecutionFault fault_ = ExecutionFault::kNone;
  ExecutionFault pending_fault_ = ExecutionFault::kNone;
  core::MonotonicNs pending_since_ns_ = core::kInvalidTime;
  double pending_forward_mps_ = 0.0;
  double pending_lateral_mps_ = 0.0;
  double pending_yaw_rate_radps_ = 0.0;
};

}  // namespace lost_signal::robot::autonomy::diagnostics
