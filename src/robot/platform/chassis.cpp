#include "robot/platform/chassis.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::platform {

DifferentialChassis::DifferentialChassis(double wheel_radius_m,
                                         double track_width_m,
                                         ChassisLimits limits,
                                         control::WheelControllerConfig wheel_config)
    : wheel_radius_m_(wheel_radius_m),
      track_width_m_(track_width_m),
      limits_(limits),
      wheels_(std::move(wheel_config)) {
  wheels_.reset();
}

iface::CommandSpace DifferentialChassis::command_space() const {
  return iface::CommandSpace::kYawRate;
}

iface::MotionCommand DifferentialChassis::clamp_command(
    const iface::MotionCommand& cmd, core::MonotonicNs now_ns) const {
  if (cmd.space != iface::CommandSpace::kYawRate) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
  iface::MotionCommand out = cmd;
  out.forward_mps =
      std::clamp(cmd.forward_mps, -limits_.max_forward_mps, limits_.max_forward_mps);
  out.yaw_rate_radps = std::clamp(cmd.yaw_rate_radps, -limits_.max_yaw_rate_radps,
                                  limits_.max_yaw_rate_radps);
  return out;
}

WheelSpeeds DifferentialChassis::inverse(double forward_mps,
                                         double yaw_rate_radps) const {
  // v = (vl + vr)/2 * r ; w = (vr - vl)/L * r
  WheelSpeeds w;
  w.left_radps =
      (forward_mps - yaw_rate_radps * track_width_m_ * 0.5) / wheel_radius_m_;
  w.right_radps =
      (forward_mps + yaw_rate_radps * track_width_m_ * 0.5) / wheel_radius_m_;
  return w;
}

void DifferentialChassis::forward(const WheelSpeeds& w, double& forward_mps,
                                  double& yaw_rate_radps) const {
  forward_mps = (w.left_radps + w.right_radps) * 0.5 * wheel_radius_m_;
  yaw_rate_radps =
      (w.right_radps - w.left_radps) * wheel_radius_m_ / track_width_m_;
}

iface::ActuatorCommand DifferentialChassis::to_actuator(
    const iface::MotionCommand& body, const iface::MotionFeedback& fb,
    core::MonotonicNs now_ns) const {
  // Stops and invalid feedback short-circuit to safe zero; the plant's own
  // lag shapes the ramp, and the stop latch gates re-release upstream.
  if (body.space != iface::CommandSpace::kYawRate || body.is_safe_stop() ||
      !fb.wheels_valid) {
    wheels_.reset();
    return iface::ActuatorCommand::SafeZero(now_ns);
  }
  control::WheelFeedback wfb{fb.left_radps, fb.right_radps, true};
  // Fixed decision step matches the coordinator cadence contract.
  constexpr double kDtS = 0.1;
  control::WheelCommand wc =
      wheels_.update(body.forward_mps, body.yaw_rate_radps, wfb, kDtS);
  iface::ActuatorCommand out;
  out.mode = iface::ActuatorMode::kNormalizedDuty;
  out.stamp_ns = now_ns;
  out.expiry_ns = now_ns + 200'000'000;
  out.left = wc.left_duty;
  out.right = wc.right_duty;
  if (!iface::valid_actuator_command(out, now_ns)) {
    wheels_.reset();
    return iface::ActuatorCommand::SafeZero(now_ns);
  }
  return out;
}

std::unique_ptr<IChassis> make_chassis(const RobotDescription& desc) {
  if (desc.chassis_kind == "differential") {
    control::WheelControllerConfig wc;
    wc.wheel_radius_m = 0.1;
    wc.track_width_m = 0.5;
    return std::make_unique<DifferentialChassis>(0.1, 0.5, desc.limits,
                                                std::move(wc));
  }
  return nullptr;  // unknown kind: fail closed, no silent default chassis
}

}  // namespace lost_signal::robot::platform
