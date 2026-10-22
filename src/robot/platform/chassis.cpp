#include "robot/platform/chassis.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::platform {

DifferentialChassis::DifferentialChassis(double wheel_radius_m,
                                         double track_width_m,
                                         ChassisLimits limits)
    : wheel_radius_m_(wheel_radius_m),
      track_width_m_(track_width_m),
      limits_(limits) {}

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

}  // namespace lost_signal::robot::platform
