#pragma once

#include "robot/iface/command.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::platform {

// IChassis: deployable chassis kinematics. Consumes validated body commands,
// produces wheel/steer targets. No physics, no MuJoCo, no rendering here.
class IChassis {
 public:
  virtual ~IChassis() = default;
  virtual const char* chassis_id() const = 0;
  virtual iface::CommandSpace command_space() const = 0;
  virtual ChassisLimits limits() const = 0;
  // Clamp a body command into chassis limits (rate/velocity envelope).
  virtual iface::MotionCommand clamp_command(const iface::MotionCommand& cmd,
                                            core::MonotonicNs now_ns) const = 0;
};

// Differential drive: (forward, yaw_rate) <-> (left, right) wheel speeds.
struct WheelSpeeds {
  double left_radps = 0.0;
  double right_radps = 0.0;
};

class DifferentialChassis : public IChassis {
 public:
  DifferentialChassis(double wheel_radius_m, double track_width_m,
                      ChassisLimits limits);

  const char* chassis_id() const override { return "differential"; }
  iface::CommandSpace command_space() const override;
  ChassisLimits limits() const override { return limits_; }

  iface::MotionCommand clamp_command(const iface::MotionCommand& cmd,
                                     core::MonotonicNs now_ns) const override;

  WheelSpeeds inverse(double forward_mps, double yaw_rate_radps) const;
  void forward(const WheelSpeeds& w, double& forward_mps,
               double& yaw_rate_radps) const;

  double wheel_radius_m() const { return wheel_radius_m_; }
  double track_width_m() const { return track_width_m_; }

 private:
  double wheel_radius_m_;
  double track_width_m_;
  ChassisLimits limits_;
};

}  // namespace lost_signal::robot::platform
