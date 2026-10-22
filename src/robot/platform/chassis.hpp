#pragma once

#include <memory>

#include "robot/control/wheel_controller.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/motion.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::platform {

// IChassis: deployable chassis. Consumes validated body commands, produces
// actuator setpoints against measured joint feedback. No physics, no MuJoCo,
// no rendering here.
class IChassis {
 public:
  virtual ~IChassis() = default;
  virtual const char* chassis_id() const = 0;
  virtual iface::CommandSpace command_space() const = 0;
  virtual ChassisLimits limits() const = 0;
  // Clamp a body command into chassis limits (rate/velocity envelope).
  virtual iface::MotionCommand clamp_command(const iface::MotionCommand& cmd,
                                            core::MonotonicNs now_ns) const = 0;
  // Translate a (clamped, validated) body command into actuator setpoints
  // using the latest joint feedback. Without valid feedback the chassis
  // returns a safe-zero command: loops never close on stale joints.
  virtual iface::ActuatorCommand to_actuator(
      const iface::MotionCommand& body, const iface::MotionFeedback& fb,
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
                      ChassisLimits limits,
                      control::WheelControllerConfig wheel_config = {});

  const char* chassis_id() const override { return "differential"; }
  iface::CommandSpace command_space() const override;
  ChassisLimits limits() const override { return limits_; }

  iface::MotionCommand clamp_command(const iface::MotionCommand& cmd,
                                     core::MonotonicNs now_ns) const override;
  iface::ActuatorCommand to_actuator(
      const iface::MotionCommand& body, const iface::MotionFeedback& fb,
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
  // Mutable: closed-loop state advances on every to_actuator call, which is
  // logically const (same command + same feedback = same setpoints).
  mutable control::DifferentialWheelController wheels_;
};

// Build the chassis named by a RobotDescription ("differential" v1;
// "ackermann" and "holonomic" follow the same factory when their providers
// land). Unknown kinds fail closed to nullptr: no silent default chassis.
std::unique_ptr<IChassis> make_chassis(const RobotDescription& desc);

}  // namespace lost_signal::robot::platform
