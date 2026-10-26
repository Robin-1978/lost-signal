#pragma once

#include <memory>

#include "robot/control/ackermann_controller.hpp"
#include "robot/control/holonomic_controller.hpp"
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
 virtual iface::ActuatorMode actuator_mode() const = 0;
  virtual iface::CommandSpace command_space() const = 0;
  virtual ChassisLimits limits() const = 0;
  // Clamp a body command into chassis limits (rate/velocity envelope).
  virtual iface::MotionCommand clamp_command(const iface::MotionCommand& cmd,
                                            core::MonotonicNs now_ns) const = 0;
  // Translate a (clamped, validated) body command into actuator setpoints
 // using the latest joint feedback. Without valid feedback the chassis
 // returns a mechanical safe-stop command: loops never close on stale joints.
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
 iface::ActuatorMode actuator_mode() const override {
 return iface::ActuatorMode::kDiffDuty;
 }
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
  // Last to_actuator stamp for the measured controller step (nominal plant
  // rate on first call / repeated stamps; clamped to the command watchdog).
  mutable core::MonotonicNs last_update_ns_ = core::kInvalidTime;
};

// Ackermann: native space is (forward, steer-angle); a kYawRate body
// command converts through the bicycle model with a low-speed floor
// (a car cannot turn standing still). Rear drives close speed loops,
// front steers close a position->rate cascade.
class AckermannChassis : public IChassis {
 public:
  AckermannChassis(double wheel_radius_m, double wheel_base_m,
                   double track_width_m, ChassisLimits limits,
                   control::AckermannControllerConfig steer_config = {});

  const char* chassis_id() const override { return "ackermann"; }
 iface::ActuatorMode actuator_mode() const override {
 return iface::ActuatorMode::kAckermannDuty;
 }
  iface::CommandSpace command_space() const override;
  ChassisLimits limits() const override { return limits_; }

 double wheel_radius_m() const { return wheel_radius_m_; }
 double wheel_base_m() const { return wheel_base_m_; }
 double track_width_m() const { return track_width_m_; }

  iface::MotionCommand clamp_command(const iface::MotionCommand& cmd,
                                     core::MonotonicNs now_ns) const override;
  iface::ActuatorCommand to_actuator(
      const iface::MotionCommand& body, const iface::MotionFeedback& fb,
      core::MonotonicNs now_ns) const override;

 private:
  double wheel_radius_m_;
  double wheel_base_m_;
  double track_width_m_;
  ChassisLimits limits_;
  mutable control::AckermannController controller_;
  mutable core::MonotonicNs last_update_ns_ = core::kInvalidTime;
};

// Holonomic: native space is (forward, lateral, yaw). Swerve modules
// resolve speed + steer per corner; mecanum resolves 4 wheel speeds.
// A kYawRate body command runs with lateral = 0.
class HolonomicChassis : public IChassis {
 public:
  HolonomicChassis(double wheel_radius_m, double wheel_base_m,
                   double track_width_m, ChassisLimits limits,
                   control::HolonomicControllerConfig holonomic_config = {});

  const char* chassis_id() const override {
    return config_.kind == control::HolonomicKind::Swerve ? "swerve"
                                                         : "mecanum";
  }
 iface::ActuatorMode actuator_mode() const override {
 return config_.kind == control::HolonomicKind::Swerve
 ? iface::ActuatorMode::kSwerveDuty
 : iface::ActuatorMode::kMecanumDuty;
 }
  iface::CommandSpace command_space() const override;
  ChassisLimits limits() const override { return limits_; }

 double wheel_radius_m() const { return wheel_radius_m_; }
 double wheel_base_m() const { return wheel_base_m_; }
 double track_width_m() const { return track_width_m_; }

  iface::MotionCommand clamp_command(const iface::MotionCommand& cmd,
                                     core::MonotonicNs now_ns) const override;
  iface::ActuatorCommand to_actuator(
      const iface::MotionCommand& body, const iface::MotionFeedback& fb,
      core::MonotonicNs now_ns) const override;

 private:
  double wheel_radius_m_;
  double wheel_base_m_;
  double track_width_m_;
  ChassisLimits limits_;
  control::HolonomicControllerConfig config_;
  mutable control::HolonomicController controller_;
  mutable core::MonotonicNs last_update_ns_ = core::kInvalidTime;
};

// Build the chassis named by a RobotDescription ("differential",
// "ackermann", "swerve", "mecanum"). Unknown kinds fail closed to
// nullptr: no silent default chassis.
std::unique_ptr<IChassis> make_chassis(const RobotDescription& desc);

}  // namespace lost_signal::robot::platform
