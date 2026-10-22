#include "robot/platform/chassis.hpp"

#include <algorithm>
#include <cmath>

#include "robot/platform/ackermann_kinematics.hpp"
#include "robot/platform/chassis_registry.hpp"
#include "robot/platform/holonomic_kinematics.hpp"

namespace lost_signal::robot::platform {

namespace {

// Twist-to-curvature floor: below this speed the yaw->steer conversion
// freezes at the low-speed geometry instead of dividing toward infinity.
constexpr double kSteerSpeedFloorMps = 0.5;

double steer_for_twist(double yaw_rate, double forward, double wheel_base_m,
                       double max_steer_rad) {
  const double raw = std::atan2(yaw_rate * wheel_base_m,
                                std::max(std::abs(forward), kSteerSpeedFloorMps));
  return std::clamp(raw, -max_steer_rad, max_steer_rad);
}

}  // namespace

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
  // No joints, no loop: fail closed. Never drive (or brake) blind.
  if (!fb.has_joints() || fb.drive_count < 2) {
    wheels_.reset();
    return iface::ActuatorCommand::SafeZero(now_ns);
  }
  // Safe stop with measured joints is a closed-loop hold at zero: active
  // electrical braking through the same wheel loops. The force plant has
  // no passive drag, so open-loop zero duty would coast forever; the stop
  // latch upstream still gates re-release. The loop stays warm to hold.
  double target_v = 0.0;
  double target_w = 0.0;
  if (!body.is_safe_stop()) {
    if (body.space != iface::CommandSpace::kYawRate) {
      wheels_.reset();
      return iface::ActuatorCommand::SafeZero(now_ns);
    }
    target_v = body.forward_mps;
    target_w = body.yaw_rate_radps;
  }
  control::WheelFeedback wfb{fb.drive_radps[0], fb.drive_radps[1], true};
  // Fixed decision step matches the coordinator cadence contract.
  constexpr double kDtS = 0.1;
  control::WheelCommand wc =
      wheels_.update(target_v, target_w, wfb, kDtS);
  iface::ActuatorCommand out;
  out.mode = iface::ActuatorMode::kDiffDuty;
  out.stamp_ns = now_ns;
  out.expiry_ns = now_ns + 200'000'000;
  out.drive[0] = wc.left_duty;
  out.drive[1] = wc.right_duty;
  if (!iface::valid_actuator_command(out, now_ns)) {
    wheels_.reset();
    return iface::ActuatorCommand::SafeZero(now_ns);
  }
  return out;
}

AckermannChassis::AckermannChassis(double wheel_radius_m, double wheel_base_m,
                                   double track_width_m, ChassisLimits limits,
                                   control::AckermannControllerConfig steer_config)
    : wheel_radius_m_(wheel_radius_m),
      wheel_base_m_(wheel_base_m),
      track_width_m_(track_width_m),
      limits_(limits),
      controller_(std::move(steer_config)) {
  controller_.reset();
}

iface::CommandSpace AckermannChassis::command_space() const {
  return iface::CommandSpace::kSteerAngle;
}

iface::MotionCommand AckermannChassis::clamp_command(
    const iface::MotionCommand& cmd, core::MonotonicNs now_ns) const {
  if (cmd.space != iface::CommandSpace::kSteerAngle &&
      cmd.space != iface::CommandSpace::kYawRate) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
  iface::MotionCommand out = cmd;
  out.forward_mps =
      std::clamp(cmd.forward_mps, -limits_.max_forward_mps, limits_.max_forward_mps);
  if (cmd.space == iface::CommandSpace::kSteerAngle) {
    out.yaw_rate_radps = 0.0;
    out.steer_angle_rad = std::clamp(cmd.steer_angle_rad, -limits_.max_steer_rad,
                                     limits_.max_steer_rad);
  } else {
    out.steer_angle_rad = 0.0;
    out.yaw_rate_radps = std::clamp(cmd.yaw_rate_radps, -limits_.max_yaw_rate_radps,
                                    limits_.max_yaw_rate_radps);
  }
  out.lateral_mps = 0.0;
  return out;
}

iface::ActuatorCommand AckermannChassis::to_actuator(
    const iface::MotionCommand& body, const iface::MotionFeedback& fb,
    core::MonotonicNs now_ns) const {
  // No joints, no loop: fail closed. Never drive (or brake) blind.
  if (!fb.has_joints() || fb.drive_count < 2) {
    controller_.reset();
    return iface::ActuatorCommand::SafeZero(
        now_ns, iface::ActuatorMode::kAckermannDuty);
  }
  // Safe stop with measured joints: closed-loop hold at zero (brake the
  // drives, center the steers). Same force-plant rationale as diff.
  double steer = 0.0;
  double target_v = 0.0;
  if (!body.is_safe_stop()) {
    if (body.space == iface::CommandSpace::kSteerAngle) {
      target_v = body.forward_mps;
      steer = body.steer_angle_rad;
    } else if (body.space == iface::CommandSpace::kYawRate) {
      target_v = body.forward_mps;
      steer = steer_for_twist(body.yaw_rate_radps, body.forward_mps,
                              wheel_base_m_, limits_.max_steer_rad);
    } else {
      controller_.reset();
      return iface::ActuatorCommand::SafeZero(
          now_ns, iface::ActuatorMode::kAckermannDuty);
    }
  }
  control::AckermannFeedback afb;
  afb.rear_left_radps = fb.drive_radps[0];
  afb.rear_right_radps = fb.drive_radps[1];
  if (fb.steer_count >= 2) {
    afb.front_left_steer_rad = fb.steer_rad[0];
    afb.front_right_steer_rad = fb.steer_rad[1];
    afb.front_left_steer_vel_radps = fb.steer_vel_radps[0];
    afb.front_right_steer_vel_radps = fb.steer_vel_radps[1];
  }
  afb.valid = true;
  constexpr double kDtS = 0.1;
  control::AckermannCommand ac =
      controller_.update(target_v, steer, afb, kDtS);
  iface::ActuatorCommand out;
  out.mode = iface::ActuatorMode::kAckermannDuty;
  out.stamp_ns = now_ns;
  out.expiry_ns = now_ns + 200'000'000;
  out.drive[0] = ac.rear_left_duty;
  out.drive[1] = ac.rear_right_duty;
  out.steer[0] = ac.front_left_steer_duty;
  out.steer[1] = ac.front_right_steer_duty;
  if (!iface::valid_actuator_command(out, now_ns)) {
    controller_.reset();
    return iface::ActuatorCommand::SafeZero(
        now_ns, iface::ActuatorMode::kAckermannDuty);
  }
  return out;
}

HolonomicChassis::HolonomicChassis(double wheel_radius_m, double wheel_base_m,
                                   double track_width_m, ChassisLimits limits,
                                   control::HolonomicControllerConfig holonomic_config)
    : wheel_radius_m_(wheel_radius_m),
      wheel_base_m_(wheel_base_m),
      track_width_m_(track_width_m),
      limits_(limits),
      config_(std::move(holonomic_config)),
      controller_(config_) {
  controller_.reset();
}

iface::CommandSpace HolonomicChassis::command_space() const {
  return iface::CommandSpace::kHolonomic;
}

iface::MotionCommand HolonomicChassis::clamp_command(
    const iface::MotionCommand& cmd, core::MonotonicNs now_ns) const {
  if (cmd.space != iface::CommandSpace::kHolonomic &&
      cmd.space != iface::CommandSpace::kYawRate) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
  iface::MotionCommand out = cmd;
  out.forward_mps =
      std::clamp(cmd.forward_mps, -limits_.max_forward_mps, limits_.max_forward_mps);
  out.yaw_rate_radps = std::clamp(cmd.yaw_rate_radps, -limits_.max_yaw_rate_radps,
                                  limits_.max_yaw_rate_radps);
  if (cmd.space == iface::CommandSpace::kHolonomic) {
    out.lateral_mps = std::clamp(cmd.lateral_mps, -limits_.max_lateral_mps,
                                 limits_.max_lateral_mps);
  } else {
    out.lateral_mps = 0.0;
  }
  out.steer_angle_rad = 0.0;
  return out;
}

iface::ActuatorCommand HolonomicChassis::to_actuator(
    const iface::MotionCommand& body, const iface::MotionFeedback& fb,
    core::MonotonicNs now_ns) const {
  const bool swerve = config_.kind == control::HolonomicKind::Swerve;
  const iface::ActuatorMode mode =
      swerve ? iface::ActuatorMode::kSwerveDuty : iface::ActuatorMode::kMecanumDuty;
  const std::uint8_t need_steer = swerve ? 4 : 0;
  // No joints, no loop: fail closed. Never drive (or brake) blind.
  if (!fb.has_joints() || fb.drive_count < 4 || fb.steer_count < need_steer) {
    controller_.reset();
    return iface::ActuatorCommand::SafeZero(now_ns, mode);
  }
  // Safe stop with measured joints: closed-loop hold at zero. Same
  // force-plant rationale as diff (swerve modules hold their angles from
  // the zero-twist inverse below).
  double lateral = 0.0;
  double target_v = 0.0;
  double target_w = 0.0;
  if (!body.is_safe_stop()) {
    if (body.space == iface::CommandSpace::kHolonomic) {
      lateral = body.lateral_mps;
      target_v = body.forward_mps;
      target_w = body.yaw_rate_radps;
    } else if (body.space == iface::CommandSpace::kYawRate) {
      target_v = body.forward_mps;
      target_w = body.yaw_rate_radps;
    } else {
      controller_.reset();
      return iface::ActuatorCommand::SafeZero(now_ns, mode);
    }
  }
  control::HolonomicFeedback hfb;
  hfb.drive_radps = {fb.drive_radps[0], fb.drive_radps[1], fb.drive_radps[2],
                     fb.drive_radps[3]};
  hfb.steer_rad = {fb.steer_rad[0], fb.steer_rad[1], fb.steer_rad[2],
                   fb.steer_rad[3]};
  hfb.steer_vel_radps = {fb.steer_vel_radps[0], fb.steer_vel_radps[1],
                         fb.steer_vel_radps[2], fb.steer_vel_radps[3]};
  hfb.valid = true;
  constexpr double kDtS = 0.1;
  control::HolonomicCommand hc = controller_.update(
      target_v, lateral, target_w, hfb, kDtS);
  iface::ActuatorCommand out;
  out.mode = mode;
  out.stamp_ns = now_ns;
  out.expiry_ns = now_ns + 200'000'000;
  out.drive = hc.drive_duty;
  out.steer = hc.steer_duty;
  if (!iface::valid_actuator_command(out, now_ns)) {
    controller_.reset();
    return iface::ActuatorCommand::SafeZero(now_ns, mode);
  }
  return out;
}

std::unique_ptr<IChassis> make_chassis(const RobotDescription& desc) {
  return ChassisRegistry::global().create(desc);
}

namespace {

// Built-in chassis plugins. Static registration keeps make_chassis a pure
// registry lookup; new bases register from their own translation unit
// without touching this file. These live here (not a separate TU) so the
// linker cannot drop them: chassis.cpp is always pulled for the chassis
// methods the application references.
const RegisterChassis kRegisterDifferential{
    "differential", [](const RobotDescription& desc) {
      control::WheelControllerConfig wc;
      wc.wheel_radius_m = 0.1;
      wc.track_width_m = 0.5;
      return std::make_unique<DifferentialChassis>(0.1, 0.5, desc.limits,
                                                  std::move(wc));
    }};

const RegisterChassis kRegisterAckermann{
    "ackermann", [](const RobotDescription& desc) {
      control::AckermannControllerConfig ac;
      ac.wheel_base_m = 0.62;
      ac.track_width_m = 0.58;
      return std::make_unique<AckermannChassis>(0.1, ac.wheel_base_m,
                                               ac.track_width_m, desc.limits,
                                               std::move(ac));
    }};

const RegisterChassis kRegisterSwerve{
    "swerve", [](const RobotDescription& desc) {
      control::HolonomicControllerConfig hc;
      hc.kind = control::HolonomicKind::Swerve;
      return std::make_unique<HolonomicChassis>(0.1, hc.wheel_base_m,
                                               hc.track_width_m, desc.limits,
                                               std::move(hc));
    }};

const RegisterChassis kRegisterMecanum{
    "mecanum", [](const RobotDescription& desc) {
      control::HolonomicControllerConfig hc;
      hc.kind = control::HolonomicKind::Mecanum;
      return std::make_unique<HolonomicChassis>(0.1, hc.wheel_base_m,
                                               hc.track_width_m, desc.limits,
                                               std::move(hc));
    }};

}  // namespace

}  // namespace lost_signal::robot::platform
