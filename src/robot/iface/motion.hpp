#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "robot/core/frames.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/provider.hpp"

namespace lost_signal::robot::iface {

// Body-frame planar motion feedback: the actuator/joint truth reduced to
// the body twist the ROBOT controller closes its loops on, PLUS the raw
// joint channels the wheel controllers close their own loops on.
// (Body twist is what a driver derives from encoders; joints are the raw
// measurement. Both cross; each consumer picks its level.)
//
// Joint layout is fixed-size, mode-implied: differential uses drives[0..1],
// ackermann uses drives[0..1] + steers[0..1], swerve uses drives[0..3] +
// steers[0..3], mecanum uses drives[0..3]. drive_count == 0 means absent.
struct MotionFeedback {
  std::uint64_t sequence = 0;
  core::MonotonicNs stamp_ns = core::kInvalidTime;  // acquisition time
  std::string source;                               // stable endpoint id
  core::Frame frame = core::Frame::kBaseLink;

  bool odometry_valid = false;
  double forward_mps = 0.0;
 double lateral_mps = 0.0;
  double yaw_rate_radps = 0.0;

  std::uint8_t drive_count = 0;  // driven joints present (0, 2, or 4)
  std::uint8_t steer_count = 0;  // steered joints present (0, 2, or 4)
  std::array<double, 4> drive_radps = {};  // mechanical rad/s, measured
  std::array<double, 4> steer_rad = {};    // measured steer angle, rad
  std::array<double, 4> steer_vel_radps = {};  // measured steer rate

 // Optional articulated arm joint feedback. This is a hardware encoder
 // section, separate from chassis drive/steer counts; zero means no arm.
 std::uint8_t arm_count = 0;
 std::array<double, 4> arm_rad = {};
 std::array<double, 4> arm_vel_radps = {};

 bool has_joints() const { return drive_count > 0; }
 bool has_arm() const { return arm_count > 0; }

  bool valid() const {
 if (arm_count > 4) return false;
 for (std::uint8_t i = 0; i < arm_count; ++i) {
 if (!std::isfinite(arm_rad[i]) || !std::isfinite(arm_vel_radps[i])) {
 return false;
 }
 }
    return !source.empty() && stamp_ns != core::kInvalidTime &&
 sequence > 0 && (odometry_valid || drive_count > 0 || arm_count > 0);
  }
};

// Actuator command: per-channel normalized requests in ONE explicit mode.
// Drives and steers use [-1, 1]; the passive service brake uses [0, 1].
// Drives are motor duties and steers are servo-rate duties (the plant maps
// them through max rate/angle). New modes arrive as new enum values with
// their own channel counts, never by reinterpreting these arrays.
//
// Channel counts per mode: Diff 2 drives; Ackermann 2 drives + 2 steers;
// Swerve 4 + 4; Mecanum 4 drives.
enum class ActuatorMode {
  kDiffDuty = 0,
  kAckermannDuty = 1,
  kSwerveDuty = 2,
  kMecanumDuty = 3,
};

// Control ownership is a hardware boundary, not a provider implementation
// detail. Exactly one layer owns the outer loop for a motion quantity.
enum class MotionControlOwner {
 kHostMotionLoop = 0,
 kMcuJointLoop = 1,
 kMcuBodyLoop = 2,
};

struct MotionCapabilities {
 MotionControlOwner control_owner = MotionControlOwner::kHostMotionLoop;
 ActuatorMode chassis_mode = ActuatorMode::kDiffDuty;
 std::uint8_t drive_count = 0;
 std::uint8_t steer_count = 0;
 core::MonotonicNs control_period_ns = core::kInvalidTime;
 core::MonotonicNs feedback_period_ns = core::kInvalidTime;
 core::MonotonicNs maximum_command_age_ns = core::kInvalidTime;
 std::string profile_id;
 std::uint32_t profile_revision = 0;
};

// A setpoint's kind fixes the units of every populated field. A transport
// must never infer units from an array position or from the board model.
enum class MotionSetpointKind {
 kActuatorDuty = 0,
 kJointVelocity = 1,
 kBodyTwist = 2,
 kStop = 3,
};

struct MotionSetpoint {
 MotionSetpointKind kind = MotionSetpointKind::kStop;
 ActuatorMode chassis_mode = ActuatorMode::kDiffDuty;
 std::uint64_t sequence = 0;
 core::MonotonicNs stamp_ns = core::kInvalidTime;
 core::MonotonicNs expiry_ns = core::kInvalidTime;
 core::Frame frame = core::Frame::kBaseLink;
 std::array<double, 4> drive_duty = {};
 std::array<double, 4> steer_duty = {};
 std::array<double, 4> drive_radps = {};
 std::array<double, 4> steer_rad = {};
 double forward_mps = 0.0;
 double lateral_mps = 0.0;
 double yaw_rate_radps = 0.0;
 double service_brake = 0.0;
};

struct ActuatorCommand {
  ActuatorMode mode = ActuatorMode::kDiffDuty;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  core::MonotonicNs expiry_ns = core::kInvalidTime;
  std::array<double, 4> drive = {};
 std::array<double, 4> steer = {};
 // Normalized joint effort request for an articulated arm. The SIM plant
 // maps this through the configured per-joint torque limit; native drivers
 // use the same [-1, 1] contract. Zero arm_count preserves legacy commands.
 std::uint8_t arm_count = 0;
 std::array<double, 4> arm = {};
 // Normalized mechanical service-brake request shared by the driven
 // wheels. It is an explicit actuator channel, independent of encoder
 // feedback and electric motor duty. Native drivers map this value to
 // their brake actuator; SIM applies the same lagged passive brake plant.
 double service_brake = 0.0;

  static ActuatorCommand SafeZero(core::MonotonicNs now_ns,
                                  ActuatorMode mode = ActuatorMode::kDiffDuty) {
    ActuatorCommand c;
    c.mode = mode;
    c.stamp_ns = now_ns;
    c.expiry_ns = now_ns + 200'000'000;  // 200 ms watchdog, like motion
    return c;
  }

 static ActuatorCommand SafeStop(core::MonotonicNs now_ns,
 ActuatorMode mode = ActuatorMode::kDiffDuty) {
 ActuatorCommand c = SafeZero(now_ns, mode);
 c.service_brake = 1.0;
 return c;
 }

  bool is_safe_zero() const {
    for (double d : drive) {
      if (d != 0.0) return false;
    }
 for (double s : steer) {
 if (s != 0.0) return false;
 }
 for (double a : arm) {
 if (a != 0.0) return false;
 }
 return service_brake == 0.0;
 }

 bool is_safe_stop() const {
 for (double d : drive) {
 if (d != 0.0) return false;
 }
 for (double s : steer) {
 if (s != 0.0) return false;
 }
 for (double a : arm) {
 if (a != 0.0) return false;
 }
 return service_brake == 1.0;
  }
};

inline std::uint8_t actuator_drive_count(ActuatorMode mode) {
  switch (mode) {
    case ActuatorMode::kDiffDuty:
    case ActuatorMode::kAckermannDuty:
      return 2;
    case ActuatorMode::kSwerveDuty:
    case ActuatorMode::kMecanumDuty:
      return 4;
  }
  return 0;
}

inline std::uint8_t actuator_steer_count(ActuatorMode mode) {
  switch (mode) {
    case ActuatorMode::kAckermannDuty:
      return 2;
    case ActuatorMode::kSwerveDuty:
      return 4;
    default:
      return 0;
  }
}

bool valid_actuator_command(const ActuatorCommand& cmd,
                            core::MonotonicNs now_ns);

bool valid_motion_capabilities(const MotionCapabilities& capabilities);

bool valid_motion_setpoint(const MotionSetpoint& setpoint,
  const MotionCapabilities& capabilities, core::MonotonicNs now_ns);

// Motion endpoint: consumes actuator commands, produces body + joint
// feedback. Same IProvider root as the sensors; only the typed poll differs,
// so there is exactly ONE provider story: claim a descriptor, then poll
// typed values. (Body MotionCommand is autonomy output; the chassis layer
// translates it to these actuator setpoints before they reach the plant.)
class IMotionEndpoint : public IProvider {
 public:
  virtual void consume_command(const ActuatorCommand& cmd) = 0;
  virtual bool poll_feedback(MotionFeedback& out) = 0;
};

}  // namespace lost_signal::robot::iface
