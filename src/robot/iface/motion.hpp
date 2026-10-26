#pragma once

#include <array>
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

  bool has_joints() const { return drive_count > 0; }

  bool valid() const {
    return !source.empty() && stamp_ns != core::kInvalidTime &&
           sequence > 0 && (odometry_valid || drive_count > 0);
  }
};

// Actuator command: per-channel normalized duties in ONE explicit mode.
// All channels share the unit [-1, 1]: drives are motor duties, steers are
// servo-rate duties (plant maps them through max rate/angle). New modes
// arrive as new enum values with their own channel counts, never by
// reinterpreting these arrays.
//
// Channel counts per mode: Diff 2 drives; Ackermann 2 drives + 2 steers;
// Swerve 4 + 4; Mecanum 4 drives.
enum class ActuatorMode {
  kDiffDuty = 0,
  kAckermannDuty = 1,
  kSwerveDuty = 2,
  kMecanumDuty = 3,
};

struct ActuatorCommand {
  ActuatorMode mode = ActuatorMode::kDiffDuty;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  core::MonotonicNs expiry_ns = core::kInvalidTime;
  std::array<double, 4> drive = {};
  std::array<double, 4> steer = {};

  static ActuatorCommand SafeZero(core::MonotonicNs now_ns,
                                  ActuatorMode mode = ActuatorMode::kDiffDuty) {
    ActuatorCommand c;
    c.mode = mode;
    c.stamp_ns = now_ns;
    c.expiry_ns = now_ns + 200'000'000;  // 200 ms watchdog, like motion
    return c;
  }

  bool is_safe_zero() const {
    for (double d : drive) {
      if (d != 0.0) return false;
    }
    for (double s : steer) {
      if (s != 0.0) return false;
    }
    return true;
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
