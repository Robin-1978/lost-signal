#pragma once

#include <cstdint>
#include <string>

#include "robot/core/frames.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/provider.hpp"

namespace lost_signal::robot::iface {

// Body-frame planar motion feedback: the actuator/joint truth reduced to
// the body twist the ROBOT controller closes its loops on, PLUS the raw
// wheel/joint channels the wheel controllers close their own loops on.
// (Body twist is what a driver derives from encoders; wheels are the raw
// measurement. Both cross; each consumer picks its level.)
struct MotionFeedback {
  std::uint64_t sequence = 0;
  core::MonotonicNs stamp_ns = core::kInvalidTime;  // acquisition time
  std::string source;                               // stable endpoint id
  core::Frame frame = core::Frame::kBaseLink;

  bool odometry_valid = false;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;

  // Per-wheel joint truth (mechanical rad/s, measured). Valid when the
  // provider exposes joint state; wheel controllers require it.
  bool wheels_valid = false;
  double left_radps = 0.0;
  double right_radps = 0.0;

  bool valid() const {
    return !source.empty() && stamp_ns != core::kInvalidTime &&
           sequence > 0 && (odometry_valid || wheels_valid);
  }
};

// Actuator command: per-channel setpoints in ONE explicit mode.
// v1 supports normalized duty only (passive SIM plant / native driver-flex).
// Velocity/position/effort modes arrive as new enum values with their own
// units and validation, never by reinterpreting these fields.
enum class ActuatorMode {
  kNormalizedDuty = 0,  // [-1, 1] per channel
};

struct ActuatorCommand {
  ActuatorMode mode = ActuatorMode::kNormalizedDuty;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  core::MonotonicNs expiry_ns = core::kInvalidTime;
  // Named channels; differential v1 uses exactly left/right wheel duty.
  double left = 0.0;
  double right = 0.0;

  static ActuatorCommand SafeZero(core::MonotonicNs now_ns) {
    ActuatorCommand c;
    c.stamp_ns = now_ns;
    c.expiry_ns = now_ns + 200'000'000;  // 200 ms watchdog, like motion
    return c;
  }

  bool is_safe_zero() const { return left == 0.0 && right == 0.0; }
};

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
