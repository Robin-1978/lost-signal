#pragma once

#include <cstdint>
#include <string>

#include "robot/core/frames.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/provider.hpp"

namespace lost_signal::robot::iface {

// Body-frame planar motion feedback: the actuator/joint truth reduced to the
// body twist the ROBOT controller closes its loops on. Per-joint command
// modes are future work; feedback stays body-level exactly like the command
// side, so neither side reinterprets the other's numbers.
struct MotionFeedback {
  std::uint64_t sequence = 0;
  core::MonotonicNs stamp_ns = core::kInvalidTime;  // acquisition time
  std::string source;                               // stable endpoint id
  core::Frame frame = core::Frame::kBaseLink;

  bool odometry_valid = false;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;

  bool valid() const {
    return !source.empty() && stamp_ns != core::kInvalidTime &&
           sequence > 0 && odometry_valid;
  }
};

// Motion endpoint: consumes body commands, produces body feedback.
// Same IProvider root as the sensors; only the typed poll differs, so there
// is exactly ONE provider story: claim a descriptor, then poll typed values.
class IMotionEndpoint : public IProvider {
 public:
  virtual void consume_command(const MotionCommand& cmd) = 0;
  virtual bool poll_feedback(MotionFeedback& out) = 0;
};

}  // namespace lost_signal::robot::iface
