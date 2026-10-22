#pragma once

#include "robot/iface/command.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::iface {

// Platform provider contract. SIM, replay, and native drivers implement this.
// ROBOT never reads provider private state; only these typed values cross.
class IPlatformProvider {
 public:
  virtual ~IPlatformProvider() = default;
  virtual const char* endpoint_id() const = 0;
  virtual bool poll_observation(ObservationFrame& out) = 0;
  virtual void consume_command(const MotionCommand& cmd) = 0;
};

}  // namespace lost_signal::robot::iface
