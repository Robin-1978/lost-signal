#pragma once

#include "robot/iface/motion.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// SimMotionEndpoint: the ONLY sim code that turns commands into motion and
// motion into feedback. v1 kinematic plant: the body tracks the delivered
// command; truth -> typed MotionFeedback, never truth itself.
class SimMotionEndpoint : public robot::iface::IMotionEndpoint {
 public:
  explicit SimMotionEndpoint(world::World* world);

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return world_ != nullptr; }
  void consume_command(const robot::iface::ActuatorCommand& cmd) override;
  bool poll_feedback(robot::iface::MotionFeedback& out) override;

 private:
  world::World* world_ = nullptr;
  robot::iface::EndpointDescriptor desc_;
  std::uint64_t seq_ = 0;
};

}  // namespace lost_signal::sim::providers
