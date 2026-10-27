#pragma once

#include "robot/iface/motion.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// SimMotionEndpoint: the ONLY sim code that turns commands into motion and
// motion into feedback. Force plant: duties integrate to joint speeds in the
// World; the endpoint reports the ENCODER view — angles quantized to counts
// per revolution, velocities differentiated from quantized angles over the
// poll stamp delta (no wall clock, no hidden rate). First sample has no
// history and reports zero velocity (a just-powered encoder); repeated
// stamps hold the previous velocity. Truth angles stay continuous; only the
// measurement is quantized, and the ROBOT loop closes on it like native.
// No mount extrinsics apply here by construction: joints are joint-space
// (encoders sit on the shafts they measure) and the odometry twist is the
// base-frame twist by definition (frame_id base_link).
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
  bool have_prev_ = false;
  robot::core::MonotonicNs prev_stamp_ns_ = robot::core::kInvalidTime;
  std::array<double, 4> prev_drive_angle_rad_ = {};
  std::array<double, 4> prev_steer_angle_rad_ = {};
  std::array<double, 4> prev_drive_vel_radps_ = {};
  std::array<double, 4> prev_steer_vel_radps_ = {};
 std::array<double, 4> prev_arm_angle_rad_ = {};
 std::array<double, 4> prev_arm_vel_radps_ = {};
};

}  // namespace lost_signal::sim::providers
