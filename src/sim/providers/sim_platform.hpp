#pragma once

#include "robot/iface/platform.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// Simulated platform provider: converts WorldTruth into typed LSRI
// observations and applies validated commands to the world. The only SIM
// code allowed to know joint/world internals; ROBOT sees only LSRI.
class SimPlatformProvider : public robot::iface::IPlatformProvider {
 public:
  explicit SimPlatformProvider(world::World* world);

  const char* endpoint_id() const override;
  bool poll_observation(robot::iface::ObservationFrame& out) override;
  void consume_command(const robot::iface::MotionCommand& cmd) override;

 private:
  world::World* world_ = nullptr;
  std::uint64_t seq_ = 0;
};

}  // namespace lost_signal::sim::providers
