#pragma once

#include <cstdint>

#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

// Sim2DLidar: synthetic planar scan raycast from WorldTruth (SIM-owned
// generation). ROBOT only sees the typed Lidar2DScan. Bearings are
// robot-frame; the cast adds the true body yaw so a turning base scans the
// world (a world-frame fan would silently rotate every reading with yaw).
namespace lost_signal::sim::world {
class World;
}  // namespace lost_signal::sim::world

namespace lost_signal::sim::providers {

class Sim2DLidarProvider
    : public robot::iface::ISensorProvider<robot::iface::Lidar2DScan> {
 public:
  Sim2DLidarProvider(world::World* world, const char* endpoint_id,
                     int beam_count, float max_range_m);

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return world_ != nullptr; }
  bool poll(robot::iface::Lidar2DScan& out) override {
    return poll(out, true);
  }
  // include_actors=false renders static geometry only: the reference the
  // foreground classifier diffs against. Same rays, same stamps.
  bool poll(robot::iface::Lidar2DScan& out, bool include_actors);

 private:
  world::World* world_ = nullptr;
  robot::iface::EndpointDescriptor desc_;
  int beam_count_ = 72;
  float max_range_m_ = 10.0f;
  std::uint64_t seq_ = 0;
};

}  // namespace lost_signal::sim::providers
