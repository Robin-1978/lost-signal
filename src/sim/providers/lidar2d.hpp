#pragma once

#include <cstdint>

#include "robot/core/transform.hpp"
#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

// Sim2DLidar: true 3D raycast from the sensor pose through WorldTruth
// (SIM-owned generation). ROBOT only sees the typed Lidar2DScan. The
// sensor pose composes the live body position + attitude (engine
// roll/pitch/yaw/height) with the mount extrinsics, so a tilted body or
// an offset mount scans the world it actually sees. Static scene resolves
// through the engine (mj_ray vs floor + boxes, robot geoms excluded);
// actors have no engine bodies and are supplemented by an analytic upright
// cylinder (same split as main's game lidar). Bearings stay robot-frame:
// the cast adds the sensor yaw so a turning base scans the world (a
// world-frame fan would silently rotate every reading with yaw).
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
  // Sensor mount extrinsics in the base frame (xyz meters, RPY rad,
  // REP-103). Non-finite input is rejected, previous mount kept. Default
  // zeros = sensor at the base origin (legacy behavior).
  void set_mount(const double xyz_m[3], const double rpy_rad[3]);
  // Pre-run geometry tuning (SIM harness only): positive beam count and
  // range, otherwise rejected with previous kept.
  void set_beams(int beam_count);
  void set_max_range(float max_range_m);

 private:
  world::World* world_ = nullptr;
  robot::iface::EndpointDescriptor desc_;
  int beam_count_ = 72;
  float max_range_m_ = 10.0f;
  std::uint64_t seq_ = 0;
  double mount_xyz_m_[3] = {0.0, 0.0, 0.0};
  double mount_rpy_rad_[3] = {0.0, 0.0, 0.0};
};

}  // namespace lost_signal::sim::providers
