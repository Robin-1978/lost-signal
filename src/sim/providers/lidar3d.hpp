#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "robot/core/transform.hpp"
#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

// Sim3DLidar: true 3D multi-line raycast from the sensor pose through
// WorldTruth (SIM-owned generation). ROBOT only sees the typed
// Lidar3DCloud. Same scene split as the 2D producer: static geometry via
// the engine (mj_ray vs floor + boxes, robot geoms excluded), actors via
// the analytic upright cylinder. Points are sensor-frame (x forward,
// y left, z up), azimuth-major (all rings per azimuth step, like a
// rotating scanner); misses are dropped (real-driver behavior, never
// max-range fill); intensity follows the shared return-strength model
// (material reflectivity + distance attenuation, SIM-side only).
namespace lost_signal::sim::world {
class World;
}  // namespace lost_signal::sim::world

namespace lost_signal::sim::providers {

class Sim3DLidarProvider
    : public robot::iface::ISensorProvider<robot::iface::Lidar3DCloud> {
 public:
  // VLP-16-like defaults: 16 rings over [-15, +15] deg, 72 azimuths.
  Sim3DLidarProvider(world::World* world, const char* endpoint_id,
                     int ring_count = 16, int azimuth_beams = 72,
                     float max_range_m = 10.0f);

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return world_ != nullptr; }
  bool poll(robot::iface::Lidar3DCloud& out) override {
    return poll(out, true);
  }
  // include_actors=false renders static geometry only. Same rays, same
  // stamps (mirrors the 2D reference pass).
  bool poll(robot::iface::Lidar3DCloud& out, bool include_actors);
  // Composite 2D scan from the multi-line cloud (suite-side front end):
  // per-azimuth minimum over non-ground rings at the 2D layout
  // (angle_min -pi, even spacing, `azimuth_beams` bins). Ground = world
  // hit height below `kGroundCutM` (floor/paint excluded, plates kept).
  // Misses (all rings ground or out of range) read max range. Same
  // generation as poll(), same stamps. The intensities overload reports
  // the winning beam's return strength per azimuth (0 on misses).
  bool poll_composite(std::vector<float>& ranges_m, bool include_actors);
  bool poll_composite(std::vector<float>& ranges_m,
                      std::vector<float>& intensities_m, bool include_actors);
  // Sensor mount extrinsics in the base frame (xyz meters, RPY rad,
  // REP-103). Non-finite input is rejected, previous mount kept.
  void set_mount(const double xyz_m[3], const double rpy_rad[3]);
  // Pre-run geometry tuning (SIM harness only): positive counts/ranges,
  // finite FOV with min < max; otherwise rejected with previous kept.
  void set_rings(int ring_count);
  void set_azimuths(int azimuth_beams);
  // Vertical FOV for the ring fan (uniform spacing; single ring = 0).
  void set_vertical_fov(double min_elev_rad, double max_elev_rad);
  void set_max_range(float max_range_m);
  void set_scan_period(float scan_period_s);
  // World height below which a return counts as ground (floor/paint).
  // Must be finite in (0, 1); rejected otherwise with previous kept.
  void set_ground_cut(float ground_cut_m) {
    if (std::isfinite(ground_cut_m) && ground_cut_m > 0.0f &&
        ground_cut_m < 1.0f) {
      ground_cut_m_ = ground_cut_m;
    }
  }

 private:
  world::World* world_ = nullptr;
  robot::iface::EndpointDescriptor desc_;
  int ring_count_ = 16;
  int azimuth_beams_ = 72;
  double min_elev_rad_ = -0.2617993877991494;  // -15 deg
  double max_elev_rad_ = 0.2617993877991494;   // +15 deg
  float max_range_m_ = 10.0f;
  float scan_period_s_ = 0.1f;
  float ground_cut_m_ = 0.05f;
  std::uint64_t seq_ = 0;
  double mount_xyz_m_[3] = {0.0, 0.0, 0.0};
  double mount_rpy_rad_[3] = {0.0, 0.0, 0.0};
};

}  // namespace lost_signal::sim::providers
