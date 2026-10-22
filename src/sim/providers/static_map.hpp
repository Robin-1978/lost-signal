#pragma once

#include "robot/iface/map.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// StaticMapProvider: rasterizes the authored static geometry into one
// immutable MapSnapshot revision per geometry version. Actors never enter
// the map; the revision only moves when static geometry changes, so the
// coordinator replans on content change, not on every tick.
class StaticMapProvider : public robot::iface::IMapProvider {
 public:
  StaticMapProvider(world::World* world, double min_x_m, double min_y_m,
                    double width_m, double height_m, double resolution_m);

  robot::iface::MapSnapshot active_snapshot() const override;
  bool select_map(const std::string& map_id) override;

 private:
  void rebuild() const;

  world::World* world_ = nullptr;
  double min_x_m_, min_y_m_, width_m_, height_m_, resolution_m_;
  mutable robot::iface::MapSnapshot snapshot_;
  mutable std::uint64_t built_version_ = 0;
  mutable bool has_built_ = false;
};

}  // namespace lost_signal::sim::providers
