#pragma once

#include "robot/iface/map.hpp"
#include "sim/eval/map_package.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// StaticMapProvider: rasterizes the authored static geometry into one
// immutable MapSnapshot revision per geometry version. Actors never enter
// the map; the revision only moves when static geometry changes, so the
// coordinator replans on content change, not on every tick.
//
// Package mode serves a stored survey instead of live geometry (offline
// interchange, `sim/eval/map_package`): same snapshot seam, revision
// pinned at 1, map selection by package id.
class StaticMapProvider : public robot::iface::IMapProvider {
 public:
  StaticMapProvider(world::World* world, double min_x_m, double min_y_m,
                    double width_m, double height_m, double resolution_m);
  explicit StaticMapProvider(const eval::MapPackage& package);

 const robot::iface::MapSnapshot& active_snapshot() const override;
  bool select_map(const std::string& map_id) override;
  // SIM-side orchestration only (scenario fits the survey to authored
  // geometry after a --map load): re-windows the raster and retires the
  // cached revision so the coordinator replans on the new content.
  // Non-positive sizes are ignored (window kept, never half-applied).
  // Never called by ROBOT code; never during a live tick.
  void set_window(double min_x_m, double min_y_m, double width_m,
                  double height_m);

 private:
  void rebuild() const;

  world::World* world_ = nullptr;
  double min_x_m_ = 0.0, min_y_m_ = 0.0, width_m_ = 0.0, height_m_ = 0.0,
         resolution_m_ = 0.1;
  eval::MapPackage package_;
  bool from_package_ = false;
  // Window epoch: every set_window moves the revision even when the
  // geometry version stands still, so a re-windowed survey always reads
  // as new content downstream. Zero until the first call (legacy pins
  // see identical revisions).
  std::uint64_t window_epoch_ = 0;
  mutable robot::iface::MapSnapshot snapshot_;
  mutable std::uint64_t built_version_ = 0;
  mutable bool has_built_ = false;
};

}  // namespace lost_signal::sim::providers
