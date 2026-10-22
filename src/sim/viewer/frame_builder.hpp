#pragma once

// Builds draw-model Frames from SIM-side state. Pure functions over World
// truth + the recorded episode step (which already carries provider scan
// output and the truth-side judge). No raylib, no window, no ROBOT input:
// the frame is presentation of recorded state, same as an offline replay.

#include <string>
#include <vector>

#include "sim/eval/episode.hpp"
#include "sim/viewer/frame.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::viewer {

// Selection state owned by the map editor; the frame only mirrors it so
// the frontend can draw handles. Box index into world.static_boxes(),
// place index into world.places(); -1 means none.
struct EditorSelection {
  int box = -1;
  int box_corner = -1;
  int place = -1;
};

// reached_ids: place ids the objective driver reports done (frontend reads
// session.objectives().results()). trail: past truth positions, oldest
// first (frontend-owned history, capped by the caller).
Frame build_frame(const world::World& world, const eval::EvalStep& step,
                  const std::vector<std::string>& reached_ids,
                  const std::vector<std::pair<double, double>>& trail,
                  const EditorSelection& sel = EditorSelection{});

// World-space lidar endpoints from truth body pose + recorded scan.
// Sensor-offset ignored (documented viz approximation); invalid scan or
// non-finite ranges yield no hits.
std::vector<std::pair<double, double>> lidar_endpoints(
    double body_x_m, double body_y_m, double body_yaw_rad,
    const robot::iface::ObservationFrame& obs);

}  // namespace lost_signal::sim::viewer
