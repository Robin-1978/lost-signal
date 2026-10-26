#pragma once

// Builds draw-model Frames from SIM-side state. Pure functions over World
// truth + the recorded episode step (which already carries provider scan
// output and the truth-side judge). No raylib, no window, no ROBOT input:
// the frame is presentation of recorded state, same as an offline replay.

#include <optional>
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

// Lidar mount for ray-origin projection (base frame, xyz meters + RPY
// rad, same extrinsics generation runs). Identity = sensor at the base
// origin (legacy approximation).
struct LidarMount {
  double xyz_m[3] = {0.0, 0.0, 0.0};
  double rpy_rad[3] = {0.0, 0.0, 0.0};
};

// reached_ids: place ids the objective driver reports done (frontend reads
// session.objectives().results()). trail: past truth positions, oldest
// first (frontend-owned history, capped by the caller).
Frame build_frame(const world::World& world, const eval::EvalStep& step,
                  const std::vector<std::string>& reached_ids,
                  const std::vector<std::pair<double, double>>& trail,
                  const EditorSelection& sel = EditorSelection{},
                  const LidarMount& mount = LidarMount{});

// Rebuild one coherent historical frame from a recorded episode index. The
// truth trail ends at that index and every dynamic field comes from the same
// EvalStep snapshot. Returns nullopt for an invalid index.
std::optional<Frame> build_episode_frame(
 const world::World& world, const eval::EpisodeRecorder& episode,
 std::size_t episode_index, const std::vector<std::string>& reached_ids,
 const EditorSelection& sel = EditorSelection{},
 const LidarMount& mount = LidarMount{},
 std::size_t maximum_trail_points = 600);

// World-space lidar endpoints from the TRUE sensor pose (body pose
// composed with the mount, same math as generation) + recorded scan.
// Planar projection: beam tilt from body/mount pitch is not deprojected
// (same stated limit as the frozen legacy path); the ORIGIN is exact, so
// a misconfigured mount shows up instead of hiding. Invalid scan or
// non-finite ranges yield no hits.
std::vector<std::pair<double, double>> lidar_endpoints(
    double body_x_m, double body_y_m, double body_yaw_rad,
    const robot::iface::ObservationFrame& obs,
    const LidarMount& mount = LidarMount{});

}  // namespace lost_signal::sim::viewer
