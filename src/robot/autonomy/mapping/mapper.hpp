#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/mapping/elevation.hpp"
#include "robot/autonomy/mapping/log_odds.hpp"
#include "robot/autonomy/mapping/terrain_math.hpp"
#include "robot/core/geometry.hpp"
#include "robot/core/time.hpp"
#include "robot/core/transform.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"
#include "robot/core/deskew.hpp"

namespace lost_signal::robot::autonomy {

// Rolling local world model derived from the sensor stream. Occupancy is
// -1 unknown, 0 free, 100 occupied (row-major, x fastest). Unknown is
// conservative for planning and safety: a cell becomes traversable only
// after a sensor ray or SLAM replay marks it free.
struct LocalMap {
  bool valid = false;
  double resolution_m = 0.1;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double origin_x_m = 0.0;  // map-frame pose of cell (0,0)
  double origin_y_m = 0.0;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  std::vector<int8_t> occupancy;  // -1 unknown; size width*height when valid
  // Optional 2.5D layer, same geometry. NaN = unknown (planar scans cannot
  // observe height; a multi-line provider fills this, see docs).
  std::vector<float> elevation_m;

  bool has_elevation() const {
    return valid && elevation_m.size() == width * height;
  }
  float elevation_at(double wx, double wy) const {
    if (!has_elevation()) return std::numeric_limits<float>::quiet_NaN();
    const int gx = static_cast<int>(
        std::floor((wx - origin_x_m) / resolution_m));
    const int gy = static_cast<int>(
        std::floor((wy - origin_y_m) / resolution_m));
    if (!inside(gx, gy)) return std::numeric_limits<float>::quiet_NaN();
    return elevation_m[static_cast<std::size_t>(gy) * width + gx];
  }

  bool inside(int gx, int gy) const {
    return gx >= 0 && gy >= 0 && gx < static_cast<int>(width) &&
           gy < static_cast<int>(height);
  }
 // Free means observed-free. Unknown is never free.
  bool is_free(double wx, double wy) const;
 // Blocked includes unknown and out-of-window cells. A planner may only
 // enter space explicitly cleared by a sensor ray or SLAM replay.
  bool is_blocked(double wx, double wy) const;
 // Continuous line-of-travel query, sampled at <= half a cell so a segment
 // cannot jump an occupied or unknown cell.
  bool segment_is_free(double x0, double y0, double x1, double y1) const;
 // Planner admission query. Unknown and outside-window cells are blocked.
  bool segment_is_traversable(double x0, double y0, double x1,
                              double y1) const;
  // Exact world-space distance to the occupied set. Occupied cells own
  // their full square; outside the window counts as occupied. Zero inside
  // occupied space; continuous in free space, suitable as a planner critic.
  double clearance(double wx, double wy) const;
  // Inflate occupied cells by radius into a binary costmap. Transient
  // observation layers ignore the boundary (the base map owns that policy;
  // an empty sensor layer must contribute no obstacles).
  LocalMap inflated(double radius_m) const;
};

// IMapper: observations -> rolling local map around the estimate.
class IMapper {
 public:
  virtual ~IMapper() = default;
  virtual void reset() = 0;
  virtual void update(const iface::ObservationFrame& obs,
                      const EstimatedState& est) = 0;
  virtual LocalMap local_map() const = 0;
  // Full lidar mount from the TF tree (coordinator fans out).
  virtual void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
    (void)base_from_lidar;
  }
  // Surveyed-elevation prior for ground classification (coordinator pushes
  // on map revision; default ignores). With a survey, background returns
  // consistent with surveyed ground clear the ray but never paint occupied
  // (ramps stay traversable); without, today's behavior exactly.
  virtual void set_survey(const iface::MapSnapshot& map) { (void)map; }
 // Dynamic tracks are ROBOT-side evidence used to keep associated
 // foreground returns out of the persistent static layer.
 virtual void set_dynamic_tracks(const std::vector<DynamicTrack>& tracks) {
 (void)tracks;
 }
};

// v1: fixed-size window centered on the estimate, all cells unknown.
// Evidence fuses through log-odds (transient returns fade, static walls
// persist); the published raster keeps the query semantics above.
class RayClearingMapper : public IMapper {
 public:
  void reset() override {
    map_ = LocalMap{};
    fusion_.reset(0);
    elevation_.reset(0);
 dynamic_tracks_.clear();
  }
  void set_lidar_mount(const core::RigidTransform& base_from_lidar) override {
    base_from_lidar_ = base_from_lidar;
  }
  void set_survey(const iface::MapSnapshot& map) override { survey_ = map; }
 void set_dynamic_tracks(const std::vector<DynamicTrack>& tracks) override {
 dynamic_tracks_ = tracks;
 }
  void update(const iface::ObservationFrame& obs,
              const EstimatedState& est) override {
    if (!est.valid) {
      map_.valid = false;
      return;
    }
    // Rolling window: recenter (shifting old content) when the estimate
    // leaves the inner half. Evidence is world-frame: the correction's
    // translational part remaps cell beliefs rigidly (nearest cell, error
    // <= half a cell, absorbed by later fusion); rotation never moves
    // cells (the grid has no heading — yaw only moves the robot within
    // it). Fresh windows (invalid/empty/resized) reset, exactly as before.
    const double half_w = map_.width * map_.resolution_m * 0.5;
    const double half_h = map_.height * map_.resolution_m * 0.5;
    bool need_window = !map_.valid || map_.occupancy.size() !=
                                            map_.width * map_.height;
    if (!need_window) {
      const double cx = map_.origin_x_m + half_w;
      const double cyy = map_.origin_y_m + half_h;
      need_window = std::abs(est.x_m - cx) > half_w * 0.5 ||
                    std::abs(est.y_m - cyy) > half_h * 0.5;
    }
    if (need_window) {
      const double new_ox = est.x_m - 5.0;
      const double new_oy = est.y_m - 5.0;
      const bool can_shift =
          map_.valid && map_.resolution_m == 0.1 && map_.width == 100 &&
          map_.height == 100 &&
          fusion_.cell_count() == map_.width * map_.height &&
          elevation_.cell_count() == map_.width * map_.height;
      map_.valid = true;
      map_.resolution_m = 0.1;
      map_.width = 100;
      map_.height = 100;
      if (can_shift) {
        // Rigid remap of both belief layers by world coordinates (new
        // cells with no old source read default = unknown, never a guess).
        // Snapshot first: overlapping shifts would otherwise read cells
        // an earlier write already overwrote.
        std::vector<OccupancyLogOdds::CellState> old_occ(
            map_.width * map_.height);
        std::vector<ElevationFusion::CellState> old_elv(
            map_.width * map_.height);
        for (std::size_t c = 0; c < old_occ.size(); ++c) {
          old_occ[c] = fusion_.cell_state(c);
          old_elv[c] = elevation_.cell_state(c);
        }
        const double old_ox = map_.origin_x_m;
        const double old_oy = map_.origin_y_m;
        for (std::uint32_t gy = 0; gy < 100; ++gy) {
          for (std::uint32_t gx = 0; gx < 100; ++gx) {
            const double wx = new_ox + (gx + 0.5) * 0.1;
            const double wy = new_oy + (gy + 0.5) * 0.1;
            const int ogx =
                static_cast<int>(std::floor((wx - old_ox) / 0.1));
            const int ogy =
                static_cast<int>(std::floor((wy - old_oy) / 0.1));
            const std::size_t ni = static_cast<std::size_t>(gy) * 100 + gx;
            if (ogx < 0 || ogy < 0 || ogx >= 100 || ogy >= 100) {
              fusion_.set_cell_state(ni, OccupancyLogOdds::CellState{});
              elevation_.set_cell_state(ni, ElevationFusion::CellState{});
              continue;
            }
            const std::size_t oi =
                static_cast<std::size_t>(ogy) * 100 + ogx;
            fusion_.set_cell_state(ni, old_occ[oi]);
            elevation_.set_cell_state(ni, old_elv[oi]);
          }
        }
      } else {
        map_.occupancy.assign(map_.width * map_.height, -1);
        fusion_.reset(map_.width * map_.height);
        elevation_.reset(map_.width * map_.height);
        map_.elevation_m.assign(map_.width * map_.height,
                                std::numeric_limits<float>::quiet_NaN());
      }
      map_.origin_x_m = new_ox;
      map_.origin_y_m = new_oy;
      // Published rasters refresh now (the early return on missing scans
      // below must never serve the old window's values under new origins).
      publish();
    }
    if (fusion_.cell_count() != map_.width * map_.height) {
      fusion_.reset(map_.width * map_.height);  // fail closed, never alias
    }
    if (elevation_.cell_count() != map_.width * map_.height) {
      elevation_.reset(map_.width * map_.height);
      map_.elevation_m.assign(map_.width * map_.height,
                              std::numeric_limits<float>::quiet_NaN());
    }
    map_.stamp_ns = obs.stamp_ns;
    if (!obs.scan_valid || obs.ranges_m.empty()) return;
    // Ray clearing from the true 3D sensor pose (estimate + TF mount);
    // endpoints project to x/y (the rolling map is 2D).
    const core::RigidTransform map_from_base{
        {est.x_m, est.y_m, est.z_m},
        core::quaternion_from_rpy(est.roll_rad, est.pitch_rad, est.yaw_rad)};
    const core::RigidTransform map_from_sensor =
        core::compose(map_from_base, base_from_lidar_);
    const double ssx = map_from_sensor.translation_m.x;
    const double ssy = map_from_sensor.translation_m.y;
    const double ssz = map_from_sensor.translation_m.z;
    // Surveyed sensor height for the ground classifier (estimate z rides
    // above the spawn plane; mount + ride complete the absolute height).
    // No survey (or no elevation layer) paints as before, exactly.
    const bool use_survey = survey_.has_elevation();
    const double survey_sensor_z =
        est.z_m + base_from_lidar_.translation_m.z + 0.10;
 // With a static reference, only matching returns paint endpoint cells. In
 // an unknown environment there is no reference to compare against, so a
 // return is occupied evidence unless it is associated with a ROBOT-side
 // dynamic track. Unmatched returns remain conservative.
    // Surveyed-ground background beams clear but never paint (ramps stay
    // traversable — same classifier as the validator, one doctrine).
    const std::size_t n = obs.ranges_m.size();
    const bool has_ref =
        obs.scan_static_valid && obs.static_ranges_m.size() == n;
    // Rolling-shutter deskew (measured twist only; frozen scans read every
    // beam at the stamp pose). Same helper as perception, one doctrine.
    const bool has_toff =
        obs.time_offset_valid && obs.time_offsets_s.size() == n;
    const core::DeskewTwist twist{est.forward_mps, est.yaw_rate_radps};
    for (std::size_t i = 0; i < n; ++i) {
      const float range = obs.ranges_m[i];
      if (!(range > 0.0) || !std::isfinite(range)) continue;
      const double angle = iface::scan_beam_angle_rad(obs, i);
      const double toff = has_toff ? obs.time_offsets_s[i] : 0.0;
      const core::Vec2 local = core::deskew_beam(range, angle, toff, twist);
      const core::Vec3 off = core::rotate_vector(
          map_from_sensor.rotation, {local.x, local.y, 0.0});
      const double ex = ssx + off.x;
      const double ey = ssy + off.y;
      clear_segment(ssx, ssy, ex, ey);
 bool background = !has_ref;
      if (has_ref) {
        background =
            std::abs(range - obs.static_ranges_m[i]) <= 0.30;
      }
 if (background) {
 for (const auto& track : dynamic_tracks_) {
 const double track_radius =
 std::max(0.05, track.occupied_radius_m() + 0.20);
 if (std::hypot(ex - track.x_m, ey - track.y_m) <= track_radius) {
 background = false;
 break;
 }
 }
 }
      if (background) {
        if (use_survey &&
            endpoint_is_survey_ground(survey_, ex, ey, survey_sensor_z,
                                      0.15)) {
          // Live elevation: this return measured the surface at (ex, ey)
          // (reconstructed 3D endpoint height, TF sensor height + rotated
          // beam). Fuses only survey-confirmed ground — actor heights and
          // wall faces never enter the terrain layer. Pose uncertainty
          // rides from the estimate (correlated across the scan, never
          // averaged away); the device assumption is documented config.
          int gx, gy;
          world_to_grid(ex, ey, gx, gy);
          if (map_.inside(gx, gy)) {
            elevation_.fuse(static_cast<std::size_t>(gy) * map_.width + gx,
                            ssz + off.z, est.position_sigma_m);
          }
          continue;  // surveyed ground: cleared above, never painted
        }
        occupy(ex, ey);
      }
    }
    // Publish the fused rasters (hysteresis inside: one noisy frame cannot
    // flip a confirmed wall; disagreement beyond the sigma cap publishes
    // unknown so downstream falls back to the survey prior, never a guess).
    publish();
  }
  LocalMap local_map() const override { return map_; }

 private:
  void publish() {
    map_.occupancy = fusion_.values();
    if (map_.elevation_m.size() != map_.width * map_.height) {
      map_.elevation_m.assign(map_.width * map_.height,
                              std::numeric_limits<float>::quiet_NaN());
    }
    for (std::size_t c = 0; c < map_.elevation_m.size(); ++c) {
      const ElevationEstimate e = elevation_.estimate(c);
      map_.elevation_m[c] = e.valid ? static_cast<float>(e.height_m)
                                    : std::numeric_limits<float>::quiet_NaN();
    }
  }
  void clear_segment(double x0, double y0, double x1, double y1) {
    const double length = std::hypot(x1 - x0, y1 - y0);
    const int steps = std::max(
        1, static_cast<int>(
               std::ceil(length / (0.5 * map_.resolution_m))));
    std::size_t last = map_.width * map_.height;  // sentinel: no cell
    for (int s = 0; s < steps; ++s) {  // exclude endpoint: actor cells stay
      const double b = static_cast<double>(s) / steps;
      int gx, gy;
      world_to_grid(x0 + (x1 - x0) * b, y0 + (y1 - y0) * b, gx, gy);
      if (!map_.inside(gx, gy)) continue;
      const std::size_t idx =
          static_cast<std::size_t>(gy) * map_.width + gx;
      if (idx == last) continue;  // one vote per cell per scan, no pile-up
      last = idx;
      fusion_.observe_free(idx);
    }
  }
  void occupy(double wx, double wy) {
    int gx, gy;
    world_to_grid(wx, wy, gx, gy);
    if (map_.inside(gx, gy)) {
      fusion_.observe_occupied(static_cast<std::size_t>(gy) * map_.width +
                               gx);
    }
  }
  void world_to_grid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - map_.origin_x_m) / map_.resolution_m));
    gy = static_cast<int>(std::floor((wy - map_.origin_y_m) / map_.resolution_m));
  }

  LocalMap map_{};
  OccupancyLogOdds fusion_{};
  // Live terrain heights from surveyed-ground returns. The 0.03 m device
  // assumption is the ROBOT's documented ranging model (same order as the
  // lidar device noise); pose uncertainty adds per observation from the
  // estimate. Never SIM truth: inputs are ranges + estimate + prior.
  ElevationFusion elevation_{};
  core::RigidTransform base_from_lidar_{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
  iface::MapSnapshot survey_;  // empty until set_survey (no elevation)
 std::vector<DynamicTrack> dynamic_tracks_;
};

}  // namespace lost_signal::robot::autonomy
