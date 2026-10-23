#include "robot/autonomy/perception/perception.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"
#include "robot/core/transform.hpp"

namespace lost_signal::robot::autonomy {

LidarPerception::LidarPerception(LidarPerceptionConfig config)
    : config_(config) {}

void LidarPerception::reset() {
  tracks_.clear();
  next_id_ = 1;
}

PerceptionResult LidarPerception::update(const iface::ObservationFrame& obs,
                                         const EstimatedState& est) {
  PerceptionResult out;
  out.stamp_ns = obs.stamp_ns;
  if (!est.valid || !obs.scan_valid || obs.ranges_m.empty()) return out;
  out.scan_ok = true;
  const double now_s = obs.stamp_ns * 1e-9;

  const std::size_t n = obs.ranges_m.size();
  const bool has_ref =
      obs.scan_static_valid && obs.static_ranges_m.size() == n;
  // Without a static reference, classification is unavailable: age out
  // stale tracks, report the live ones, keep scan_ok so raw safety coverage
  // is retained downstream.
  if (!has_ref) {
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (++it->second.misses > config_.max_misses) {
        it = tracks_.erase(it);
      } else {
        DynamicTrack t = it->second.track;
        t.age_s = now_s - it->second.first_seen_s;
        out.tracks.push_back(t);
        ++it;
      }
    }
    return out;
  }

  // Beam endpoints in the map frame from the true 3D sensor pose:
  // map<-base from the estimate (x/y/z + roll/pitch/yaw) composed with the
  // TF mount base<-lidar; each beam leaves the tilted scan plane along its
  // 3D direction. Clustering/tracking use the x/y projection (the maps are
  // 2D); the range itself is the tilted-beam measurement, never flattened.
  // The angular layout matches the provider contract.
  const core::RigidTransform map_from_base{
      {est.x_m, est.y_m, est.z_m},
      core::quaternion_from_rpy(est.roll_rad, est.pitch_rad, est.yaw_rad)};
  const core::RigidTransform map_from_sensor =
      core::compose(map_from_base, base_from_lidar_);
  const core::Vec3 sensor_p = map_from_sensor.translation_m;

  struct Beam {
    bool foreground = false;
    double wx = 0.0, wy = 0.0;
  };
  std::vector<Beam> beams(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double range = obs.ranges_m[i];
    if (!(range > 0.0) || !std::isfinite(range)) continue;
    if (std::abs(range - obs.static_ranges_m[i]) <=
        config_.foreground_threshold_m)
      continue;
    const double angle = -core::kPi + 2.0 * core::kPi * i / n;
    const core::Vec3 dir = core::rotate_vector(
        map_from_sensor.rotation, {std::cos(angle), std::sin(angle), 0.0});
    beams[i].foreground = true;
    beams[i].wx = sensor_p.x + range * dir.x;
    beams[i].wy = sensor_p.y + range * dir.y;
  }

  // Pass 1: maximal runs of index-adjacent foreground beams.
  struct Run {
    std::size_t begin, end;  // [begin, end)
  };
  std::vector<Run> runs;
  for (std::size_t i = 0; i < n;) {
    if (!beams[i].foreground) {
      ++i;
      continue;
    }
    std::size_t j = i;
    while (j < n && beams[j].foreground) ++j;
    runs.push_back({i, j});
    i = j;
  }
  // Pass 2: split runs on world-space range gaps;wrap seam merges later.
  struct Cluster {
    double sx = 0.0, sy = 0.0;
    int count = 0;
  };
  std::vector<Cluster> clusters;
  for (const auto& run : runs) {
    Cluster open{};
    bool has_open = false;
    for (std::size_t i = run.begin; i < run.end; ++i) {
      if (has_open) {
        const double gap = std::hypot(
            beams[i].wx - beams[i - 1].wx, beams[i].wy - beams[i - 1].wy);
        if (gap > config_.cluster_range_gap_m) {
          clusters.push_back(open);
          open = Cluster{};
        }
      }
      open.sx += beams[i].wx;
      open.sy += beams[i].wy;
      open.count += 1;
      has_open = true;
    }
    if (has_open) clusters.push_back(open);
  }
  // Pass 3: wrap-around merge when both seam clusters are one range gap.
  if (clusters.size() > 1 && beams.front().foreground &&
      beams.back().foreground) {
    // First cluster starts at beam 0 and last ends at beam n-1 by construction.
    bool first_at_zero = runs.front().begin == 0;
    bool last_at_end = runs.back().end == n;
    if (first_at_zero && last_at_end) {
      const Cluster& first = clusters.front();
      const Cluster& last = clusters.back();
      const double gap = std::hypot(first.sx / first.count - last.sx / last.count,
                                    first.sy / first.count - last.sy / last.count);
      if (gap <= config_.cluster_range_gap_m) {
        Cluster merged = last;
        merged.sx += first.sx;
        merged.sy += first.sy;
        merged.count += first.count;
        clusters.front() = merged;
        clusters.pop_back();
      }
    }
  }
  clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
                     [&](const Cluster& c) {
                       return c.count < config_.min_beams_per_cluster;
                     }),
      clusters.end());

  // Nearest-neighbor association with a distance gate.
  std::unordered_map<std::uint32_t, bool> matched;
  for (const auto& c : clusters) {
    const double cx = c.sx / c.count, cyy = c.sy / c.count;
    std::uint32_t best_id = 0;
    double best_d = config_.association_gate_m;
    for (auto& [id, st] : tracks_) {
      const double d =
          std::hypot(cx - st.track.x_m, cyy - st.track.y_m);
      if (d < best_d) {
        best_d = d;
        best_id = id;
      }
    }
    if (best_id != 0) {
      TrackState& st = tracks_[best_id];
      // Average velocity since first sighting: immune to centroid
      // quantization jitter that finite differences amplify (0.1 m jitter at
      // 50 Hz looks like 5 m/s). Converges for steady motion; maneuvers lag
      // until the track is re-seeded by association failure.
      const double span = std::max(1e-3, now_s - st.first_seen_s);
      st.track.vx_mps = (cx - st.first_x_m) / span;
      st.track.vy_mps = (cyy - st.first_y_m) / span;
      st.track.x_m = cx;
      st.track.y_m = cyy;
      st.updates += 1;
      st.misses = 0;
      st.last_seen_s = now_s;
      st.track.prediction_ready = st.updates >= config_.min_updates_for_ready;
      matched[best_id] = true;
    } else {
      TrackState st;
      st.track.id = next_id_++;
      st.track.x_m = cx;
      st.track.y_m = cyy;
      st.first_x_m = cx;
      st.first_y_m = cyy;
      st.updates = 1;
      st.misses = 0;
      st.first_seen_s = now_s;
      st.last_seen_s = now_s;
      matched[st.track.id] = true;
      tracks_[st.track.id] = st;
    }
  }
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (!matched.count(it->first) && ++it->second.misses > config_.max_misses) {
      it = tracks_.erase(it);
    } else {
      DynamicTrack t = it->second.track;
      t.age_s = now_s - it->second.first_seen_s;
      out.tracks.push_back(t);
      ++it;
    }
  }
  return out;
}

}  // namespace lost_signal::robot::autonomy
