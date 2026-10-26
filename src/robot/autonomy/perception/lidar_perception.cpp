#include "robot/autonomy/perception/perception.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "robot/core/deskew.hpp"
#include "robot/core/geometry.hpp"
#include "robot/core/transform.hpp"

namespace lost_signal::robot::autonomy {

namespace {

core::Vec2 limited_velocity(core::Vec2 velocity, double maximum_mps) {
 const double speed = std::hypot(velocity.x, velocity.y);
 if (!(maximum_mps > 0.0) || speed <= maximum_mps || speed < 1e-9) {
 return velocity;
 }
 const double scale = maximum_mps / speed;
 return {velocity.x * scale, velocity.y * scale};
}

std::optional<double> raycast_static_map(
 const iface::MapSnapshot& map, core::Vec3 origin,
 core::Vec3 direction, double max_range_m) {
 if (!map.has_raster() || !(max_range_m > 0.0) ||
 !std::isfinite(max_range_m)) {
 return std::nullopt;
 }
 const double horizontal = std::hypot(direction.x, direction.y);
 if (horizontal < 1e-9) return std::nullopt;
 const double step_m = std::max(0.01, 0.5 * map.resolution_m / horizontal);
 for (double range_m = step_m; range_m <= max_range_m + 1e-9;
 range_m += step_m) {
 const double wx = origin.x + direction.x * range_m;
 const double wy = origin.y + direction.y * range_m;
 const int gx = static_cast<int>(
 std::floor((wx - map.origin_x_m) / map.resolution_m));
 const int gy = static_cast<int>(
 std::floor((wy - map.origin_y_m) / map.resolution_m));
 if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.width) ||
 gy >= static_cast<int>(map.height)) {
 return std::nullopt;
 }
 const int8_t cell = map.occupancy[
 static_cast<std::size_t>(gy) * map.width +
 static_cast<std::size_t>(gx)];
 if (cell < 0) return std::nullopt;
 if (cell >= 50) return std::min(range_m, max_range_m);
 }
 return max_range_m;
}

bool endpoint_matches_static_map(
 const iface::MapSnapshot& map, double wx, double wy,
 double tolerance_m) {
 if (!map.has_raster() || !(tolerance_m >= 0.0)) return false;
 const int center_x = static_cast<int>(
 std::floor((wx - map.origin_x_m) / map.resolution_m));
 const int center_y = static_cast<int>(
 std::floor((wy - map.origin_y_m) / map.resolution_m));
 const int radius = static_cast<int>(
 std::ceil(tolerance_m / map.resolution_m)) + 1;
 for (int gy = center_y - radius; gy <= center_y + radius; ++gy) {
 for (int gx = center_x - radius; gx <= center_x + radius; ++gx) {
 if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.width) ||
 gy >= static_cast<int>(map.height)) {
 continue;
 }
 if (map.occupancy[static_cast<std::size_t>(gy) * map.width +
 static_cast<std::size_t>(gx)] < 50) {
 continue;
 }
 const double x0 = map.origin_x_m + gx * map.resolution_m;
 const double y0 = map.origin_y_m + gy * map.resolution_m;
 const double dx = std::max({x0 - wx, 0.0,
 wx - (x0 + map.resolution_m)});
 const double dy = std::max({y0 - wy, 0.0,
 wy - (y0 + map.resolution_m)});
 if (std::hypot(dx, dy) <= tolerance_m) return true;
 }
 }
 return false;
}

template <typename Samples>
core::Vec2 fitted_velocity(const Samples& samples, core::Vec2 fallback,
 double maximum_mps) {
 if (samples.size() < 4) return fallback;
 double mean_t = 0.0;
 core::Vec2 mean_p;
 for (const auto& sample : samples) {
 mean_t += sample.time_s;
 mean_p.x += sample.position.x;
 mean_p.y += sample.position.y;
 }
 const double count = static_cast<double>(samples.size());
 mean_t /= count;
 mean_p.x /= count;
 mean_p.y /= count;
 double time_variance = 0.0;
 core::Vec2 covariance;
 for (const auto& sample : samples) {
 const double centered_t = sample.time_s - mean_t;
 time_variance += centered_t * centered_t;
 covariance.x += centered_t * (sample.position.x - mean_p.x);
 covariance.y += centered_t * (sample.position.y - mean_p.y);
 }
 if (time_variance < 1e-9) return fallback;
 return limited_velocity(
 {covariance.x / time_variance, covariance.y / time_variance},
 maximum_mps);
}

} // namespace

LidarPerception::LidarPerception(LidarPerceptionConfig config)
    : config_(config) {}

void LidarPerception::reset() {
  tracks_.clear();
  next_id_ = 1;
 static_map_ = nullptr;
 static_map_has_known_occupancy_ = false;
}

PerceptionResult LidarPerception::update(const iface::ObservationFrame& obs,
                                         const EstimatedState& est) {
  PerceptionResult out;
  out.stamp_ns = obs.stamp_ns;
  if (!est.valid || !obs.scan_valid || obs.ranges_m.empty()) return out;
  out.scan_ok = true;
  const double now_s = obs.stamp_ns * 1e-9;

 double map_reference_quantile = std::numeric_limits<double>::infinity();
 if (config_.foreground_false_positive_probability > 0.0 &&
 config_.foreground_false_positive_probability < 1.0) {
 // Radial quantile of an isotropic two-dimensional Gaussian:
 // P(radius > q*sigma) = exp(-q^2/2).
 map_reference_quantile = std::sqrt(
 -2.0 * std::log(config_.foreground_false_positive_probability));
 }

  const std::size_t n = obs.ranges_m.size();
 const bool has_provider_ref =
      obs.scan_static_valid && obs.static_ranges_m.size() == n;
 // A provider never needs to render an actor-free truth scan. With a static
 // reference, disagreement supplies foreground directly. Without one, track
 // compact return clusters over time in the estimated map frame and publish
 // only motion-confirmed clusters. This is weaker on the first few scans but
 // works in a genuinely unknown/SLAM environment from ROBOT observations.
 double inferred_no_return_m = 0.0;
 if (!has_provider_ref) {
 for (const float range : obs.ranges_m) {
 if (range > 0.0f && std::isfinite(range)) {
 inferred_no_return_m = std::max(inferred_no_return_m,
 static_cast<double>(range));
 }
 }
 }

 // Beam endpoints in the map frame from the estimated 3D sensor pose:
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
 bool reference_backed = false;
    double wx = 0.0, wy = 0.0;
  };
  // Rolling-shutter deskew (measured twist only): each beam re-expressed
  // in the scan-end sensor frame. Frozen scans (no offsets) read every
  // beam at the stamp pose, today's behavior exactly.
  const bool has_toff =
      obs.time_offset_valid && obs.time_offsets_s.size() == n;
  const core::DeskewTwist twist{est.forward_mps, est.yaw_rate_radps};
  std::vector<Beam> beams(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double range = obs.ranges_m[i];
    if (!(range > 0.0) || !std::isfinite(range)) continue;
    const double angle = iface::scan_beam_angle_rad(obs, i);
    const double toff = has_toff ? obs.time_offsets_s[i] : 0.0;
    // Deskewed endpoint in the scan-end sensor frame (a rigid correction
    // of the true endpoint for a static world — used directly, never
    // renormalized to the acquisition range).
    const core::Vec2 local = core::deskew_beam(range, angle, toff, twist);
    const core::Vec3 off = core::rotate_vector(
        map_from_sensor.rotation, {local.x, local.y, 0.0});
 bool reference_backed = false;
 if (has_provider_ref) {
 if (std::abs(range - obs.static_ranges_m[i]) <=
 config_.foreground_threshold_m) {
 continue;
 }
 reference_backed = true;
 } else {
 const double mount_radius_m = std::hypot(
 base_from_lidar_.translation_m.x, base_from_lidar_.translation_m.y);
 const double position_sigma_m =
 std::isfinite(est.position_sigma_m) && est.position_sigma_m >= 0.0
 ? est.position_sigma_m
 : std::numeric_limits<double>::infinity();
 const double heading_sigma_rad =
 std::isfinite(est.heading_sigma_rad) && est.heading_sigma_rad >= 0.0
 ? est.heading_sigma_rad
 : std::numeric_limits<double>::infinity();
 const double pose_projection_sigma_m = std::hypot(
 position_sigma_m,
 (range + mount_radius_m) * heading_sigma_rad);
 const double map_foreground_threshold_m =
 config_.foreground_threshold_m +
 map_reference_quantile * pose_projection_sigma_m;
 const auto expected = static_map_has_known_occupancy_
 ? raycast_static_map(
 *static_map_, sensor_p,
 {off.x / range, off.y / range, off.z / range},
 inferred_no_return_m)
 : std::optional<double>{};
 if (expected) {
 // Only a return measurably in front of the ROBOT map is foreground.
 // A farther return is clearing evidence or map mismatch, never an actor.
 if (!(range + map_foreground_threshold_m < *expected)) continue;
 reference_backed = true;
 } else if (static_map_has_known_occupancy_ && endpoint_matches_static_map(
 *static_map_, sensor_p.x + off.x, sensor_p.y + off.y,
 map_foreground_threshold_m + static_map_->resolution_m)) {
 // Sparse priors legitimately keep free space unknown. Matching a measured
 // endpoint to known occupied geometry does not require assuming the unknown
 // cells before it are free, and prevents wall fragments from becoming
 // mapless moving-object tracks as their visible centroid changes.
 continue;
 } else if (inferred_no_return_m > config_.foreground_threshold_m &&
 range >= inferred_no_return_m - config_.foreground_threshold_m) {
 // Finite max-range returns form a ring around the sensor and carry no
 // object geometry. Leave them to ray clearing.
 continue;
 }
 }
    beams[i].foreground = true;
 beams[i].reference_backed = reference_backed;
    beams[i].wx = sensor_p.x + off.x;
    beams[i].wy = sensor_p.y + off.y;
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
 double min_x = 0.0, max_x = 0.0;
 double min_y = 0.0, max_y = 0.0;
 bool reference_backed = false;
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
 has_open = false;
        }
      }
      if (!has_open) {
 open.min_x = open.max_x = beams[i].wx;
 open.min_y = open.max_y = beams[i].wy;
 } else {
 open.min_x = std::min(open.min_x, beams[i].wx);
 open.max_x = std::max(open.max_x, beams[i].wx);
 open.min_y = std::min(open.min_y, beams[i].wy);
 open.max_y = std::max(open.max_y, beams[i].wy);
 }
 open.sx += beams[i].wx;
 open.sy += beams[i].wy;
 open.count += 1;
 open.reference_backed |= beams[i].reference_backed;
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
 merged.min_x = std::min(first.min_x, last.min_x);
 merged.max_x = std::max(first.max_x, last.max_x);
 merged.min_y = std::min(first.min_y, last.min_y);
 merged.max_y = std::max(first.max_y, last.max_y);
 merged.reference_backed =
 first.reference_backed || last.reference_backed;
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

 // Predicted nearest-neighbor association with one-to-one assignment. The
 // previous implementation compared only against the last centroid and
 // allowed two clusters to update the same track, which swaps identities at
 // crossings and corrupts velocity exactly where prediction matters most.
  std::unordered_map<std::uint32_t, bool> matched;
  for (const auto& c : clusters) {
    const double cx = c.sx / c.count, cyy = c.sy / c.count;
 const double extent =
 0.5 * std::hypot(c.max_x - c.min_x, c.max_y - c.min_y);
 const double radius = std::clamp(extent, config_.min_object_radius_m,
 config_.max_object_radius_m);
 std::uint32_t best_id = 0;
    double best_d = config_.association_gate_m;
    for (auto& [id, st] : tracks_) {
 if (matched.count(id) != 0) continue;
 const double dt = std::max(0.0, now_s - st.last_seen_s);
 const double predicted_x = st.track.x_m + st.track.vx_mps * dt;
 const double predicted_y = st.track.y_m + st.track.vy_mps * dt;
 const double d = std::hypot(cx - predicted_x, cyy - predicted_y);
      if (d < best_d) {
        best_d = d;
        best_id = id;
      }
    }
    if (best_id != 0) {
      TrackState& st = tracks_[best_id];
 const double dt = std::max(1e-3, now_s - st.last_seen_s);
 const core::Vec2 previous_velocity{st.track.vx_mps, st.track.vy_mps};
 const core::Vec2 predicted{
 st.track.x_m + previous_velocity.x * dt,
 st.track.y_m + previous_velocity.y * dt};
 const core::Vec2 innovation{cx - predicted.x, cyy - predicted.y};
 const double alpha = std::clamp(config_.position_filter_gain, 0.0, 1.0);
 const double beta = std::clamp(config_.velocity_filter_gain, 0.0, 1.0);
 const core::Vec2 corrected_position{
 predicted.x + alpha * innovation.x,
 predicted.y + alpha * innovation.y};
 const core::Vec2 alpha_beta_velocity = limited_velocity(
 {previous_velocity.x + beta * innovation.x / dt,
 previous_velocity.y + beta * innovation.y / dt},
 config_.max_tracked_speed_mps);
 st.motion_samples.push_back({now_s, {cx, cyy}});
 while (!st.motion_samples.empty() &&
 now_s - st.motion_samples.front().time_s >
 std::max(0.1, config_.velocity_fit_window_s)) {
 st.motion_samples.pop_front();
 }
 const core::Vec2 velocity = fitted_velocity(
 st.motion_samples, alpha_beta_velocity, config_.max_tracked_speed_mps);
 const double position_residual = std::hypot(innovation.x, innovation.y);
 const double velocity_residual = std::hypot(
 velocity.x - previous_velocity.x, velocity.y - previous_velocity.y);
 st.position_error_ema_m = std::clamp(
 0.85 * st.position_error_ema_m + 0.15 * position_residual,
 std::max(0.0, config_.position_uncertainty_floor_m),
 std::max(config_.position_uncertainty_floor_m,
 config_.position_uncertainty_cap_m));
 st.velocity_error_ema_mps = std::max(
 std::max(0.0, config_.velocity_uncertainty_floor_mps),
 0.85 * st.velocity_error_ema_mps + 0.15 * velocity_residual);
 st.track.x_m = corrected_position.x;
 st.track.y_m = corrected_position.y;
 st.track.vx_mps = velocity.x;
 st.track.vy_mps = velocity.y;
 st.track.radius_m = 0.7 * st.track.radius_m + 0.3 * radius;
 st.track.position_uncertainty_m = st.position_error_ema_m;
 st.track.position_uncertainty_growth_mps = std::max(
 config_.uncertainty_growth_floor_mps, st.velocity_error_ema_mps);
 st.track.position_uncertainty_cap_m =
 std::max(st.track.position_uncertainty_m,
 config_.position_uncertainty_cap_m);
 st.track.velocity_uncertainty_mps = st.velocity_error_ema_mps;
 st.track.last_observed_ns = obs.stamp_ns;
 st.track.min_observed.x = std::min(st.track.min_observed.x, cx);
 st.track.min_observed.y = std::min(st.track.min_observed.y, cyy);
 st.track.max_observed.x = std::max(st.track.max_observed.x, cx);
 st.track.max_observed.y = std::max(st.track.max_observed.y, cyy);
 const double speed = std::hypot(velocity.x, velocity.y);
 const double motion_gate = std::max(0.0, config_.min_dynamic_speed_mps);
 if (speed >= motion_gate) {
 ++st.moving_updates;
 st.stationary_evidence_s = std::max(
 0.0, st.stationary_evidence_s - dt);
 if (velocity.x > motion_gate) st.seen_positive_x = true;
 if (velocity.x < -motion_gate) st.seen_negative_x = true;
 if (velocity.y > motion_gate) st.seen_positive_y = true;
 if (velocity.y < -motion_gate) st.seen_negative_y = true;
 st.last_reliable_velocity = velocity;
 st.reliable_velocity_valid = true;
 } else {
 st.moving_updates = 0;
 st.stationary_evidence_s += dt;
 }
 const double motion_history_s = st.motion_samples.size() >= 2
 ? st.motion_samples.back().time_s - st.motion_samples.front().time_s
 : 0.0;
 if (st.moving_updates >= std::max(1, config_.min_motion_updates) &&
 motion_history_s >= std::max(0.0, config_.velocity_fit_window_s)) {
 st.mapless_dynamic_confirmed = true;
 }
 const double static_reclassification_time_s =
 std::max(0.0, config_.static_reclassification_time_s);
 if (st.mapless_dynamic_confirmed &&
 st.stationary_evidence_s >= static_reclassification_time_s) {
 // Raw scan endpoints remain in the rolling occupancy layer. Releasing a
 // stopped forecast prevents one object from being charged indefinitely as
 // both static geometry and a dynamic prediction; coherent motion can
 // promote it again through the same time-based gate above.
 st.mapless_dynamic_confirmed = false;
 st.moving_updates = 0;
 st.stationary_evidence_s = static_reclassification_time_s;
 }
 st.track.bounded_x = st.seen_positive_x && st.seen_negative_x &&
 st.track.max_observed.x - st.track.min_observed.x >= 0.20;
 st.track.bounded_y = st.seen_positive_y && st.seen_negative_y &&
 st.track.max_observed.y - st.track.min_observed.y >= 0.20;
 st.track.alternate_velocity_valid =
 st.mapless_dynamic_confirmed && speed < motion_gate &&
 st.reliable_velocity_valid;
 st.track.alternate_velocity = st.last_reliable_velocity;
 st.updates += 1;
      st.misses = 0;
      st.last_seen_s = now_s;
 st.created_with_static_reference |= c.reference_backed;
 st.track.prediction_ready =
 st.updates >= config_.min_updates_for_ready &&
 st.mapless_dynamic_confirmed;
      matched[best_id] = true;
    } else {
      TrackState st;
      st.track.id = next_id_++;
      st.track.x_m = cx;
 st.track.y_m = cyy;
 st.track.radius_m = radius;
 st.track.last_observed_ns = obs.stamp_ns;
 st.track.position_uncertainty_m =
 std::max(0.0, config_.position_uncertainty_floor_m);
 st.track.position_uncertainty_growth_mps =
 std::max(0.0, config_.uncertainty_growth_floor_mps);
 st.track.position_uncertainty_cap_m = std::max(
 st.track.position_uncertainty_m, config_.position_uncertainty_cap_m);
 st.track.velocity_uncertainty_mps =
 std::max(0.0, config_.velocity_uncertainty_floor_mps);
 st.track.min_observed = {cx, cyy};
 st.track.max_observed = {cx, cyy};
 st.first_x_m = cx;
      st.first_y_m = cyy;
      st.updates = 1;
      st.misses = 0;
      st.first_seen_s = now_s;
      st.last_seen_s = now_s;
 st.created_with_static_reference = c.reference_backed;
 st.position_error_ema_m = st.track.position_uncertainty_m;
 st.velocity_error_ema_mps = st.track.velocity_uncertainty_mps;
 st.motion_samples.push_back({now_s, {cx, cyy}});
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
 const bool unresolved_reference_foreground =
 it->second.created_with_static_reference &&
 it->second.updates <
 std::max(1, config_.min_updates_for_ready);
 if (unresolved_reference_foreground ||
 it->second.mapless_dynamic_confirmed) {
 out.tracks.push_back(t);
 }
      ++it;
    }
  }
  return out;
}

}  // namespace lost_signal::robot::autonomy
