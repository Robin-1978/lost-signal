#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/mapping/mapper.hpp"

namespace {
lost_signal::robot::iface::ObservationFrame make_scan(
    lost_signal::robot::core::MonotonicNs stamp, float bg, int fg_lo,
    int fg_hi, float fg_range) {
  using namespace lost_signal::robot;
  iface::ObservationFrame o;
  o.sequence = static_cast<std::uint64_t>(stamp);
  o.stamp_ns = stamp;
  o.source = "t.lidar";
  o.frame = core::Frame::kLidarLink;
  o.odometry_valid = true;
  o.scan_valid = true;
  o.ranges_m.assign(72, bg);
  o.static_ranges_m.assign(72, bg);
  o.scan_static_valid = true;
  for (int i = fg_lo; i <= fg_hi; ++i) o.ranges_m[i] = fg_range;
  return o;
}

lost_signal::robot::autonomy::EstimatedState origin() {
  lost_signal::robot::autonomy::EstimatedState e;
  e.valid = true;
  e.stamp_ns = 1;
  return e;
}
}  // namespace

int main() {
  using namespace lost_signal::robot::autonomy;

  // Foreground bump vs empty reference -> one track near the bump.
  {
    LidarPerception perc;
    // Beams 34..38 of 72 span angles ~ -9deg..+9deg -> +x direction.
    auto obs = make_scan(1'000'000, 10.0f, 34, 38, 3.0f);
    PerceptionResult r = perc.update(obs, origin());
    assert(r.scan_ok);
    assert(r.tracks.size() == 1);
    assert(std::abs(r.tracks[0].x_m - 3.0) < 0.4);
    assert(std::abs(r.tracks[0].y_m) < 0.4);
    assert(!r.tracks[0].prediction_ready);  // single sighting: not ready
  }

  // Non-zero scan start with an explicit increment: beams 0..4 point +x.
 // The old hardcoded [-pi, pi) layout put them near -x.
 {
 LidarPerception perc;
 auto obs = make_scan(1'000'000, 10.0f, 0, 4, 3.0f);
 obs.scan_angle_min_rad = 0.0;
 obs.scan_angle_increment_rad = 2.0 * M_PI / 72.0;
 PerceptionResult r = perc.update(obs, origin());
 assert(r.scan_ok);
 assert(r.tracks.size() == 1);
 assert(std::abs(r.tracks[0].x_m - 3.0) < 0.4);
 assert(r.tracks[0].y_m > 0.2 && r.tracks[0].y_m < 0.9);
 }

 // A local forward sector uses the same start/increment contract.
 {
 LidarPerception perc;
 auto obs = make_scan(1'000'000, 10.0f, 0, 4, 3.0f);
 obs.scan_angle_min_rad = -0.02;
 obs.scan_angle_increment_rad = 0.01;
 PerceptionResult r = perc.update(obs, origin());
 assert(r.tracks.size() == 1);
 assert(std::abs(r.tracks[0].x_m - 3.0) < 0.4);
 assert(std::abs(r.tracks[0].y_m) < 0.1);
 }

 // Cluster extent drives the conservative occupied radius.
 {
 LidarPerception perc;
 auto obs = make_scan(1'000'000, 10.0f, 34, 38, 3.0f);
 PerceptionResult r = perc.update(obs, origin());
 assert(r.tracks.size() == 1);
 assert(r.tracks[0].radius_m >= 0.15);
 assert(r.tracks[0].radius_m <= 0.9);
 }

 // A foreground return is held conservatively while unresolved. Repeated
 // stationary evidence then hands it to the rolling static map instead of
 // inventing a dynamic prediction for an unreported crate or wall.
  {
    LidarPerception perc;
    std::uint32_t id = 0;
    for (int k = 1; k <= 4; ++k) {
      auto obs = make_scan(k * 1'000'000, 10.0f, 34, 38, 3.0f);
      PerceptionResult r = perc.update(obs, origin());
 if (k < 3) {
 assert(r.tracks.size() == 1);
 if (id == 0) {
        id = r.tracks[0].id;
      } else {
        assert(r.tracks[0].id == id);
      }
 } else {
 assert(r.tracks.empty());
 }
    }
 }

 // Motion-confirmed foreground remains an anonymous dynamic track and only
 // becomes prediction-ready after timestamp-distinct velocity evidence.
 {
 LidarPerception perc;
 PerceptionResult result;
 for (int k = 1; k <= 12; ++k) {
 auto obs = make_scan(
 k * 100'000'000LL, 10.0f, 34, 38,
 3.0f + 0.10f * static_cast<float>(k));
 result = perc.update(obs, origin());
 }
 assert(result.tracks.size() == 1);
 assert(result.tracks.front().prediction_ready);
  }

 // A confirmed mover that becomes stationary returns to the rolling static
 // layer after timestamped evidence, so its current geometry is not charged
 // forever by both occupancy and dynamic prediction.
 {
 LidarPerceptionConfig config;
 config.static_reclassification_time_s = 0.50;
 LidarPerception perc(config);
 PerceptionResult result;
 for (int k = 1; k <= 12; ++k) {
 auto obs = make_scan(
 k * 100'000'000LL, 10.0f, 34, 38,
 3.0f + 0.10f * static_cast<float>(k));
 result = perc.update(obs, origin());
 }
 assert(result.tracks.size() == 1);
 for (int k = 13; k <= 24; ++k) {
 auto obs = make_scan(k * 100'000'000LL, 10.0f, 34, 38, 4.2f);
 result = perc.update(obs, origin());
 }
 assert(result.tracks.empty());
 }

 // A confirmed mover keeps its anonymous identity through an ordinary stop.
 // Its current hypothesis becomes stationary while the last reliable motion
 // remains an alternate intent. Resuming does not pay a second one-second
 // warm-up or change the track id.
 {
 LidarPerception perc;
 PerceptionResult result;
 for (int k = 1; k <= 12; ++k) {
 auto obs = make_scan(
 k * 100'000'000LL, 10.0f, 34, 38,
 3.0f + 0.10f * static_cast<float>(k));
 result = perc.update(obs, origin());
 }
 assert(result.tracks.size() == 1);
 const std::uint32_t id = result.tracks.front().id;
 for (int k = 13; k <= 42; ++k) {
 auto obs = make_scan(k * 100'000'000LL, 10.0f, 34, 38, 4.2f);
 result = perc.update(obs, origin());
 assert(result.tracks.size() == 1);
 assert(result.tracks.front().id == id);
 assert(result.tracks.front().prediction_ready);
 }
 assert(std::hypot(result.tracks.front().vx_mps,
 result.tracks.front().vy_mps) < 0.18);
 assert(result.tracks.front().alternate_velocity_valid);
 for (int k = 43; k <= 48; ++k) {
 auto obs = make_scan(
 k * 100'000'000LL, 10.0f, 34, 38,
 4.2f + 0.10f * static_cast<float>(k - 42));
 result = perc.update(obs, origin());
 assert(result.tracks.size() == 1);
 assert(result.tracks.front().id == id);
 assert(result.tracks.front().prediction_ready);
 }
 }

  // Departed object: beams back to reference -> track ages out.
  {
    LidarPerception perc;
    for (int k = 1; k <= 2; ++k) {
      auto obs = make_scan(k * 1'000'000, 10.0f, 34, 38, 3.0f);
      perc.update(obs, origin());
    }
    for (int k = 3; k <= 6; ++k) {
      auto obs = make_scan(k * 1'000'000, 10.0f, 0, -1, 0.0f);  // no bump
      PerceptionResult r = perc.update(obs, origin());
      if (k <= 5) {
        (void)r;
      } else {
        assert(r.tracks.empty());  // 4 misses > max_misses=3
      }
    }
  }

  // TF mount: forward-shifted lidar moves the track +x by the mount.
  {
    LidarPerception perc;
    perc.set_lidar_mount({{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}});
    auto obs = make_scan(1'000'000, 10.0f, 34, 38, 3.0f);
    PerceptionResult r = perc.update(obs, origin());
    assert(r.tracks.size() == 1);
    assert(std::abs(r.tracks[0].x_m - 4.0) < 0.4);  // 3.0 range + 1.0 mount
  }

  // No static reference: classification unavailable, coverage retained.
  {
    LidarPerception perc;
    auto obs = make_scan(1'000'000, 10.0f, 34, 38, 3.0f);
    obs.scan_static_valid = false;
    obs.static_ranges_m.clear();
    PerceptionResult r = perc.update(obs, origin());
    assert(r.scan_ok);
    assert(r.tracks.empty());
  }

 // A ROBOT occupancy prior supplies the static reference without a simulator
 // actor-free scan. Returns on the mapped wall stay static; a closer return is
 // foreground. Unknown cells would fall back to temporal scan tracking.
 {
 LidarPerception perc;
 lost_signal::robot::iface::MapSnapshot map;
 map.map_id = "prior";
 map.revision = 1;
 map.width = 200;
 map.height = 200;
 map.resolution_m = 0.1;
 map.origin_x_m = -10.0;
 map.origin_y_m = -10.0;
 map.occupancy.assign(map.width * map.height, 0);
 for (int gy = 85; gy <= 115; ++gy) {
 map.occupancy[static_cast<std::size_t>(gy) * map.width + 130] = 100;
 }
 perc.set_static_map(map);
 for (int frame = 0; frame < 4; ++frame) {
 auto wall = make_scan(
 (frame + 1) * 100'000'000LL, 10.0f, 34, 38, 3.0f);
 wall.scan_static_valid = false;
 wall.static_ranges_m.clear();
 assert(perc.update(wall, origin()).tracks.empty());
 }
 auto nearer = make_scan(500'000'000LL, 10.0f, 34, 38, 2.0f);
 nearer.scan_static_valid = false;
 nearer.static_ranges_m.clear();
 assert(perc.update(nearer, origin()).tracks.size() == 1);
 }

 // A displaced pose with matching covariance must not turn a surveyed wall
 // into a map-relative moving actor. The foreground gate is expanded from the
 // reported position and heading uncertainty; raw scan collision evidence is
 // still available to planning and safety while classification is unresolved.
 {
 LidarPerception perc;
 lost_signal::robot::iface::MapSnapshot map;
 map.map_id = "uncertain-prior";
 map.revision = 1;
 map.width = 200;
 map.height = 200;
 map.resolution_m = 0.1;
 map.origin_x_m = -10.0;
 map.origin_y_m = -10.0;
 map.occupancy.assign(map.width * map.height, 0);
 for (int gy = 85; gy <= 115; ++gy) {
 map.occupancy[static_cast<std::size_t>(gy) * map.width + 130] = 100;
 }
 perc.set_static_map(map);
 EstimatedState uncertain = origin();
 uncertain.x_m = -2.0;
 uncertain.position_sigma_m = 2.0;
 uncertain.heading_sigma_rad = 0.1;
 for (int frame = 0; frame < 5; ++frame) {
 auto wall = make_scan(
 (frame + 1) * 100'000'000LL, 10.0f, 34, 38, 3.0f);
 wall.scan_static_valid = false;
 wall.static_ranges_m.clear();
 const PerceptionResult result = perc.update(wall, uncertain);
 assert(result.scan_ok);
 assert(result.tracks.empty());
 }
 }

 // Pose uncertainty may prevent one scan from claiming map-relative
 // foreground, but it must not erase a moving return that repeatedly stays
 // in front of the surveyed wall. Such evidence remains temporal-only until
 // the ordinary velocity-fit window confirms motion.
 {
 LidarPerception perc;
 lost_signal::robot::iface::MapSnapshot map;
 map.map_id = "uncertain-moving-prior";
 map.revision = 1;
 map.width = 200;
 map.height = 200;
 map.resolution_m = 0.1;
 map.origin_x_m = -10.0;
 map.origin_y_m = -10.0;
 map.occupancy.assign(map.width * map.height, 0);
 for (int gy = 85; gy <= 115; ++gy) {
 map.occupancy[static_cast<std::size_t>(gy) * map.width + 130] = 100;
 }
 perc.set_static_map(map);
 EstimatedState uncertain = origin();
 uncertain.position_sigma_m = 2.0;
 uncertain.heading_sigma_rad = 0.1;
 PerceptionResult result;
 for (int frame = 0; frame < 12; ++frame) {
 auto mover = make_scan(
 (frame + 1) * 100'000'000LL, 10.0f, 34, 38,
 1.0f + 0.10f * static_cast<float>(frame));
 mover.scan_static_valid = false;
 mover.static_ranges_m.clear();
 result = perc.update(mover, uncertain);
 if (frame < 10) assert(result.tracks.empty());
 }
 assert(result.tracks.size() == 1);
 assert(result.tracks.front().prediction_ready);
 assert(result.tracks.front().vx_mps > 0.3);
 }

 // A sparse prior may have unknown cells before nearby mapped structure.
 // Localization covariance alone cannot turn the whole neighborhood into
 // static geometry: a moving return several metres from that structure
 // remains temporal-only and becomes ready after measured motion history.
 {
 LidarPerception perc;
 lost_signal::robot::iface::MapSnapshot map;
 map.map_id = "sparse-uncertain-moving-prior";
 map.revision = 1;
 map.width = 240;
 map.height = 240;
 map.resolution_m = 0.1;
 map.origin_x_m = -12.0;
 map.origin_y_m = -12.0;
 map.occupancy.assign(map.width * map.height, -1);
 for (int gy = 78; gy <= 82; ++gy) {
 for (int gx = 118; gx <= 122; ++gx) {
 map.occupancy[static_cast<std::size_t>(gy) * map.width + gx] = 100;
 }
 }
 perc.set_static_map(map);
 EstimatedState uncertain = origin();
 uncertain.position_sigma_m = 2.0;
 uncertain.heading_sigma_rad = 0.1;
 PerceptionResult result;
 for (int frame = 0; frame < 12; ++frame) {
 auto mover = make_scan(
 (frame + 1) * 100'000'000LL, 10.0f, 34, 38,
 1.0f + 0.10f * static_cast<float>(frame));
 mover.scan_static_valid = false;
 mover.static_ranges_m.clear();
 result = perc.update(mover, uncertain);
 if (frame < 10) assert(result.tracks.empty());
 }
 assert(result.tracks.size() == 1);
 assert(result.tracks.front().prediction_ready);
 assert(result.tracks.front().vx_mps > 0.3);
 }

 // Mapless/SLAM startup: static compact returns remain unpublished, while a
 // repeatedly moving cluster becomes an anonymous dynamic track using only
 // scan history in the estimated map frame.
 {
 LidarPerception perc;
 PerceptionResult result;
 for (int frame = 0; frame < 12; ++frame) {
 auto obs = make_scan(
 (frame + 1) * 100'000'000LL, 10.0f, 34, 38,
 3.0f + 0.10f * static_cast<float>(frame));
 obs.scan_static_valid = false;
 obs.static_ranges_m.clear();
 result = perc.update(obs, origin());
 if (frame < 10) assert(result.tracks.empty());
 }
 assert(result.tracks.size() == 1);
 assert(result.tracks.front().prediction_ready);
 assert(result.tracks.front().vx_mps > 0.3);
 assert(result.tracks.front().last_observed_ns == 1'200'000'000LL);
 }

 // REP-105: gradual map<-odom localization correction moves a static scan
 // rigidly in map, but must not become temporal object velocity.
 {
 LidarPerception perc;
 EstimatedState odom = origin();
 PerceptionResult result;
 for (int frame = 0; frame < 16; ++frame) {
 EstimatedState map_estimate = odom;
 map_estimate.x_m = 0.05 * static_cast<double>(frame);
 auto wall = make_scan(
 (frame + 1) * 100'000'000LL, 10.0f, 34, 38, 3.0f);
 wall.scan_static_valid = false;
 wall.static_ranges_m.clear();
 result = perc.update(wall, map_estimate, odom);
 assert(result.tracks.empty());
 }
 }
 {
 LidarPerception perc;
 PerceptionResult result;
 for (int frame = 0; frame < 8; ++frame) {
 auto obs = make_scan(
 (frame + 1) * 100'000'000LL, 10.0f, 34, 38, 3.0f);
 obs.scan_static_valid = false;
 obs.static_ranges_m.clear();
 result = perc.update(obs, origin());
 }
 assert(result.tracks.empty());
 }

 // Unknown environment: raw returns build static occupancy, while a
 // currently associated dynamic return is kept out of the persistent map.
 {
 RayClearingMapper mapper;
 auto obs = make_scan(1'000'000, 4.0f, 36, 36, 3.0f);
 obs.scan_static_valid = false;
 obs.static_ranges_m.clear();
 DynamicTrack actor;
 actor.x_m = 3.0;
 actor.radius_m = 0.25;
 mapper.set_dynamic_tracks({actor});
 mapper.update(obs, origin());
 const LocalMap m = mapper.local_map();
 auto cell = [&](double x, double y) -> int8_t {
  const int gx = static_cast<int>(std::floor((x - m.origin_x_m) /
                                             m.resolution_m));
  const int gy = static_cast<int>(std::floor((y - m.origin_y_m) /
                                             m.resolution_m));
  return m.occupancy[static_cast<std::size_t>(gy) * m.width + gx];
 };
 assert(cell(-3.95, -0.05) == 100);
 assert(cell(3.0, 0.0) != 100);
 }

 // Once persistent local occupancy represents a stopped/novel obstacle,
 // the temporal track cannot charge the same object a second time. Unknown
 // cells do not suppress dynamic evidence.
 {
 LocalMap occupied;
 occupied.valid = true;
 occupied.resolution_m = 0.1;
 occupied.width = 20;
 occupied.height = 20;
 occupied.origin_x_m = -1.0;
 occupied.origin_y_m = -1.0;
 occupied.occupancy.assign(occupied.width * occupied.height, 0);
 occupied.occupancy[10 * occupied.width + 10] = 100;
 PerceptionResult perception;
 DynamicTrack track;
 track.x_m = 0.05;
 track.y_m = 0.05;
 track.radius_m = 0.2;
 perception.tracks.push_back(track);
 assert(reconcile_tracks_with_local_occupancy(perception, occupied)
 .tracks.empty());
 track.prediction_ready = true;
 perception.tracks = {track};
 assert(reconcile_tracks_with_local_occupancy(perception, occupied)
 .tracks.size() == 1);
 track.prediction_ready = false;
 perception.tracks = {track};
 occupied.occupancy.assign(occupied.width * occupied.height, -1);
 assert(reconcile_tracks_with_local_occupancy(perception, occupied)
 .tracks.size() == 1);
 }

  // Mapper: background paints occupied, foreground clears but never paints.
  {
    RayClearingMapper mapper;
    auto obs = make_scan(1'000'000, 4.0f, 34, 38, 3.0f);
    EstimatedState est = origin();
    est.x_m = 0.0;
    est.y_m = 0.0;
    mapper.update(obs, est);
    LocalMap m = mapper.local_map();
    assert(m.valid);
    // Background beam straight back (-x is beam 0): endpoint occupied.
    // (Interior query point: exact cell corners are FP-fragile.)
    assert(m.is_blocked(-3.95, -0.05));
    // Mid-ray of that beam is cleared free.
    assert(m.is_free(-2.0, 0.0));
    // Foreground bump at ~3 m: ray cleared, endpoint NOT painted.
 // The foreground endpoint is not painted as static, so it remains
 // unknown. Unknown space is conservative for planning.
 assert(m.is_blocked(3.0, 0.0));
    assert(m.is_free(2.0, 0.0));
    // Recenter on teleport: window follows, old content dropped.
    est.x_m = 100.0;
    mapper.update(obs, est);
    LocalMap m2 = mapper.local_map();
    assert(m2.valid);
 assert(m2.is_blocked(4.0, 0.0));
  }

  std::printf("perception ok\n");
  return 0;
}
