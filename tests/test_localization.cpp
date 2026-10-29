#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/localization/global_search.hpp"
#include "robot/autonomy/localization/localization_system.hpp"
#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/autonomy/localization/startup_selector.hpp"
#include "robot/autonomy/localization/supervisor.hpp"

namespace {
// Minimal slab raycast test oracle (independent of the sim suite).
double cast_ray(double ox, double oy, double dx, double dy,
                const std::vector<std::vector<int>>& grid, double res,
                double ox0, double oy0, double max_range) {
  double best = max_range;
  const int w = static_cast<int>(grid[0].size());
  const int h = static_cast<int>(grid.size());
  for (int gy = 0; gy < h; ++gy)
    for (int gx = 0; gx < w; ++gx) {
      if (grid[gy][gx] < 50) continue;
      const double x0 = ox0 + gx * res, x1 = ox0 + (gx + 1) * res;
      const double y0 = oy0 + gy * res, y1 = oy0 + (gy + 1) * res;
      double tmin = 0.0, tmax = max_range;
      // x slab
      if (std::abs(dx) < 1e-12) {
        if (ox < x0 || ox > x1) continue;
      } else {
        double t0 = (x0 - ox) / dx, t1 = (x1 - ox) / dx;
        if (t0 > t1) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmin > tmax) continue;
      }
      // y slab
      if (std::abs(dy) < 1e-12) {
        if (oy < y0 || oy > y1) continue;
      } else {
        double t0 = (y0 - oy) / dy, t1 = (y1 - oy) / dy;
        if (t0 > t1) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmin > tmax) continue;
      }
      if (tmin > 0.0) best = std::min(best, tmin);
    }
  return best;
}

lost_signal::robot::iface::MapSnapshot make_wall_map() {
  using namespace lost_signal::robot;
  iface::MapSnapshot m;
  m.map_id = "wall";
  m.revision = 3;
  m.width = 40;
  m.height = 40;
  m.resolution_m = 0.5;
  m.origin_x_m = -10.0;
  m.origin_y_m = -10.0;
  m.frame_id = "map";
  m.occupancy.assign(40 * 40, -1);
  // T-junction: a full-height wall plus a perpendicular stub. Corners
  // discriminate in both axes; a uniform wall alone is legitimately
  // ambiguous along its length (the matcher must reject that case).
  // T-junction plus a thin pillar: corners discriminate in both axes and
  // the pillar breaks the score plateau that makes shifted hypotheses tie
  // exactly (quantized scores need sub-search-window texture).
  // T-junction plus clutter in all quadrants: corners discriminate in both
  // axes, and scattered boxes break the score plateaus that make shifted
  // hypotheses tie exactly (quantized scores need sub-search-window texture
  // in every direction the scan sees).
  auto box = [&](double x0, double y0, double x1, double y1, double cx,
                 double cy) {
    return cx >= x0 && cx < x1 && cy >= y0 && cy < y1;
  };
  for (int gy = 0; gy < 40; ++gy)
    for (int gx = 0; gx < 40; ++gx) {
      double cx = -10.0 + (gx + 0.5) * 0.5;
      double cy = -10.0 + (gy + 0.5) * 0.5;
      bool occ = (cx >= 4.0 && cx < 5.0);                       // full wall
      occ = occ || box(4.0, -1.0, 10.0, 0.0, cx, cy);           // stub
      occ = occ || box(2.5, -3.0, 3.0, -2.5, cx, cy);           // pillar
      occ = occ || box(-2.0, 3.0, -1.0, 4.0, cx, cy);          // rear-left
      occ = occ || box(0.0, 5.0, 0.5, 6.0, cx, cy);            // ahead-left
      m.occupancy[gy * 40 + gx] = occ ? 100 : 0;
    }
  return m;
}

std::vector<float> scan_from(double x, double y, double yaw,
                             const lost_signal::robot::iface::MapSnapshot& m) {
  const int n = 72;
  std::vector<std::vector<int>> grid(m.height,
                                     std::vector<int>(m.width, 0));
  for (std::size_t i = 0; i < m.occupancy.size(); ++i)
    grid[i / m.width][i % m.width] = m.occupancy[i];
  std::vector<float> ranges;
  for (int i = 0; i < n; ++i) {
    double a = -M_PI + 2.0 * M_PI * i / n + yaw;
    ranges.push_back(
        static_cast<float>(cast_ray(x, y, std::cos(a), std::sin(a), grid,
                                    m.resolution_m, m.origin_x_m,
                                    m.origin_y_m, 10.0f)));
  }
  return ranges;
}
}  // namespace

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  // 1. Scan match converges from a perturbed prior onto the wall map.
  // The gap in the wall breaks the y-symmetry that would make the match
  // ambiguous (a uniform wall is correctly rejected, see main-stack notes).
  {
    auto map = make_wall_map();
    auto ranges = scan_from(1.0, 0.0, 0.0, map);
    ScanMatcher matcher;
    ScanMatchResult r =
        matcher.localize(ranges, -M_PI, 2.0 * M_PI / 72, 1.3, -0.2, 0.08, map, 1);
    assert(r.accepted);
    assert(std::abs(r.x_m - 1.0) < 0.3);
    assert(std::abs(r.y_m - 0.0) < 0.3);
    assert(std::abs(r.yaw_rad) < 0.15);
    assert(r.overlap > 0.3);
 assert(r.pose_information_valid);
 assert(r.pose_information[0] >= 0.0);
 assert(r.pose_information[4] >= 0.0);
 assert(r.pose_information[8] >= 0.0);
  }

  // 2. Empty map: no geometric returns -> rejected, never a fake fix.
  {
    auto map = make_wall_map();
    std::fill(map.occupancy.begin(), map.occupancy.end(), -1);
    auto ranges = scan_from(1.0, 1.0, 0.0, map);
    ScanMatcher matcher;
    ScanMatchResult r =
        matcher.localize(ranges, -M_PI, 2.0 * M_PI / 72, 1.0, 1.0, 0.0, map, 1);
    assert(!r.accepted);
  }

  // 2a. A rejected absolute innovation widens only the observed covariance
  // directions. Replaying the same correlated wall evidence is idempotent;
  // it must not turn a valid localized estimate into an unbounded goal hold.
  {
    FusedOdometryEstimator estimator;
    estimator.reset();
    estimator.seed_pose(2.0, -1.0, 0.3, 0.05, 0.02, 1);
    const double x_before = estimator.estimate().x_m;
    const double y_before = estimator.estimate().y_m;
    const double yaw_before = estimator.estimate().yaw_rad;
    const auto odom_before = estimator.odometry_estimate();

    estimator.note_absolute_rejection(1.0, 2.0, 0.4, 2);
    const double sigma_after_first = estimator.estimate().position_sigma_m;
    const double heading_after_first = estimator.estimate().heading_sigma_rad;
    assert(std::abs(sigma_after_first - 2.0) < 1e-12);
    assert(std::abs(heading_after_first - 0.4) < 1e-12);
    assert(std::abs(estimator.estimate().x_m - x_before) < 1e-12);
    assert(std::abs(estimator.estimate().y_m - y_before) < 1e-12);
    assert(std::abs(estimator.estimate().yaw_rad - yaw_before) < 1e-12);

    estimator.note_absolute_rejection(1.0, 2.0, 0.4, 3);
    assert(std::abs(estimator.estimate().position_sigma_m -
                    sigma_after_first) < 1e-12);
    assert(std::abs(estimator.estimate().heading_sigma_rad -
                    heading_after_first) < 1e-12);
    assert(std::abs(estimator.odometry_estimate().x_m - odom_before.x_m) <
           1e-12);
    assert(std::abs(estimator.odometry_estimate().y_m - odom_before.y_m) <
           1e-12);
    assert(std::abs(estimator.odometry_estimate().yaw_rad - odom_before.yaw_rad) <
           1e-12);

    estimator.note_absolute_rejection(3.0, 0.5, 0.1, 4);
    assert(estimator.estimate().position_sigma_m > sigma_after_first);
    assert(std::abs(estimator.estimate().heading_sigma_rad -
                    heading_after_first) < 1e-12);

    const double sigma_before_search_failure =
        estimator.estimate().position_sigma_m;
    estimator.note_absolute_search_failure(4.0, 0.8, 5);
    assert(estimator.estimate().position_sigma_m >= 4.0);
    assert(estimator.estimate().heading_sigma_rad >= 0.8);
    estimator.note_absolute_search_failure(4.0, 0.8, 6);
    assert(std::abs(estimator.estimate().position_sigma_m - 4.0) < 1e-12);
    assert(std::abs(estimator.estimate().heading_sigma_rad - 0.8) < 1e-12);
    assert(estimator.estimate().position_sigma_m >= sigma_before_search_failure);
  }

  // 2b. Uniform wall alone is legitimately ambiguous along its length:
  // shifted hypotheses score identically, so the matcher must refuse
  // rather than drag the estimate to a corridor twin.
  {
    auto map = make_wall_map();
    for (std::size_t i = 0; i < map.occupancy.size(); ++i) {
      int gx = i % 40, gy = i / 40;
      double cx = -10.0 + (gx + 0.5) * 0.5;
      double cy = -10.0 + (gy + 0.5) * 0.5;
      bool occ = (cx >= 4.0 && cx < 5.0);
      map.occupancy[i] = occ ? 100 : 0;
    }
    auto ranges = scan_from(1.0, 0.0, 0.0, map);
    ScanMatcher matcher;
    ScanMatchResult r =
        matcher.localize(ranges, -M_PI, 2.0 * M_PI / 72, 1.1, 0.1, 0.0, map, 1);
    assert(!r.accepted);
 assert(r.rejection_reason == "map match is geometrically ambiguous");
 assert(r.partial_observation_valid);
 assert(r.pose_information_valid);
 assert(r.pose_information[0] > 0.0);
 // The vertical wall observes wall-normal x and heading, but supplies no
 // position information along y. Information-form fusion may use x without
 // selecting one of the tied y poses.
 assert(r.pose_information[4] < 1e-9);
 FusedOdometryEstimator corridor_estimate;
 corridor_estimate.reset();
 corridor_estimate.seed_pose(1.1, 0.4, 0.0, 0.5, 0.2, 1);
 corridor_estimate.correct_pose_information(
 r.x_m, r.y_m, r.yaw_rad, r.pose_information,
 r.position_sigma_m, r.heading_sigma_rad, 2);
 assert(std::abs(corridor_estimate.estimate().x_m - 1.0) < 0.1);
 assert(std::abs(corridor_estimate.estimate().y_m - 0.4) < 1e-9);

 // The localization owner must consume the same partial observation without
 // relabelling it as a complete pose fix or selecting a corridor twin.
 LocalizationSystem localization;
 localization.mark_localized();
 FusedOdometryEstimator estimator;
 estimator.reset();
 estimator.seed_pose(1.3, 0.4, 0.0, 0.2, 0.1, 1);
 RayClearingMapper mapper;
 iface::ObservationFrame observation;
 observation.sequence = 1;
 observation.stamp_ns = 20'000'000;
 observation.source = "t.corridor-lidar";
 observation.frame = core::Frame::kLidarLink;
 observation.odometry_valid = true;
 observation.scan_valid = true;
 observation.ranges_m = ranges;
 observation.scan_angle_min_rad = -M_PI;
 observation.scan_angle_increment_rad = 2.0 * M_PI / ranges.size();
 const auto partial = localization.update(
 map, observation, observation.stamp_ns, true, estimator, mapper);
 assert(partial.action == "local_correction:partial observable ridge");
 assert(std::abs(estimator.estimate().x_m - 1.0) < 0.3);
 assert(std::abs(estimator.estimate().y_m - 0.4) < 1e-9);

 // A partial ridge is still a measurement in its observable directions.
 // A separated wall may look locally identical, but its wall-normal
 // innovation must pass the normalized information gate before it can move
 // map->odom. Motion along the null corridor axis remains irrelevant.
 LocalizationSystem separated_localization;
 separated_localization.mark_localized();
 FusedOdometryEstimator separated_estimator;
 separated_estimator.reset();
 separated_estimator.seed_pose(2.0, 0.4, 0.0, 0.4, 0.1, 1);
 const double separated_sigma_before =
     separated_estimator.estimate().position_sigma_m;
 const double separated_heading_before =
     separated_estimator.estimate().heading_sigma_rad;
 RayClearingMapper separated_mapper;
 const auto separated = separated_localization.update(
 map, observation, observation.stamp_ns, true,
 separated_estimator, separated_mapper);
 assert(separated.action ==
 "local_correction:partial innovation rejected");
 assert(std::abs(separated_estimator.estimate().x_m - 2.0) < 1e-9);
 assert(std::abs(separated_estimator.estimate().y_m - 0.4) < 1e-9);
 // A rejected ridge is weak negative evidence. It neither moves the pose nor
 // widens the covariance: its residual along the unobservable corridor axis
 // is arbitrary, and the local search window grows with covariance.
 assert(std::abs(separated_estimator.estimate().position_sigma_m -
                 separated_sigma_before) < 1e-12);
 assert(std::abs(separated_estimator.estimate().heading_sigma_rad -
                 separated_heading_before) < 1e-12);
 // Replaying the same rejected ridge must leave the belief where it was. The
 // supervisor may only start whole-map recovery when covariance itself
 // crosses the loss envelope, never because rejections accumulated: a
 // feedback of rejected residuals into sigma ratchets a long corridor past
 // that envelope and lets recovery promote an aliased pose.
 for (int replay_index = 1; replay_index <= 4; ++replay_index) {
   auto replay = observation;
   replay.sequence = observation.sequence + replay_index;
   replay.stamp_ns = observation.stamp_ns + replay_index * 6'000'000'000LL;
   const auto again = separated_localization.update(
       map, replay, replay.stamp_ns, true, separated_estimator,
       separated_mapper);
   assert(again.action == "local_correction:partial innovation rejected");
   assert(std::abs(separated_estimator.estimate().x_m - 2.0) < 1e-9);
   assert(std::abs(separated_estimator.estimate().y_m - 0.4) < 1e-9);
   assert(std::abs(separated_estimator.estimate().position_sigma_m -
                   separated_sigma_before) < 1e-12);
   assert(std::abs(separated_estimator.estimate().heading_sigma_rad -
                   separated_heading_before) < 1e-12);
 }

 // A decisive global candidate entering temporal recovery must survive the
 // same wall ridge. Later scans validate the observable wall normal and yaw;
 // measured odometry owns motion along the unobservable corridor axis.
 StartupTrackerConfig recovery_config;
 recovery_config.promotion_translation_m = 0.01;
 StartupHypotheses recovery(recovery_config);
 GlobalSearchResult candidate;
 candidate.x_m = 1.1;
 candidate.y_m = 0.4;
 candidate.position_sigma_m = 0.12;
 candidate.heading_sigma_rad = 0.04;
 candidate.score = 0.8;
 recovery.seed(candidate, 0, /*require_motion=*/true);
 const double increment = 2.0 * M_PI / ranges.size();
 const auto first_support = recovery.update(
 ranges, -M_PI, increment, map, 20'000'000, 0.0, 0.0, true);
 assert(first_support.status == StartupTrackStatus::kTracking);
 const auto moved_ranges = scan_from(1.02, 0.0, 0.0, map);
 ScanMatcherConfig tracker_match_config;
 tracker_match_config.search_translation_m = recovery_config.match_translation_m;
 tracker_match_config.translation_step_m = recovery_config.match_translation_step_m;
 tracker_match_config.search_heading_rad = recovery_config.match_heading_rad;
 tracker_match_config.heading_step_rad = recovery_config.match_heading_step_rad;
 tracker_match_config.maximum_returns = recovery_config.match_max_returns;
 ScanMatcher tracker_matcher(tracker_match_config);
 const auto direct_match = tracker_matcher.localize(
 moved_ranges, -M_PI, increment, 1.02, 0.4, 0.0, map, 40'000'000);
 assert(!direct_match.accepted);
 assert(direct_match.partial_observation_valid);
 assert(direct_match.rejection_reason ==
 "map match is geometrically ambiguous");
 const auto confirmed = recovery.update(
 moved_ranges, -M_PI, increment, map, 40'000'000, 1.0, 0.0, true);
 assert(confirmed.status == StartupTrackStatus::kPromoted);
 assert(std::abs(confirmed.x_m - 1.02) < 0.15);
 assert(std::abs(confirmed.y_m - 0.4) < 1e-9);

 // Recovery promotion may require full-rank evidence. A wall-only scan can
 // keep tracking the observed normal, but it must not turn two partial
 // confirmations into an absolute map pose.
 StartupTrackerConfig guarded_config = recovery_config;
 guarded_config.require_full_observation_for_promotion = true;
 StartupHypotheses guarded_recovery(guarded_config);
 guarded_recovery.seed(candidate, 0, /*require_motion=*/true);
 const auto guarded_first = guarded_recovery.update(
 ranges, -M_PI, increment, map, 20'000'000, 0.0, 0.0, true);
 const auto guarded_second = guarded_recovery.update(
 moved_ranges, -M_PI, increment, map, 40'000'000, 1.0, 0.0, true);
 assert(guarded_first.status == StartupTrackStatus::kTracking);
 assert(guarded_second.status == StartupTrackStatus::kTracking);
 assert(!guarded_second.full_observation_confirmed);
 assert(guarded_recovery.size() == 1);

 // Once the same tracker receives a geometrically complete observation, the
 // existing confirmation and motion requirements are sufficient to promote.
 auto full_map = make_wall_map();
 const auto full_ranges = scan_from(1.0, 0.0, 0.0, full_map);
 StartupTrackerConfig full_config = guarded_config;
 StartupHypotheses full_recovery(full_config);
 GlobalSearchResult full_candidate;
 full_candidate.x_m = 1.3;
 full_candidate.y_m = -0.2;
 full_candidate.yaw_rad = 0.08;
 full_candidate.position_sigma_m = 0.12;
 full_candidate.heading_sigma_rad = 0.04;
 full_candidate.runner_ups.push_back({50.0, 50.0, 0.0, 0.1});
 full_recovery.seed(full_candidate, 0, /*require_motion=*/false);
 StartupTrackResult full_result;
 for (int k = 1; k <= 4; ++k) {
 full_result = full_recovery.update(
 full_ranges, -M_PI, increment, full_map,
 static_cast<lost_signal::robot::core::MonotonicNs>(k + 1) * 20'000'000LL,
 0.0, 0.0, true);
 if (full_result.status == StartupTrackStatus::kPromoted) break;
 }
 assert(full_result.status == StartupTrackStatus::kPromoted);
 assert(full_result.full_observation_confirmed);
 assert(std::abs(full_result.x_m - 1.0) < 0.6);
  }

 // 2c. A local search whose best score still improves beyond its bounded
 // window is clipped, not converged. It must refuse the correction so a
 // high-covariance supervisor can retain the wider global recovery instead
 // of walking the map pose through repeated window-edge updates.
 {
 auto map = make_wall_map();
 auto ranges = scan_from(1.0, 0.0, 0.0, map);
 ScanMatcherConfig config;
 config.search_translation_m = 0.50;
 config.translation_step_m = 0.25;
 ScanMatcher matcher(config);
 const ScanMatchResult r = matcher.localize(
 ranges, -M_PI, 2.0 * M_PI / 72,
 1.75, 0.0, 0.0, map, 1);
 assert(!r.accepted);
 assert(r.rejection_reason == "map optimum reaches search boundary");
 }

  // 3. correct_pose blends by variance and shrinks sigma.
  {
    FusedOdometryEstimator est;
    est.reset();
    iface::ObservationFrame o;
    o.sequence = 1;
    o.stamp_ns = 0;
    o.source = "t.odom";
    o.frame = core::Frame::kBaseLink;
    o.odometry_valid = true;
    o.forward_mps = 1.0;
    est.predict(o);
    for (int k = 2; k <= 6; ++k) {
      o.sequence = static_cast<std::uint64_t>(k);
      o.stamp_ns = static_cast<core::MonotonicNs>(k - 1) * 20'000'000LL;
      est.predict(o);
    }
    const double drifted_x = est.estimate().x_m;
    assert(std::abs(drifted_x - 0.1) < 1e-9);
 const double continuous_x = est.odometry_estimate().x_m;
 const double continuous_sigma = est.odometry_estimate().position_sigma_m;
    const double before = est.estimate().position_sigma_m;
    assert(before > 0.05);
    est.correct_pose(0.02, 0.0, 0.0, 0.02, 0.01, 2'000'000);
    assert(est.estimate().position_sigma_m < before);
 assert(est.odometry_estimate().x_m == continuous_x);
 assert(est.odometry_estimate().position_sigma_m == continuous_sigma);
    // Blended, not snapped: the estimate moves toward the correction
    // proportionally to relative certainty, never jumps to it.
    assert(est.estimate().x_m > 0.025 && est.estimate().x_m < 0.09);
    // Malformed corrections are ignored, never applied.
    const double x0 = est.estimate().x_m;
    est.correct_pose(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.02,
                     0.01, 3'000'000);
    assert(est.estimate().x_m == x0);

 // Prediction continues in both frames without erasing the persistent map
 // correction. Odom remains continuous while map_T_odom carries the offset.
 const double map_odom_offset =
 est.estimate().x_m - est.odometry_estimate().x_m;
 o.sequence = 7;
 o.stamp_ns = 120'000'000;
 est.predict(o);
 assert(std::abs(est.odometry_estimate().x_m - 0.12) < 1e-9);
 assert(std::abs((est.estimate().x_m - est.odometry_estimate().x_m) -
 map_odom_offset) < 1e-9);
  }

 // 3a. A rank-deficient scan update reduces only observed directions. A
 // strong x/yaw measurement with zero y information must not collapse the
 // public radial sigma while the corridor axis remains uncertain.
 {
 FusedOdometryEstimator est;
 est.reset();
 est.seed_pose(0.0, 0.0, 0.0, 1.0, 0.5, 1);
 const std::array<double, 9> information{
 400.0, 0.0, 0.0,
 0.0, 0.0, 0.0,
 0.0, 0.0, 400.0};
 est.correct_pose_information(
 1.0, 2.0, 0.1, information, 0.05, 0.05, 2);
 assert(est.estimate().x_m > 0.9);
 assert(std::abs(est.estimate().y_m) < 1e-12);
 assert(est.estimate().yaw_rad > 0.09);
 assert(est.estimate().position_sigma_m > 0.99);
 }

 // 3b. A scan-to-submap fix is relative to a drifted keyframe chain. It may
 // correct incremental pose error, but it must not manufacture absolute
 // map-frame confidence or postpone global recovery indefinitely.
 {
 FusedOdometryEstimator est;
 est.reset();
 est.seed_pose(1.0, 2.0, 0.2, 0.40, 0.20, 1);
 const double position_sigma = est.estimate().position_sigma_m;
 const double heading_sigma = est.estimate().heading_sigma_rad;
 est.correct_relative_pose(0.8, 2.1, 0.1, 0.05, 0.02, 2);
 assert(est.estimate().x_m < 1.0);
 assert(est.estimate().y_m > 2.0);
 assert(est.estimate().yaw_rad < 0.2);
 assert(est.estimate().position_sigma_m == position_sigma);
 assert(est.estimate().heading_sigma_rad == heading_sigma);
 assert(est.odometry_estimate().x_m == 0.0);
 assert(est.odometry_estimate().y_m == 0.0);
 assert(est.odometry_estimate().yaw_rad == 0.0);
 }

 // 3c. Heading uncertainty projects into cross-track position uncertainty as
 // the robot moves. A scalar position sigma that ignored this Jacobian could
 // remain centimetric while a small yaw bias accumulated metres of drift.
 {
 FusedOdometryEstimator::Config config;
 config.position_process_variance_per_m = 0.0;
 config.position_process_variance_per_s = 0.0;
 config.heading_process_variance_per_s = 0.0;
 FusedOdometryEstimator est(config);
 est.reset();
 est.seed_pose(0.0, 0.0, 0.0, 0.05, 0.20, 1);
 iface::ObservationFrame motion;
 motion.sequence = 1;
 motion.stamp_ns = 500'000'001;
 motion.source = "t.heading-coupling";
 motion.frame = core::Frame::kBaseLink;
 motion.odometry_valid = true;
 motion.forward_mps = 2.0;
 est.predict(motion);
 assert(std::abs(est.estimate().x_m - 1.0) < 1e-9);
 assert(est.estimate().position_sigma_m > 0.20);
 assert(est.estimate().position_sigma_m < 0.21);
 }

  // 4. Supervisor: unknown start -> global attempt; accept -> localized;
 // cadence gates local corrections; rejected local matches alone do not
 // trigger a whole-map search while fused covariance remains bounded.
  {
    LocalizationSupervisor sup;
    sup.reset(false);
    LocalizationHealth h;
    std::string reason;
    assert(sup.next(0.0, 0.0, 0.0, 0.0, h, &reason) ==
           LocalizationAction::kGlobalRelocalization);
    ScanMatchResult ok;
    ok.accepted = true;
    sup.record_global(true, ok);
    assert(sup.localized());
    assert(sup.next(0.1, 0.0, 0.0, 0.0, h) == LocalizationAction::kDefer);
    assert(sup.next(1.0, 2.0, 0.0, 0.0, h, &reason) ==
           LocalizationAction::kLocalCorrection);
    ScanMatchResult bad;
    sup.record_local(false, bad);
    sup.record_local(false, bad);
    sup.record_local(false, bad);
 assert(sup.next(10.0, 2.0, 0.0, 0.0, h) ==
 LocalizationAction::kLocalCorrection);
 h.position_sigma_m = 0.5;
 assert(sup.next(12.1, 2.0, 0.0, 0.0, h) ==
 LocalizationAction::kGlobalRelocalization);
  }

  // 5. Coordinator wiring: seeded start localizes on the first match,
  // corrections land in the running estimate, supervisor reports localized.
  {
    auto map = make_wall_map();
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    coord.seed_pose(1.0, 0.0, 0.0);  // known dock, like operator initialpose
    for (int k = 1; k <= 6; ++k) {
      iface::ObservationFrame o;
      o.sequence = static_cast<std::uint64_t>(k);
    o.stamp_ns = static_cast<core::MonotonicNs>(k) * 20'000'000LL;
      o.source = "t.lidar";
      o.frame = core::Frame::kLidarLink;
      o.odometry_valid = true;
      o.forward_mps = 0.0;  // holding: corrections, not motion, move sigma
      o.yaw_rate_radps = 0.0;
      o.scan_valid = true;
      o.ranges_m = scan_from(1.0, 0.0, 0.0, map);
      o.scan_angle_min_rad = -M_PI;
      o.scan_angle_increment_rad = 2.0 * M_PI / o.ranges_m.size();
      o.imu_valid = true;
      coord.ingest_observation(o);
      coord.update(map, iface::ChannelHealth{}, o.stamp_ns);
    }
    assert(coord.localized());
    assert(coord.estimate().position_sigma_m < 0.2);
    assert(std::abs(coord.estimate().x_m - 1.0) < 0.5);
  }

  // 6. Global search: kidnap recovery from an unknown start. The true
  // pose is (1,0,0) but no prior is given; the whole-map search must
  // accept near the true pose. A uniform-wall map stays ambiguous and
  // must reject rather than guess.
  {
    auto map = make_wall_map();
    auto ranges = scan_from(1.0, 0.0, 0.0, map);
    GlobalSearcher searcher;
    GlobalSearchResult r = searcher.search(
        ranges, -M_PI, 2.0 * M_PI / ranges.size(), map, 1);
    assert(r.accepted);
    assert(std::abs(r.x_m - 1.0) < 0.5);
    assert(std::abs(r.y_m - 0.0) < 0.5);
    assert(std::abs(r.yaw_rad) < 0.2);
    assert(r.coarse_evaluations > 0);
 // Cooperative execution visits the same candidates in the same order and
 // therefore returns the bit-identical decision without one long tick.
 searcher.begin(ranges, -M_PI, 2.0 * M_PI / ranges.size(), map, 1);
 std::optional<GlobalSearchResult> incremental;
 int slices = 0;
 while (!incremental.has_value() && slices < 4000) {
 incremental = searcher.advance(256);
 ++slices;
 }
 assert(incremental.has_value() && slices > 1);
 assert(incremental->accepted == r.accepted);
 assert(incremental->x_m == r.x_m && incremental->y_m == r.y_m);
 assert(incremental->yaw_rad == r.yaw_rad);
 assert(incremental->score == r.score);
 assert(incremental->coarse_evaluations == r.coarse_evaluations);
 assert(incremental->coarse_positions_considered ==
 r.coarse_positions_considered);
 assert(incremental->coarse_positions_skipped_unobservable ==
 r.coarse_positions_skipped_unobservable);
 // Once a map frame exists, recovery uses its covariance-supported region
 // rather than allowing a repeated structure elsewhere in the survey to
 // compete with the prior. It must retain the same true local solution with
 // fewer coarse evaluations.
 searcher.begin_near(ranges, -M_PI, 2.0 * M_PI / ranges.size(), map, 1,
 1.0, 0.0, 2.0, 0.0, 0.5);
 std::optional<GlobalSearchResult> regional;
 while (!regional.has_value()) regional = searcher.advance(256);
 assert(regional->accepted);
 assert(std::abs(regional->x_m - 1.0) < 0.5);
 assert(std::abs(regional->y_m) < 0.5);
 assert(regional->coarse_evaluations < r.coarse_evaluations);

 // A sparse prior may contain occupied returns but no surveyed free cells.
 // Unknown cells remain eligible seeds; they are neutral until the measured
 // returns, overlap, margin, and final raycast gates validate the pose.
 auto sparse = make_wall_map();
 for (auto& cell : sparse.occupancy) {
 if (cell < 50) cell = -1;
 }
 auto sparse_ranges = scan_from(1.0, 0.0, 0.0, sparse);
 GlobalSearchResult sr = searcher.search(
 sparse_ranges, -M_PI, 2.0 * M_PI / sparse_ranges.size(), sparse, 1);
 assert(sr.accepted);
 assert(std::abs(sr.x_m - 1.0) < 0.6);

    // Uniform wall: every y-shifted hypothesis ties, so the global
    // search must refuse instead of kidnapping the estimate to a twin.
    auto uniform = make_wall_map();
    for (std::size_t i = 0; i < uniform.occupancy.size(); ++i) {
      int gx = i % 40, gy = i / 40;
      double cx = -10.0 + (gx + 0.5) * 0.5;
      double cy = -10.0 + (gy + 0.5) * 0.5;
      bool occ = (cx >= 4.0 && cx < 5.0);
      uniform.occupancy[i] = occ ? 100 : 0;
    }
    auto uranges = scan_from(1.0, 0.0, 0.0, uniform);
    GlobalSearchResult u = searcher.search(
        uranges, -M_PI, 2.0 * M_PI / uranges.size(), uniform, 1);
    assert(!u.accepted);

 // Exact observability pruning keeps a mostly empty large raster out of the
 // expensive heading/beam pass. Only base positions within lidar reach of
 // the single mapped obstacle can explain a hit; all others are counted as
 // skipped diagnostics, never scored as partial search results.
 iface::MapSnapshot empty_extent;
 empty_extent.map_id = "large-empty-extent";
 empty_extent.revision = 1;
 empty_extent.width = 100;
 empty_extent.height = 100;
 empty_extent.resolution_m = 1.0;
 empty_extent.origin_x_m = 0.0;
 empty_extent.origin_y_m = 0.0;
 empty_extent.occupancy.assign(100 * 100, -1);
 empty_extent.occupancy[50 * 100 + 50] = 100;
 GlobalSearchConfig bounded_config;
 bounded_config.max_range_m = 2.0;
 bounded_config.coarse_position_step_m = 1.0;
 bounded_config.coarse_heading_step_rad = M_PI_2;
 bounded_config.refine_translation_m = 0.0;
 bounded_config.refine_heading_rad = 0.0;
 bounded_config.final_translation_m = 0.0;
 bounded_config.final_heading_rad = 0.0;
 GlobalSearcher bounded_searcher(bounded_config);
 std::vector<float> short_hits(72, 1.0F);
 GlobalSearchResult bounded = bounded_searcher.search(
 short_hits, -M_PI, 2.0 * M_PI / short_hits.size(), empty_extent, 1);
 assert(bounded.coarse_positions_considered == 24);
 assert(bounded.coarse_positions_skipped_unobservable == 9975);
 assert(bounded.coarse_evaluations ==
 bounded.coarse_positions_considered * 4);
 const GlobalSearchResult bounded_reused = bounded_searcher.search(
 short_hits, -M_PI, 2.0 * M_PI / short_hits.size(), empty_extent, 2);
 assert(bounded_reused.map_preparation_cells == 0);

 // Lifetime work is not a correctness gate. This deliberately evaluates
 // more than the removed legacy 8M ceiling, one bounded slice at a time.
 // A large or high-resolution search may take more ticks, but it must finish
 // the same deterministic candidate set instead of reporting a false
 // localization failure.
 iface::MapSnapshot long_search_map;
 long_search_map.map_id = "cooperative-over-eight-million";
 long_search_map.revision = 1;
 long_search_map.width = 23;
 long_search_map.height = 23;
 long_search_map.resolution_m = 1.0;
 long_search_map.occupancy.assign(23 * 23, 0);
 for (std::uint32_t y = 0; y < long_search_map.height; ++y) {
 for (std::uint32_t x = 0; x < long_search_map.width; ++x) {
 if (x == 0 || y == 0 || x + 1 == long_search_map.width ||
 y + 1 == long_search_map.height) {
 long_search_map.occupancy[y * long_search_map.width + x] = 100;
 }
 }
 }
 GlobalSearchConfig long_config;
 long_config.max_range_m = 100.0;
 long_config.minimum_returns = 1;
 long_config.coarse_beams = 1;
 long_config.coarse_hypotheses = 1;
 long_config.coarse_hypothesis_distance_m = 100.0;
 long_config.coarse_hypothesis_heading_rad = 2.0 * M_PI;
 long_config.coarse_heading_step_rad = 2.0 * M_PI / 18150.0;
 long_config.refine_translation_m = 0.0;
 long_config.refine_heading_rad = 0.0;
 long_config.final_translation_m = 0.0;
 long_config.final_heading_rad = 0.0;
 long_config.final_max_returns = 1;
 GlobalSearcher long_searcher(long_config);
 const std::vector<float> one_return{1.0F};
 long_searcher.begin(one_return, 0.0, 1.0, long_search_map, 4);
 assert(long_searcher.active_map_matches(long_search_map));
 iface::MapSnapshot replacement_map = long_search_map;
 assert(!long_searcher.active_map_matches(replacement_map));
 ++long_search_map.revision;
 assert(!long_searcher.active_map_matches(long_search_map));
 --long_search_map.revision;
 assert(!long_searcher.advance(1).has_value());
 std::optional<GlobalSearchResult> long_result;
 while (!long_result.has_value()) long_result = long_searcher.advance(65536);
 assert(long_result->coarse_evaluations > 8000000);
 assert(long_result->rejection_reason != "evaluation budget exceeded");

 // Sparse scans fail before allocating or preparing a map-sized index.
 std::vector<float> no_hits(72, 2.0F);
 GlobalSearchResult insufficient = bounded_searcher.search(
 no_hits, -M_PI, 2.0 * M_PI / no_hits.size(), empty_extent, 1);
 assert(!insufficient.accepted);
 assert(insufficient.rejection_reason == "insufficient geometric returns");
 assert(insufficient.coarse_evaluations == 0);

 // The expensive map-only index is built once per immutable revision. A
 // retry with a different scan reuses it, while a revision change rebuilds
 // it before any candidate can be scored.
 GlobalSearcher cached_searcher(bounded_config);
 const auto run_cached = [&](const iface::MapSnapshot& revision,
 core::MonotonicNs stamp_ns) {
 cached_searcher.begin(short_hits, -M_PI,
 2.0 * M_PI / short_hits.size(), revision, stamp_ns);
 std::optional<GlobalSearchResult> result;
 for (int slice = 0; slice < 1000 && !result.has_value(); ++slice) {
 result = cached_searcher.advance(4096);
 }
 assert(result.has_value());
 return *result;
 };
 const auto first_revision_search = run_cached(empty_extent, 2);
 assert(first_revision_search.map_preparation_cells ==
 2 * empty_extent.occupancy.size());
 const auto reused_revision_search = run_cached(empty_extent, 3);
 assert(reused_revision_search.map_preparation_cells == 0);
 // Named providers promise immutable content for one id/revision. Geometry
 // is checked defensively as well: a malformed fixture that reuses the key
 // with a different allocation must never read the old index. Occupancy-only
 // edits must increment revision under IMapProvider's contract.
 auto malformed_reshaped_extent = empty_extent;
 malformed_reshaped_extent.width += 1;
 malformed_reshaped_extent.occupancy.assign(
 static_cast<std::size_t>(malformed_reshaped_extent.width) *
 malformed_reshaped_extent.height,
 -1);
 malformed_reshaped_extent.occupancy[
 50 * malformed_reshaped_extent.width + 50] = 100;
 const auto reshaped_search = run_cached(malformed_reshaped_extent, 4);
 assert(reshaped_search.map_preparation_cells ==
 2 * malformed_reshaped_extent.occupancy.size());
 auto changed_extent = empty_extent;
 ++changed_extent.revision;
 const auto changed_revision_search = run_cached(changed_extent, 5);
 assert(changed_revision_search.map_preparation_cells ==
 2 * changed_extent.occupancy.size());
 auto anonymous_extent = empty_extent;
 anonymous_extent.map_id.clear();
 const auto anonymous_first = run_cached(anonymous_extent, 6);
 const auto anonymous_second = run_cached(anonymous_extent, 7);
 assert(anonymous_first.map_preparation_cells ==
 2 * anonymous_extent.occupancy.size());
 assert(anonymous_second.map_preparation_cells ==
 2 * anonymous_extent.occupancy.size());
  }

  // 7. Coordinator kidnap path: unlocalized start takes the global branch
  // and lands localized near the true pose.
  {
    auto map = make_wall_map();
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    // No seed_pose: the supervisor starts unlocalized and the first tick
 // begins the whole-map search. Work is spread over bounded updates.
    iface::ObservationFrame o;
    o.sequence = 1;
    o.stamp_ns = 20'000'000;
    o.source = "t.lidar";
    o.frame = core::Frame::kLidarLink;
    o.odometry_valid = true;
    o.scan_valid = true;
    o.ranges_m = scan_from(1.0, 0.0, 0.0, map);
    o.scan_angle_min_rad = -M_PI;
    o.scan_angle_increment_rad = 2.0 * M_PI / o.ranges_m.size();
    o.imu_valid = true;
 for (int i = 0; i < 1000 && !coord.localized(); ++i) {
 o.sequence = static_cast<std::uint64_t>(i + 1);
    o.stamp_ns = static_cast<core::MonotonicNs>(i + 1) * 20'000'000LL;
 coord.ingest_observation(o);
 coord.update(map, iface::ChannelHealth{}, o.stamp_ns);
 }
    assert(coord.localized());
    assert(std::abs(coord.estimate().x_m - 1.0) < 0.6);
    assert(std::abs(coord.estimate().y_m - 0.0) < 0.6);
  }

 // A cooperative global search owns a frozen scan pose. Carrying its result
 // to completion must use continuous REP-105 odometry, because an unrelated
 // map-frame correction may move estimate() while the search is in flight.
 // Reusing that map jump as physical motion would apply the correction twice.
 {
 auto map = make_wall_map();
 LocalizationSystem localization;
 FusedOdometryEstimator estimator;
 estimator.reset();
 estimator.seed_pose(0.0, 0.0, 0.0, 0.20, 0.10, 1);
 RayClearingMapper mapper;
 iface::ObservationFrame scan;
 scan.sequence = 1;
 scan.stamp_ns = 20'000'000;
 scan.source = "t.frozen-search-lidar";
 scan.frame = core::Frame::kLidarLink;
 scan.scan_valid = true;
 scan.ranges_m = scan_from(1.0, 0.0, 0.0, map);
 scan.scan_angle_min_rad = -M_PI;
 scan.scan_angle_increment_rad = 2.0 * M_PI / scan.ranges_m.size();
 const auto started = localization.update(
 map, scan, scan.stamp_ns, true, estimator, mapper);
 assert(started.action == "global_relocalization:search pending");
 const EstimatedState odom_before = estimator.odometry_estimate();
 estimator.correct_pose(1.0, 0.0, 0.0, 0.01, 0.01, scan.stamp_ns);
 assert(estimator.estimate().x_m > 0.9);
 assert(std::abs(estimator.odometry_estimate().x_m - odom_before.x_m) < 1e-12);
 for (int i = 0; i < 1000 && !localization.localized(); ++i) {
 iface::ObservationFrame no_scan;
 no_scan.sequence = static_cast<std::uint64_t>(i + 2);
 no_scan.stamp_ns = static_cast<core::MonotonicNs>(i + 2) * 20'000'000LL;
 localization.update(
 map, no_scan, no_scan.stamp_ns, true, estimator, mapper);
 }
 assert(localization.localized());
 assert(std::abs(estimator.estimate().x_m - 1.0) < 0.6);
 }

 // Recovery is a stated probability gate over the joint (x, y, yaw) pose.
 // Search and acceptance use the same chi-square-derived ellipsoid instead
 // of separate tuned sigma multipliers, including the circular yaw error.
 {
 LocalizationSupervisorConfig config;
 config.minimum_global_recovery_radius_m = 0.75;
 config.minimum_global_recovery_heading_rad = 0.35;
 config.global_recovery_probability = 0.9995;
 LocalizationSupervisor supervisor(config);
 const double radius = supervisor.global_recovery_radius_m(1.0);
 const double heading_radius =
 supervisor.global_recovery_heading_radius_rad(1.0);
 assert(radius > 4.0 && radius < 5.0);
 assert(heading_radius == M_PI);
 assert(supervisor.accept_global_innovation(0.5 * radius, 0.0, 1.0, 1.0));
 assert(!supervisor.accept_global_innovation(
 std::nextafter(radius, std::numeric_limits<double>::infinity()),
 0.0, 1.0, 1.0));
 assert(!supervisor.accept_global_innovation(
 0.0, 0.36, 0.01, 0.01));
 assert(supervisor.global_recovery_radius_m(0.01) == 0.75);
 assert(supervisor.global_recovery_heading_radius_rad(0.01) == 0.35);
 }

 // An established map frame never searches beyond the region supported by
 // its covariance. A geometrically strong scan from a distant place must
 // complete as a rejection without teleporting the trusted prior.
 {
 auto map = make_wall_map();
 auto ranges = scan_from(1.0, 0.0, 0.0, map);
 LocalizationSystem localization;
 localization.mark_localized();
 FusedOdometryEstimator estimator;
 estimator.reset();
 estimator.seed_pose(-8.0, -8.0, 0.0, 0.50, 0.10, 1);
 RayClearingMapper mapper;
 // An asynchronous control tick without a new lidar frame must not consume
 // the global-retry opportunity. The next fresh scan starts recovery
 // immediately instead of observing an artificial backoff.
 iface::ObservationFrame no_scan;
 no_scan.sequence = 1;
 no_scan.stamp_ns = 20'000'000;
 no_scan.odometry_valid = true;
 const auto no_scan_result = localization.update(
 map, no_scan, no_scan.stamp_ns, true, estimator, mapper);
 assert(no_scan_result.action == "defer");

 bool bounded_recovery_completed = false;
 for (int i = 0; i < 1000 && !bounded_recovery_completed; ++i) {
 iface::ObservationFrame o;
o.sequence = static_cast<std::uint64_t>(i + 2);
o.stamp_ns = static_cast<core::MonotonicNs>(i + 2) * 20'000'000LL;
 o.source = "t.recovery-lidar";
 o.frame = core::Frame::kLidarLink;
 o.odometry_valid = true;
 o.scan_valid = true;
 o.ranges_m = ranges;
 o.scan_angle_min_rad = -M_PI;
 o.scan_angle_increment_rad = 2.0 * M_PI / o.ranges_m.size();
 const auto result = localization.update(
 map, o, o.stamp_ns, true, estimator, mapper);
 if (i == 0) {
 assert(result.action == "global_relocalization:search pending");
 }
 bounded_recovery_completed =
 result.action.rfind("global_relocalization:", 0) == 0 &&
 result.action != "global_relocalization:search pending";
 }
 assert(bounded_recovery_completed);
 assert(estimator.estimate().x_m == -8.0);
 assert(estimator.estimate().y_m == -8.0);
 }

 // A cooperative whole-map search freezes its first scan while odometry
 // continues. The accepted pose must be advanced by the intervening ROBOT
 // motion; applying the old scan pose at completion would jump backwards.
 {
 auto map = make_wall_map();
 AutonomyCoordinator coord;
 assert(coord.configure().ok && coord.activate().ok);
 double true_x = 1.0;
 iface::ObservationFrame o;
 o.source = "t.moving-lidar";
 o.frame = core::Frame::kLidarLink;
 o.odometry_valid = true;
 o.forward_mps = 0.5;
 o.scan_angle_min_rad = -M_PI;
 o.scan_angle_increment_rad = 2.0 * M_PI / 72;
 o.imu_valid = true;
 for (int i = 0; i < 1000 && !coord.localized(); ++i) {
 if (i > 0) true_x += 0.5 * 0.02;
 o.sequence = static_cast<std::uint64_t>(i + 1);
    o.stamp_ns = static_cast<core::MonotonicNs>(i + 1) * 20'000'000LL;
 o.scan_valid = true;
 o.ranges_m = scan_from(true_x, 0.0, 0.0, map);
 coord.ingest_observation(o);
 coord.update(map, iface::ChannelHealth{}, o.stamp_ns);
 }
 assert(coord.localized());
 assert(std::abs(coord.estimate().x_m - true_x) < 0.6);
 assert(std::abs(coord.estimate().y_m) < 0.6);
 }

  // 9. Deferred startup disambiguation: fabricated ambiguous seed (true
  // pose + far decoy) promotes the true pose once the decoy drops; close
  // twins merge instead of deadlocking; hopeless seeds time out.
  {
    auto map = make_wall_map();
    auto ranges = scan_from(1.0, 0.0, 0.0, map);
    // Sole-survivor promote: the decoy sits outside the raster (local
    // matches reject), the true pose confirms twice and promotes.
    {
      StartupHypotheses tracker;
      GlobalSearchResult amb;
      amb.x_m = 1.3;
      amb.y_m = -0.2;
      amb.yaw_rad = 0.08;
      amb.score = 0.8;
      GlobalSearchResult::RunnerUp decoy{50.0, 50.0, 0.0, 0.78};
      amb.runner_ups.push_back(decoy);
      tracker.seed(amb);
      assert(tracker.size() == 2);
      auto r1 = tracker.update(ranges, -M_PI, 2.0 * M_PI / ranges.size(),
                               map, 1);
      assert(r1.status == StartupTrackStatus::kTracking);
      assert(r1.hypotheses == 2);  // one miss tolerated, decoy kept once


      auto r2 = tracker.update(ranges, -M_PI, 2.0 * M_PI / ranges.size(),
                               map, 2);
 assert(r2.status == StartupTrackStatus::kTracking);
 const auto r3 = tracker.update(
 ranges, -M_PI, 2.0 * M_PI / ranges.size(), map, 3);
 // The decoy survives short transient misses, then three contradictions drop
 // it; the repeatedly supported true pose is now the sole survivor.
 assert(r3.status == StartupTrackStatus::kPromoted);
 assert(std::abs(r3.x_m - 1.0) < 0.6);
 assert(std::abs(r3.y_m - 0.0) < 0.6);
      assert(tracker.empty());  // consumed on promotion
    }
    // Merge: near twins converge to one hypothesis and promote.
    {
      StartupHypotheses tracker;
      GlobalSearchResult amb;
      amb.x_m = 1.3;
      amb.y_m = -0.2;
      amb.yaw_rad = 0.08;
      amb.score = 0.8;
      GlobalSearchResult::RunnerUp twin{1.4, -0.1, 0.05, 0.79};
      amb.runner_ups.push_back(twin);
      tracker.seed(amb);
      auto r1 = tracker.update(ranges, -M_PI, 2.0 * M_PI / ranges.size(),
                               map, 1);
      auto r2 = tracker.update(ranges, -M_PI, 2.0 * M_PI / ranges.size(),
                               map, 2);
      assert(r2.status == StartupTrackStatus::kPromoted);
      assert(std::abs(r2.x_m - 1.0) < 0.6);
    }
    // Timeout: hopeless seeds (all outside the raster) give up; the
    // supervisor retry backstops, the tracker never guesses.
    {
      StartupTrackerConfig cfg;
      cfg.max_scans = 3;
      StartupHypotheses tracker(cfg);
      GlobalSearchResult amb;
      amb.x_m = 50.0;
      amb.y_m = 50.0;
      amb.score = 0.5;
      GlobalSearchResult::RunnerUp decoy{60.0, 60.0, 0.0, 0.49};
      amb.runner_ups.push_back(decoy);
      tracker.seed(amb);
      StartupTrackResult last{StartupTrackStatus::kTracking, 0, 0, 0, 0, 0};
      for (int k = 1; k <= 3; ++k) {
        last = tracker.update(ranges, -M_PI, 2.0 * M_PI / ranges.size(),
                              map, k);
      }
      assert(last.status == StartupTrackStatus::kLost);
      assert(tracker.empty());
    }
    // Single-hypothesis seeds track nothing (no ambiguity to resolve).
    {
      StartupHypotheses tracker;
      GlobalSearchResult solo;
      solo.x_m = 1.0;
      tracker.seed(solo);
      assert(tracker.empty());
    }
 // A global-recovery winner is not allowed to promote by replaying the same
 // stationary scan. Measured motion creates the temporal baseline; the next
 // scan must remain consistent with the propagated hypothesis.
 {
 StartupTrackerConfig config;
 config.promotion_translation_m = 0.01;
 StartupHypotheses tracker(config);
 GlobalSearchResult recovery;
 recovery.x_m = 1.0;
 recovery.position_sigma_m = 0.12;
 recovery.heading_sigma_rad = 0.04;
 tracker.seed(recovery, 0, /*require_motion=*/true);
 const double increment = 2.0 * M_PI / ranges.size();
 const auto stationary_1 = tracker.update(
 ranges, -M_PI, increment, map, 20'000'000, 0.0, 0.0, true);
 const auto stationary_2 = tracker.update(
 ranges, -M_PI, increment, map, 40'000'000, 0.0, 0.0, true);
 assert(stationary_1.status == StartupTrackStatus::kTracking);
 assert(stationary_2.status == StartupTrackStatus::kTracking);
 const auto moved_ranges = scan_from(1.02, 0.0, 0.0, map);
 const auto promoted = tracker.update(
 moved_ranges, -M_PI, increment, map, 60'000'000, 1.0, 0.0, true);
 assert(promoted.status == StartupTrackStatus::kPromoted);
 assert(promoted.position_sigma_m > 0.0);
 assert(promoted.heading_sigma_rad > 0.0);
 }
  }

  // 10. Coordinator deferred startup: on a uniform wall the whole-map
  // search rejects WITH runner-ups (twins tie) -> tracker seeds, tracks
  // without ever teleporting the estimate to a twin, and stays honest
  // (unlocalized) while the geometry cannot disambiguate.
  {
    auto uniform = make_wall_map();
    for (std::size_t i = 0; i < uniform.occupancy.size(); ++i) {
      int gx = i % 40, gy = i / 40;
      double cx = -10.0 + (gx + 0.5) * 0.5;
      uniform.occupancy[i] = (cx >= 4.0 && cx < 5.0) ? 100 : 0;
    }
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
 coord.set_goal({0.0, 5.0, 0.0});
    auto ranges = scan_from(1.0, 0.0, 0.0, uniform);
 bool saw_seeded = false;
 bool saw_tracking = false;
 for (int k = 1; k <= 600 && !saw_tracking; ++k) {
      iface::ObservationFrame o;
      o.sequence = static_cast<std::uint64_t>(k);
    o.stamp_ns = static_cast<core::MonotonicNs>(k) * 20'000'000LL;
      o.source = "t.lidar";
      o.frame = core::Frame::kLidarLink;
      o.odometry_valid = true;
      o.forward_mps = 0.0;  // holding: twins never separate
      o.yaw_rate_radps = 0.0;
      o.scan_valid = true;
      o.ranges_m = ranges;
      o.scan_angle_min_rad = -M_PI;
      o.scan_angle_increment_rad = 2.0 * M_PI / ranges.size();
      o.imu_valid = true;
      coord.ingest_observation(o);
 const iface::MotionCommand cmd =
 coord.update(uniform, iface::ChannelHealth{}, o.stamp_ns);
 // A map-frame route exists, but the ambiguous scan has not established
 // that frame. Navigation must remain stopped until localization succeeds.
 assert(cmd.is_safe_stop());
 // Ambiguous global seeds deferred tracking instead of guessing. The
 // cooperative search may need many bounded ticks before that decision.
 if (coord.last_localization_action() == "startup-seeded") {
 saw_seeded = true;
 } else if (saw_seeded &&
 coord.last_localization_action() == "startup-tracking") {
 saw_tracking = true;
 }
    }
 assert(saw_seeded && saw_tracking);
    // Genuinely unobservable: still unlocalized, estimate never teleported
    // to a corridor twin (odometry holding near the origin).
    assert(!coord.localized());
    assert(std::abs(coord.estimate().x_m) < 1.0);
    assert(std::abs(coord.estimate().y_m) < 1.0);
  }

  // 8. Mapless startup: no static raster -> trust the odometry origin once,
  // SLAM owns the drift. No teleport (seeded at the live estimate).
  {
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    iface::MapSnapshot empty_map;  // vector-only: has_raster() == false
    assert(!empty_map.has_raster());
    for (int k = 1; k <= 3; ++k) {
      iface::ObservationFrame o;
      o.sequence = static_cast<std::uint64_t>(k);
    o.stamp_ns = static_cast<core::MonotonicNs>(k) * 20'000'000LL;
      o.source = "t.lidar";
      o.frame = core::Frame::kLidarLink;
      o.odometry_valid = true;
      o.forward_mps = 1.0;
      o.scan_valid = true;
      o.ranges_m.assign(72, 10.0f);
      o.scan_angle_min_rad = -M_PI;
      o.scan_angle_increment_rad = 2.0 * M_PI / o.ranges_m.size();
      coord.ingest_observation(o);
      coord.update(empty_map, iface::ChannelHealth{}, o.stamp_ns);
      if (k == 1) {
        // First tick seeds the odometry origin and reports the mode.
        assert(coord.last_localization_action() == "slam-only");
      }
    }
    assert(coord.localized());
 // The first sample establishes the time origin. Two subsequent 20 ms
 // intervals at 1 m/s produce 0.04 m; startup must not invent motion before
 // the first timestamp or discard motion after it.
    assert(coord.estimate().valid);
 assert(std::abs(coord.estimate().x_m - 0.04) < 1e-9);
    assert(coord.slam_keyframes() >= 1);
    // Selector contract, unit level.
    assert(select_startup_mode(true, true).mode ==
           StartupMode::kSeedAndTrack);
    assert(select_startup_mode(true, false).mode ==
           StartupMode::kGlobalSearch);
    assert(select_startup_mode(false, false).mode == StartupMode::kSlamOnly);
  }

 // 9. An explicit SLAM-only deployment does not wait for global search even
 // if a known raster is available. The raster may be a sparse navigation
 // prior rather than a localization authority; the ROBOT odometry origin is
 // established once and SLAM starts immediately.
 {
 AutonomyCoordinator coord;
 coord.set_startup_mode(StartupMode::kSlamOnly);
 assert(coord.configure().ok && coord.activate().ok);
 auto map = make_wall_map();
 iface::ObservationFrame o;
 o.sequence = 1;
 o.stamp_ns = 20'000'000;
 o.source = "t.lidar";
 o.frame = core::Frame::kLidarLink;
 o.odometry_valid = true;
 o.forward_mps = 0.5;
 o.scan_valid = true;
 o.ranges_m.assign(72, 10.0f);
 o.scan_angle_min_rad = -M_PI;
 o.scan_angle_increment_rad = 2.0 * M_PI / o.ranges_m.size();
 coord.ingest_observation(o);
 coord.update(map, iface::ChannelHealth{}, o.stamp_ns);
 assert(coord.localized());
 assert(coord.last_localization_action() == "slam-only");
 assert(std::abs(coord.estimate().x_m) < 1e-12);
 assert(coord.slam_keyframes() >= 1);

 // Once the frame exists, the mode remains SLAM-only: a surveyed scan and
 // a due correction cadence must not silently reactivate scan-to-survey
 // localization or let an incidental obstacle claim absolute certainty.
 o.sequence = 2;
 o.stamp_ns = 400'000'000;
 o.ranges_m = scan_from(1.0, 0.0, 0.0, map);
 coord.ingest_observation(o);
 coord.update(map, iface::ChannelHealth{}, o.stamp_ns);
 assert(coord.last_localization_action() == "slam-only");
 }

 // 10. Mapless scan-to-submap tracking owns the local navigation frame. An
 // accepted track must bound the covariance used by planning; otherwise
 // wheel/IMU process noise eventually stops a healthy mapless mission. This
 // is local SLAM confidence, not surveyed-map or simulator truth certainty.
 {
 LocalizationSystem localization;
 localization.set_startup_mode(StartupMode::kSlamOnly);
 localization.reset();
 FusedOdometryEstimator estimator;
 estimator.reset();
 RayClearingMapper mapper;
 mapper.reset();
 iface::MapSnapshot no_survey;
 const auto geometry = make_wall_map();
 std::size_t accepted_tracks = 0;
 double true_x_m = 0.0;
 for (int sample = 0; sample < 8 && accepted_tracks == 0; ++sample) {
 iface::ObservationFrame observation;
 observation.sequence = static_cast<std::uint64_t>(sample + 1);
 observation.stamp_ns =
 1'000'000'000LL + static_cast<core::MonotonicNs>(sample) * 200'000'000LL;
 observation.source = "t.mapless-correlated-track";
 observation.frame = core::Frame::kLidarLink;
 observation.odometry_valid = true;
 observation.forward_mps = sample == 0 ? 0.0 : 1.25;
 observation.scan_valid = true;
 observation.ranges_m = scan_from(true_x_m, 0.0, 0.0, geometry);
 observation.scan_angle_min_rad = -M_PI;
 observation.scan_angle_increment_rad = 2.0 * M_PI / 72;
 estimator.predict(observation);
 const double sigma_before = estimator.estimate().position_sigma_m;
 const std::size_t tracks_before = localization.slam_tracks();
 localization.update(
 no_survey, observation, observation.stamp_ns, true, estimator, mapper);
 accepted_tracks = localization.slam_tracks();
 if (accepted_tracks > tracks_before) {
 assert(estimator.estimate().position_sigma_m < sigma_before);
 }
 true_x_m += 0.25;
 }
 assert(accepted_tracks > 0);
 }

  std::printf("localization ok\n");
  return 0;
}
