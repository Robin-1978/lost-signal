#include <cassert>
#include <cmath>
#include <vector>

#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/localization/global_search.hpp"
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

 // Sparse scans fail before allocating or preparing a map-sized index.
 std::vector<float> no_hits(72, 2.0F);
 GlobalSearchResult insufficient = bounded_searcher.search(
 no_hits, -M_PI, 2.0 * M_PI / no_hits.size(), empty_extent, 1);
 assert(!insufficient.accepted);
 assert(insufficient.rejection_reason == "insufficient geometric returns");
 assert(insufficient.coarse_evaluations == 0);
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
      // Decoy rejected twice -> dropped; true pose confirmed twice.
      assert(r2.status == StartupTrackStatus::kPromoted);
      assert(std::abs(r2.x_m - 1.0) < 0.6);
      assert(std::abs(r2.y_m - 0.0) < 0.6);
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

  std::printf("localization ok\n");
  return 0;
}
