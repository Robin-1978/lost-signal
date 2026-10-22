#include <cassert>
#include <cmath>
#include <vector>

#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/localization/global_search.hpp"
#include "robot/autonomy/localization/scan_matcher.hpp"
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
      o.stamp_ns = (k - 1) * 20'000'000;  // 20 ms steps at 1 m/s
      est.predict(o);
    }
    const double drifted_x = est.estimate().x_m;
    assert(std::abs(drifted_x - 0.1) < 1e-9);
    const double before = est.estimate().position_sigma_m;
    assert(before > 0.05);
    est.correct_pose(0.02, 0.0, 0.0, 0.02, 0.01, 2'000'000);
    assert(est.estimate().position_sigma_m < before);
    // Blended, not snapped: the estimate moves toward the correction
    // proportionally to relative certainty, never jumps to it.
    assert(est.estimate().x_m > 0.025 && est.estimate().x_m < 0.09);
    // Malformed corrections are ignored, never applied.
    const double x0 = est.estimate().x_m;
    est.correct_pose(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.02,
                     0.01, 3'000'000);
    assert(est.estimate().x_m == x0);
  }

  // 4. Supervisor: unknown start -> global attempt; accept -> localized;
  // cadence gates local corrections; repeated rejections escalate.
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
      o.stamp_ns = k * 20'000'000;
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
  }

  // 7. Coordinator kidnap path: unlocalized start takes the global branch
  // and lands localized near the true pose.
  {
    auto map = make_wall_map();
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    // No seed_pose: the supervisor starts unlocalized and the first tick
    // runs the whole-map search.
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
    coord.ingest_observation(o);
    coord.update(map, iface::ChannelHealth{}, o.stamp_ns);
    assert(coord.localized());
    assert(std::abs(coord.estimate().x_m - 1.0) < 0.6);
    assert(std::abs(coord.estimate().y_m - 0.0) < 0.6);
  }

  std::printf("localization ok\n");
  return 0;
}
