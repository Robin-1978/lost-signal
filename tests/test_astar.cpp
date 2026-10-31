#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "robot/autonomy/planning/astar_planner.hpp"
#include "robot/autonomy/planning/route_pipeline.hpp"

namespace {
lost_signal::robot::iface::MapSnapshot make_map(int w, int h, double res,
                                                 double ox, double oy) {
  lost_signal::robot::iface::MapSnapshot m;
  m.map_id = "test";
 static std::uint64_t next_revision = 1;
 m.revision = next_revision++;
  m.width = w;
  m.height = h;
  m.resolution_m = res;
  m.origin_x_m = ox;
  m.origin_y_m = oy;
  m.occupancy.assign(w * h, -1);
  m.elevation_m.assign(
      w * h, std::numeric_limits<float>::quiet_NaN());  // unknown height
  m.frame_id = "map";
  return m;
}

void fill_box(lost_signal::robot::iface::MapSnapshot& m, double x0, double y0,
              double x1, double y1) {
  int w = m.width, h = m.height;
  for (int gy = 0; gy < h; ++gy)
    for (int gx = 0; gx < w; ++gx) {
      double cx = m.origin_x_m + (gx + 0.5) * m.resolution_m;
      double cy = m.origin_y_m + (gy + 0.5) * m.resolution_m;
      if (cx >= x0 && cx < x1 && cy >= y0 && cy < y1)
        m.occupancy[gy * w + gx] = 100;
    }
}

double distance_to_box(double x, double y, double x0, double y0,
 double x1, double y1) {
 const double dx = std::max({x0 - x, 0.0, x - x1});
 const double dy = std::max({y0 - y, 0.0, y - y1});
 return std::hypot(dx, dy);
}

lost_signal::robot::autonomy::EstimatedState at(double x, double y) {
  lost_signal::robot::autonomy::EstimatedState e;
  e.valid = true;
  e.x_m = x;
  e.y_m = y;
  return e;
}
}  // namespace

class CountingPlanner : public lost_signal::robot::autonomy::IGlobalPlanner {
 public:
 lost_signal::robot::autonomy::Route plan(
 const lost_signal::robot::autonomy::EstimatedState& start,
 const lost_signal::robot::autonomy::RoutePoint& goal,
 const lost_signal::robot::iface::MapSnapshot&) override {
 ++calls;
 lost_signal::robot::autonomy::Route out;
 out.valid = start.valid;
 out.points = {{start.x_m, start.y_m, start.yaw_rad}, goal};
 return out;
 }
 int calls = 0;
};

class IncrementalCountingPlanner
 : public lost_signal::robot::autonomy::IGlobalPlanner {
 public:
 lost_signal::robot::autonomy::Route plan(
 const lost_signal::robot::autonomy::EstimatedState&,
 const lost_signal::robot::autonomy::RoutePoint&,
 const lost_signal::robot::iface::MapSnapshot&) override {
 assert(false && "incremental planner must not use synchronous plan");
 return {};
 }
 bool supports_incremental_plan() const override { return true; }
 std::size_t incremental_plan_budget() const override { return 1; }
 void begin_plan(
 const lost_signal::robot::autonomy::EstimatedState& start,
 const lost_signal::robot::autonomy::RoutePoint& goal,
 const lost_signal::robot::iface::MapSnapshot&) override {
 ++begin_calls;
 advances = 0;
 start_ = start;
 goal_ = goal;
 }
 lost_signal::robot::autonomy::GlobalPlanProgress continue_plan(
 std::size_t budget) override {
 assert(budget == 1);
 ++continue_calls;
 ++advances;
 if (fail) {
 return {lost_signal::robot::autonomy::GlobalPlanStatus::kFailed, {}};
 }
 if (advances < advances_to_finish) {
 return {lost_signal::robot::autonomy::GlobalPlanStatus::kPending, {}};
 }
 lost_signal::robot::autonomy::Route route;
 route.valid = true;
 route.points = {
 {start_.x_m, start_.y_m, start_.yaw_rad}, goal_};
 return {lost_signal::robot::autonomy::GlobalPlanStatus::kSucceeded,
 std::move(route)};
 }
 void cancel_plan() override { ++cancel_calls; }

 int begin_calls = 0;
 int continue_calls = 0;
 int cancel_calls = 0;
 int advances = 0;
 int advances_to_finish = 3;
 bool fail = false;

 private:
 lost_signal::robot::autonomy::EstimatedState start_;
 lost_signal::robot::autonomy::RoutePoint goal_;
};

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  AStarPlanner planner;

 // Invalid policy cannot silently collapse footprint inflation.
 {
 AStarPlanner::Config invalid;
 invalid.footprint_radius_m = 0.0;
 AStarPlanner invalid_planner(invalid);
 auto map = make_map(20, 20, 0.5, -5.0, -5.0);
 assert(!invalid_planner.plan(at(0.0, 0.0), {2.0, 0.0, 0.0}, map).valid);
 }

  // No raster: degraded straight-line route.
  {
    Route r = planner.plan(at(0.0, 0.0), {2.0, 0.0, 0.0}, iface::MapSnapshot{});
    assert(r.valid && r.points.size() >= 2);
  }

  // A start and goal that fall in the same raster cell still need a route
  // segment: the local planner must be able to settle the terminal heading
  // instead of treating a reachable pose as a global-plan failure.
  {
    auto map = make_map(8, 8, 1.0, -4.0, -4.0);
    for (auto& cell : map.occupancy) cell = 0;
    const Route r = planner.plan(at(0.10, 0.10), {0.20, 0.20, 1.0}, map);
    assert(r.valid);
    assert(r.points.size() >= 2);
    assert(std::abs(r.points.front().x_m - 0.10) < 1e-12);
    assert(std::abs(r.points.front().y_m - 0.10) < 1e-12);
    assert(std::abs(r.points.back().x_m - 0.20) < 1e-12);
    assert(std::abs(r.points.back().y_m - 0.20) < 1e-12);
    assert(std::abs(r.points.back().yaw_rad - 1.0) < 1e-12);
  }

  // A free start may lie inside the conservative clearance inflation after a
  // pose correction. A* keeps it as an escape seed while rejecting raw
  // occupied starts; the route must begin at the exact ROBOT estimate.
  {
    auto map = make_map(16, 16, 0.5, -4.0, -4.0);
    std::fill(map.occupancy.begin(), map.occupancy.end(), 0);
    fill_box(map, 0.5, -0.25, 1.0, 0.25);
    const Route r = planner.plan(at(0.0, 0.0), {-2.0, 0.0, 0.0}, map);
    assert(r.valid && r.points.size() >= 2);
    assert(std::abs(r.points.front().x_m) < 1e-12);
    assert(std::abs(r.points.front().y_m) < 1e-12);
  }

  // A normal free start keeps the raster route origin.  Exact-pose
  // preservation is reserved for the clearance-only escape case above; it
  // must not introduce a short, off-grid first segment on every route.
  {
    auto map = make_map(16, 16, 0.5, -4.0, -4.0);
    std::fill(map.occupancy.begin(), map.occupancy.end(), 0);
    const Route r = planner.plan(at(0.10, 0.10), {2.0, 0.10, 0.0}, map);
    assert(r.valid && r.points.size() >= 2);
    assert(std::abs(r.points.front().x_m - 0.25) < 1e-12);
    assert(std::abs(r.points.front().y_m - 0.25) < 1e-12);
  }

 // Route refresh policy is module-owned: an explicit replan calls the
 // planner even when an existing route is still valid.
 {
 CountingPlanner counting;
 Route active;
 const auto map = iface::MapSnapshot{};
 assert(RoutePlanningPipeline::refresh(counting, at(0.0, 0.0),
 {2.0, 0.0, 0.0}, true, map, false, active));
 assert(counting.calls == 1 && active.valid);
 assert(!RoutePlanningPipeline::refresh(counting, at(0.0, 0.0),
 {2.0, 0.0, 0.0}, true, map, false, active));
 assert(counting.calls == 1);
 assert(RoutePlanningPipeline::refresh(counting, at(0.0, 0.0),
 {2.0, 0.0, 0.0}, true, map, true, active));
 assert(counting.calls == 2);
 }

 // Cooperative A* obeys its expansion budget and converges to exactly the
 // same route as the synchronous compatibility API.
 {
 AStarPlanner::Config config;
 config.expansions_per_cycle = 1;
 AStarPlanner incremental(config);
 auto map = make_map(80, 80, 0.25, -10.0, -10.0);
 for (auto& cell : map.occupancy) cell = 0;
 fill_box(map, -0.5, -10.0, 0.5, -1.0);
 fill_box(map, -0.5, 1.0, 0.5, 10.0);
 const EstimatedState start = at(-8.0, 0.0);
 const RoutePoint goal{8.0, 0.0, 0.0};
 const Route synchronous = incremental.plan(start, goal, map);
 assert(synchronous.valid);
 incremental.begin_plan(start, goal, map);
 GlobalPlanProgress progress = incremental.continue_plan(1);
 assert(progress.status == GlobalPlanStatus::kPending);
 int cycles = 1;
 while (progress.status == GlobalPlanStatus::kPending && cycles < 10000) {
 progress = incremental.continue_plan(1);
 ++cycles;
 }
 assert(cycles > 1);
 assert(progress.status == GlobalPlanStatus::kSucceeded);
 assert(progress.route.valid);
 assert(progress.route.points.size() == synchronous.points.size());
 for (std::size_t i = 0; i < synchronous.points.size(); ++i) {
 assert(progress.route.points[i].x_m == synchronous.points[i].x_m);
 assert(progress.route.points[i].y_m == synchronous.points[i].y_m);
 }
 }

 // A route is tied to the estimated start from which it was validated. The
 // cross-track trigger is a chassis radius, not its diameter: motion at
 // 0.39 m remains on the route, while 0.41 m starts one event-driven replan.
 {
 CountingPlanner counting;
 RoutePlanningPipeline pipeline;
 Route active;
 const auto map = iface::MapSnapshot{};
 const RoutePoint goal{2.0, 0.0, 0.0};
 assert(pipeline.advance(
 counting, at(0.0, 0.0), goal, true, map, false, active, 0.4));
 assert(counting.calls == 1 && active.valid);
 assert(!pipeline.advance(
 counting, at(0.2, 0.39), goal, true, map, false, active, 0.4));
 assert(counting.calls == 1);
 assert(pipeline.advance(
 counting, at(0.2, 0.41), goal, true, map, false, active, 0.4));
 assert(counting.calls == 2 && active.valid);
 assert(active.points.front().y_m == 0.41);
 }

 // The route pipeline retains one in-flight request across decision cycles,
 // atomically publishes success, and cancels it when the goal changes.
 {
 IncrementalCountingPlanner counting;
 RoutePlanningPipeline pipeline;
 Route active;
 auto map = make_map(20, 20, 0.5, -5.0, -5.0);
 const RoutePoint first_goal{2.0, 0.0, 0.0};
 assert(pipeline.advance(
 counting, at(0.0, 0.0), first_goal, true, map, true, active));
 assert(counting.begin_calls == 1 && counting.continue_calls == 1);
 assert(pipeline.pending() && !pipeline.failed());
 assert(!active.valid);
 assert(pipeline.advance(
 counting, at(0.0, 0.0), first_goal, true, map, false, active));
 assert(counting.begin_calls == 1 && counting.continue_calls == 2);
 assert(!active.valid);
 const RoutePoint second_goal{3.0, 0.0, 0.0};
 assert(pipeline.advance(
 counting, at(0.0, 0.0), second_goal, true, map, false, active));
 assert(counting.cancel_calls == 1 && counting.begin_calls == 2);
 assert(!active.valid);
 assert(pipeline.advance(
 counting, at(0.0, 0.0), second_goal, true, map, false, active));
 assert(pipeline.advance(
 counting, at(0.0, 0.0), second_goal, true, map, false, active));
 assert(active.valid && active.points.back().x_m == second_goal.x_m);
 assert(!pipeline.pending() && !pipeline.failed());
 }

 // A failed request is latched instead of creating a planning storm. Moving
 // into another raster cell supplies a meaningfully different start and
 // permits one retry.
 {
 IncrementalCountingPlanner counting;
 counting.fail = true;
 RoutePlanningPipeline pipeline;
 Route active;
 auto map = make_map(20, 20, 0.5, -5.0, -5.0);
 const RoutePoint goal{2.0, 0.0, 0.0};
 assert(pipeline.advance(
 counting, at(0.0, 0.0), goal, true, map, false, active));
 assert(counting.begin_calls == 1 && !active.valid);
 assert(!pipeline.pending() && pipeline.failed());
 assert(!pipeline.advance(
 counting, at(0.1, 0.1), goal, true, map, false, active));
 assert(counting.begin_calls == 1);
 assert(pipeline.advance(
 counting, at(0.6, 0.0), goal, true, map, false, active));
 assert(counting.begin_calls == 2);
 }

 // A same-goal refresh publishes atomically. If the replacement fails from
 // a temporarily inflated start cell, keep the last validated route so the
 // local planner and independent safety layers can execute a safe escape;
 // repeated force requests do not restart the same failed search until the
 // measured start changes cells.
 {
 IncrementalCountingPlanner counting;
 counting.advances_to_finish = 1;
 RoutePlanningPipeline pipeline;
 Route active;
 auto map = make_map(20, 20, 0.5, -5.0, -5.0);
 const RoutePoint goal{2.0, 0.0, 0.0};
 assert(pipeline.advance(
 counting, at(0.0, 0.0), goal, true, map, false, active));
 assert(active.valid && counting.begin_calls == 1);

 // A same-goal refresh keeps the last validated route until the incremental
 // replacement succeeds; the local planner never sees a planning gap.
 counting.advances_to_finish = 3;
 assert(!pipeline.advance(
 counting, at(0.2, 0.0), goal, true, map, true, active));
 assert(pipeline.pending() && active.valid);
 assert(!pipeline.advance(
 counting, at(0.2, 0.0), goal, true, map, false, active));
 assert(pipeline.pending() && active.valid);
 assert(pipeline.advance(
 counting, at(0.2, 0.0), goal, true, map, false, active));
 assert(!pipeline.pending() && active.valid);

 counting.fail = true;
 assert(!pipeline.advance(
 counting, at(0.1, 0.0), goal, true, map, true, active));
 assert(active.valid && pipeline.failed() && counting.begin_calls == 3);
 assert(!pipeline.advance(
 counting, at(0.1, 0.0), goal, true, map, true, active));
 assert(active.valid && counting.begin_calls == 3);
 assert(!pipeline.advance(
 counting, at(0.6, 0.0), goal, true, map, true, active));
 assert(active.valid && counting.begin_calls == 4);
 }

  // Wall with a gap: route threads the gap, never through occupied cells.
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    fill_box(map, 4.0, -10.0, 5.0, -1.0);
    fill_box(map, 4.0, 1.0, 5.0, 10.0);  // 2 m gap at y in [-1, 1]
    Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
    assert(r.valid);
    assert(r.points.size() >= 2);
    for (const auto& p : r.points) assert(!map.blocked(p.x_m, p.y_m));
    bool in_gap = false;
    for (const auto& p : r.points) {
      if (p.x_m > 3.5 && p.x_m < 5.5) {
        assert(p.y_m > -1.5 && p.y_m < 1.5);
        in_gap = true;
      }
    }
    assert(in_gap);
  }

  // Full wall: no path, invalid route (fail closed upstream).
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    fill_box(map, 4.0, -10.0, 5.0, 10.0);
    Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
    assert(!r.valid);
  }

 // A center point fits through this raster opening, but the declared
 // footprint plus hard reserve does not. Global planning must reject the
 // route before local control reaches the bottleneck.
 {
 auto map = make_map(80, 80, 0.25, -10.0, -10.0);
 fill_box(map, 4.0, -10.0, 5.0, -0.25);
 fill_box(map, 4.0, 0.25, 5.0, 10.0);
 Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
 assert(!r.valid);
 }

 // A diagonal route around one occupied cell must follow the circular
 // footprint envelope at its corner. Sample complete route segments rather
 // than only grid vertices so a diagonal chord cannot cut through the hard
 // clearance radius between two nominally free cells.
 {
 auto map = make_map(100, 100, 0.1, -5.0, -5.0);
 for (auto& cell : map.occupancy) cell = 0;
 fill_box(map, 0.0, 0.0, 0.1, 0.1);
 Route r = planner.plan(at(-2.0, -2.0), {2.0, 2.0, 0.0}, map);
 assert(r.valid);
 constexpr double kHardRadiusM = 0.5; // default 0.4 m body + 0.1 m reserve
 for (std::size_t i = 1; i < r.points.size(); ++i) {
 const double length = std::hypot(
 r.points[i].x_m - r.points[i - 1].x_m,
 r.points[i].y_m - r.points[i - 1].y_m);
 const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.01)));
 for (int sample = 0; sample <= samples; ++sample) {
 const double blend = static_cast<double>(sample) / samples;
 const double x = r.points[i - 1].x_m +
 (r.points[i].x_m - r.points[i - 1].x_m) * blend;
 const double y = r.points[i - 1].y_m +
 (r.points[i].y_m - r.points[i - 1].y_m) * blend;
 assert(distance_to_box(x, y, 0.0, 0.0, 0.1, 0.1) + 1e-9 >=
 kHardRadiusM);
 }
 }
 }

  // Grade: a hill between start and goal is skirted, not climbed.
  {
    auto map = make_map(30, 20, 1.0, 0.0, 0.0);
    for (int gy = 0; gy < 20; ++gy)
      for (int gx = 0; gx < 30; ++gx) {
        map.occupancy[gy * 30 + gx] = 0;  // all free: isolate grade effect
        const double cx = gx + 0.5, cy = gy + 0.5;
        const double dx = cx - 15.0, dy = cy - 10.0;
        map.elevation_m[gy * 30 + gx] =
            static_cast<float>(5.0 * std::exp(-(dx * dx + dy * dy) / 18.0));
      }
    Route r = planner.plan(at(1.0, 10.0), {28.0, 10.0, 0.0}, map);
    assert(r.valid);
    // Went around the hill, not over it.
    for (const auto& p : r.points) {
      const double d =
          std::hypot(p.x_m - 15.0, p.y_m - 10.0);
      assert(d > 2.5);
    }
    // Detour is longer than the straight line.
    double length = 0.0;
    for (std::size_t i = 1; i < r.points.size(); ++i) {
      length += std::hypot(r.points[i].x_m - r.points[i - 1].x_m,
                           r.points[i].y_m - r.points[i - 1].y_m);
    }
    assert(length > 27.0 + 2.0);
  }

  // Cliff: a full-height elevation step is impassable, route invalid.
  {
    auto map = make_map(30, 20, 1.0, 0.0, 0.0);
    for (int gy = 0; gy < 20; ++gy)
      for (int gx = 0; gx < 30; ++gx) {
        map.occupancy[gy * 30 + gx] = 0;
        map.elevation_m[gy * 30 + gx] = gx < 10 ? 0.0f : 5.0f;
      }
    Route r = planner.plan(at(1.0, 10.0), {28.0, 10.0, 0.0}, map);
    assert(!r.valid);
  }

  // Believed roughness (traversal model hook): rough bands with a smooth
  // gap route through the gap; without the model the same map goes
  // straight. Roughness is the ROBOT's own IMU belief, never the author's.
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    for (auto& c : map.occupancy) c = 0;  // all free: isolate roughness
    TraversalModel model;
    model.bind(map);
    // Observe the bands rough (bump hits at 0.5 m/s); the gap stays prior.
    for (int gx = 28; gx <= 29; ++gx) {
      for (int gy = 0; gy < 40; ++gy) {
        const double cy = -10.0 + (gy + 0.5) * 0.5;
        if (cy >= -1.0 && cy < 1.0) continue;  // smooth gap
        EstimatedState est;
        est.valid = true;
        est.attitude_valid = true;
        est.x_m = -10.0 + (gx + 0.5) * 0.5;
        est.y_m = cy;
        est.forward_mps = 0.5;
        for (int k = 1; k <= 50; ++k) {
          model.observe_motion(est, 9.81 + 4.5, k * 0.02, 0.02);
        }
      }
    }
    double conf = 0.0;
    assert(model.roughness_at(4.25, 5.0, &conf) > 0.05);
    assert(model.roughness_at(4.25, 0.0, &conf) == 0.02 && conf == 0.0);
    // No model: straight through (shortest).
    {
      Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
    }
    // With the belief: threads the smooth gap.
    {
      AStarPlanner rough_planner;
      AStarPlanner::Config cfg;
      cfg.roughness_cost_per_m = 30.0;  // test knob: price the belief hard
      rough_planner = AStarPlanner(cfg);
      rough_planner.set_traversal(&model);
      Route r = rough_planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
      bool in_gap = false;
      for (const auto& p : r.points) {
        if (p.x_m > 3.5 && p.x_m < 5.5) {
          assert(p.y_m > -1.5 && p.y_m < 1.5);
          in_gap = true;
        }
      }
      assert(in_gap);
    }
  }


  // Traction memory (beyond main): slipped-on bands with a clean gap
  // route through the gap; without the memory the same map goes straight.
  // Slip is measured fire evidence, never the author's friction.
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    for (auto& c : map.occupancy) c = 0;  // all free: isolate traction
    TraversalModel model;
    model.bind(map);
    // Fire latched along the bands (severity 0.5); the gap stays prior.
    for (int gx = 28; gx <= 29; ++gx) {
      for (int gy = 0; gy < 40; ++gy) {
        const double cx = -10.0 + (gx + 0.5) * 0.5;
        const double cy = -10.0 + (gy + 0.5) * 0.5;
        if (cy >= -1.0 && cy < 1.0) continue;  // clean gap
        model.observe_slip(cx, cy, 0.5, 1.0 + gx * 0.01 + gy * 0.001);
      }
    }
    double conf = 0.0;
    assert(model.slip_at(4.25, 5.0, &conf) == 0.5 && conf == 1.0);
    assert(model.slip_at(4.25, 0.0, &conf) == 0.0 && conf == 0.0);
    // No memory: straight through (shortest).
    {
      Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
    }
    // With the memory: threads the clean gap.
    {
      AStarPlanner slip_planner;
      slip_planner.set_traversal(&model);
      Route r = slip_planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
      bool in_gap = false;
      for (const auto& p : r.points) {
        if (p.x_m > 3.5 && p.x_m < 5.5) {
          assert(p.y_m > -1.5 && p.y_m < 1.5);
          in_gap = true;
        }
      }
      assert(in_gap);
    }
  }

  std::printf("astar ok\n");
  return 0;
}
