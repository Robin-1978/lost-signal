#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/planning/dwa_planner.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/prediction.hpp"
#include "robot/iface/command.hpp"

namespace {
lost_signal::robot::autonomy::LocalMap make_map(int w, int h, double res,
                                                double ox, double oy,
                                                int8_t fill) {
  lost_signal::robot::autonomy::LocalMap m;
  m.valid = true;
  m.resolution_m = res;
  m.width = w;
  m.height = h;
  m.origin_x_m = ox;
  m.origin_y_m = oy;
  m.occupancy.assign(w * h, fill);
  return m;
}

void fill_box(lost_signal::robot::autonomy::LocalMap& m, double x0, double y0,
              double x1, double y1) {
  for (int gy = 0; gy < static_cast<int>(m.height); ++gy)
    for (int gx = 0; gx < static_cast<int>(m.width); ++gx) {
      double cx = m.origin_x_m + (gx + 0.5) * m.resolution_m;
      double cy = m.origin_y_m + (gy + 0.5) * m.resolution_m;
      if (cx >= x0 && cx < x1 && cy >= y0 && cy < y1)
        m.occupancy[gy * m.width + gx] = 100;
    }
}
}  // namespace

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  platform::ChassisLimits limits{.max_forward_mps = 2.0,
                                 .max_yaw_rate_radps = 1.5,
                                 .footprint_radius_m = 0.4};

  // LocalMap queries: observed-free vs blocked vs unknown.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, -1);
    assert(!m.is_free(0.0, 0.0));      // unknown is not free (conservative)
 assert(m.is_blocked(0.0, 0.0)); // unknown is blocked (conservative)
    assert(!m.segment_is_free(0.0, 0.0, 3.0, 0.0));  // conservative
 assert(!m.segment_is_traversable(0.0, 0.0, 3.0, 0.0)); // conservative
    fill_box(m, 4.0, -5.0, 5.0, 5.0);
    assert(!m.is_free(4.5, 0.0));
    assert(m.is_blocked(4.5, 0.0));
    assert(!m.segment_is_traversable(0.0, 0.0, 8.0, 0.0));
    assert(m.clearance(0.0, 0.0) > 3.0);

 // Lethal inflation uses distance to the occupied cell square, so a circular
 // footprint cannot clip a raster corner even when radius < one cell.
 LocalMap fine = make_map(20, 20, 0.1, -1.0, -1.0, 0);
 fill_box(fine, 0.0, 0.0, 0.1, 0.1);
 const LocalMap inflated = fine.inflated(0.06);
 assert(inflated.is_blocked(-0.05, 0.05));
 assert(inflated.is_free(-0.15, 0.05));
 // The envelope is Euclidean around the full cell square: axial and near
 // diagonal cells inside the radius are lethal, while a farther diagonal
 // cell outside the circular corner remains free (no square dilation).
 const LocalMap corner = fine.inflated(0.16);
 assert(corner.is_blocked(-0.15, 0.05));
 assert(corner.is_blocked(-0.05, -0.05));
 assert(corner.is_free(-0.15, -0.15));
 const LocalCostmap costmap = make_local_costmap(fine, 0.06, 0.20);
 assert(costmap.cost_at(-0.05, 0.05) == 1.0f);
 assert(costmap.cost_at(-0.15, 0.05) >
 costmap.cost_at(-0.35, 0.05));
 assert(costmap.cost_at(-0.35, 0.05) == 0.0f);
 // Soft cost follows the same radial geometry around a corner and decays
 // monotonically outside the hard envelope.
 const LocalCostmap corner_cost = make_local_costmap(fine, 0.10, 0.30);
 assert(corner_cost.cost_at(-0.15, -0.05) >
 corner_cost.cost_at(-0.25, -0.15));
 assert(corner_cost.cost_at(-0.25, -0.15) >
 corner_cost.cost_at(-0.45, -0.45));
  }

  // DWA on an open map drives toward the route goal, deterministically.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 1.0;
    est.y_m = 1.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    StraightLinePlanner global;
    Route route = global.plan(est, {8.0, 1.0, 0.0}, iface::MapSnapshot{});
    assert(route.valid);
    DwaPlanner planner;
    LocalPlan a = planner.plan(est, perc, m, route, limits);
    LocalPlan b = planner.plan(est, perc, m, route, limits);
    assert(a.has_plan && a.best.feasible);
    assert(a.best.forward_mps > 0.0);  // moves, not a hold
    assert(a.best.forward_mps == b.best.forward_mps &&
           a.best.yaw_rate_radps == b.best.yaw_rate_radps);
  }

 // Local decisions depend on the reachable route neighborhood, not on the
 // total mission length. Appending kilometres of identical route beyond the
 // prediction and braking horizon must leave every DWA proposal unchanged.
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 1.0;
 est.y_m = 1.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route short_route;
 short_route.valid = true;
 for (int i = 0; i <= 80; ++i) {
 short_route.points.push_back(
 {0.25 * static_cast<double>(i), 1.0, 0.0, 1});
 }
 Route long_route = short_route;
 for (int i = 81; i <= 8000; ++i) {
 long_route.points.push_back(
 {0.25 * static_cast<double>(i), 1.0, 0.0, 1});
 }
 DwaPlanner short_planner;
 DwaPlanner long_planner;
 const LocalPlan short_plan =
 short_planner.plan(est, perc, m, short_route, limits);
 const LocalPlan long_plan =
 long_planner.plan(est, perc, m, long_route, limits);
 assert(short_plan.has_plan && long_plan.has_plan);
 assert(short_plan.candidates.size() == long_plan.candidates.size());
 for (std::size_t i = 0; i < short_plan.candidates.size(); ++i) {
 const auto& a = short_plan.candidates[i];
 const auto& b = long_plan.candidates[i];
 assert(a.forward_mps == b.forward_mps);
 assert(a.yaw_rate_radps == b.yaw_rate_radps);
 assert(a.lateral_mps == b.lateral_mps);
 assert(std::abs(a.cost - b.cost) < 1e-12);
 }
 }

 // Inside the final position tolerance, terminal yaw owns heading authority.
 // The last route segment may point elsewhere after grid quantization; it
 // must not make the planner hold forever beside a correctly reached goal.
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 0.05;
 est.yaw_rad = -0.6;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {{1.0, 0.0, M_PI, 0}, {0.0, 0.0, 0.0, 0}};
 DwaPlanner planner;
 const LocalPlan p = planner.plan(est, perc, m, route, limits);
 assert(p.has_plan);
 assert(p.best.yaw_rate_radps > 0.05);
 }

 // DWA threads a gap: wall with a 2 m opening, goal straight through.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 fill_box(m, 4.0, -5.0, 5.0, -1.0);
 fill_box(m, 4.0, 1.0, 5.0, 5.0);
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 1.0;
 // Keep the start inside the finite rolling window. The boundary itself is
 // unknown/out-of-window and therefore conservatively blocked.
 est.y_m = 0.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    StraightLinePlanner global;
 Route route = global.plan(est, {8.0, 0.0, 0.0}, iface::MapSnapshot{});
    DwaPlanner planner;
    LocalPlan p = planner.plan(est, perc, m, route, limits);
    assert(p.has_plan && p.best.feasible);
    assert(p.best.forward_mps > 0.0);
  }

  // Reverse: a rear goal with a yaw envelope too small to turn requires
 // negative velocity. Forward-only sampling cannot produce this.
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 0.0;
 est.y_m = 0.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 StraightLinePlanner global;
 Route route = global.plan(est, {-2.0, 0.0, 0.0}, iface::MapSnapshot{});
 assert(route.valid);
 platform::ChassisLimits reverse_limits = limits;
 reverse_limits.max_yaw_rate_radps = 0.001;
 DwaPlanner planner;
 LocalPlan p = planner.plan(est, perc, m, route, reverse_limits);
 assert(p.has_plan && p.best.feasible);
 assert(p.best.forward_mps < 0.0);
 }

 // Ackermann envelope: yaw rate follows v*kappa and zero speed cannot
 // generate an in-place turn. The same planner remains unchanged when the
 // optional curvature field is zero (differential/holonomic).
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 0.0;
 est.y_m = 0.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 StraightLinePlanner global;
 Route route = global.plan(est, {8.0, 1.0, 0.0}, iface::MapSnapshot{});
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 0.2;
 DwaPlanner planner;
 LocalPlan p = planner.plan(est, perc, m, route, ack);
 assert(p.has_plan && p.best.feasible);
 assert(std::abs(p.best.yaw_rate_radps) <=
 std::abs(p.best.forward_mps) * ack.max_curvature_inv_m + 1e-9);
 if (std::abs(p.best.forward_mps) < 1e-9)
 assert(std::abs(p.best.yaw_rate_radps) < 1e-9);
 }

 // Stateful pruning must preserve an Ackermann cusp when the return leg
    // passes through a position already visited by the forward leg. A global
    // nearest-point query would jump directly to the reverse segment.
    {
        LocalMap m = make_map(120, 120, 0.1, -4.0, -4.0, 0);
        EstimatedState est;
        est.valid = true;
        est.stamp_ns = 1'000'000;
        PerceptionResult perc;
        perc.scan_ok = true;
        Route route;
        route.valid = true;
        route.points = {
                {0.0, 0.0, 0.0, 1}, {0.6, 0.0, 0.0, 1},
                {1.2, 0.0, 0.0, 1}, {1.8, 0.0, 0.0, 1},
                {2.4, 0.0, 0.0, 1}, {1.8, 0.0, 0.0, -1},
                {1.6, 0.0, 0.0, -1}};
        platform::ChassisLimits ack = limits;
        ack.max_curvature_inv_m = 1.0;
        DwaPlanner planner;
 for (double x : {0.0, 0.6, 1.2, 1.6, 2.2}) {
            est.x_m = x;
            const LocalPlan plan = planner.plan(est, perc, m, route, ack);
            assert(plan.has_plan && plan.best.forward_mps > 0.0);
        }
        est.x_m = 2.4;
        const LocalPlan reverse = planner.plan(est, perc, m, route, ack);
        assert(reverse.has_plan && reverse.best.forward_mps < 0.0);
    }

 // Hybrid-A* may select reverse on its very first edge. The start point's
 // legacy gear annotation must not override the arriving point's explicit
 // reverse gear or turn a feasible departure into a hold.
 {
 LocalMap m = make_map(160, 160, 0.1, 38.0, -124.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 41.87;
 est.y_m = -119.93;
 est.yaw_rad = -1.75;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {
 {41.870, -119.930, -1.750, 1},
 {41.980, -119.546, -1.952, -1},
 {42.204, -119.218, -2.389, -1},
 {42.546, -119.017, -2.827, -1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner planner;
 const LocalPlan departure = planner.plan(est, perc, m, route, ack);
 assert(departure.has_plan && departure.best.forward_mps < 0.0);
 }

 // A car-like terminal maneuver can require reverse travel that initially
 // increases Euclidean distance to its lookahead point. Route progress and
 // heading must beat holding; otherwise a feasible Hybrid-A* cusp deadlocks.
 {
 LocalMap m = make_map(160, 160, 0.1, -4.0, -4.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 0.0;
 est.y_m = 0.0;
 est.yaw_rad = -1.10;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {
 {-0.20, -0.10, -1.10, 1}, {0.0, 0.0, -1.10, 1},
 {0.10, -0.35, -1.35, 1}, {0.05, -0.70, -1.55, 1},
 {0.10, -0.35, -1.55, -1}, {0.30, 0.20, -1.35, -1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner planner;
 for (const auto& pose : std::vector<EstimatedState>{
 est,
 [&] { auto p = est; p.x_m = 0.10; p.y_m = -0.35;
 p.yaw_rad = -1.35; return p; }(),
 [&] { auto p = est; p.x_m = 0.05; p.y_m = -0.70;
 p.yaw_rad = -1.55; return p; }()}) {
 const LocalPlan plan = planner.plan(pose, perc, m, route, ack);
 assert(plan.has_plan);
 if (pose.y_m > -0.69) assert(plan.best.forward_mps > 0.0);
 }
 EstimatedState reverse_pose = est;
 reverse_pose.x_m = 0.04;
 reverse_pose.y_m = -0.58;
 reverse_pose.yaw_rad = -1.30;
 const LocalPlan plan = planner.plan(reverse_pose, perc, m, route, ack);
 assert(plan.has_plan && plan.best.forward_mps < 0.0);

 // A real chassis settles inside a footprint-sized capture region rather
 // than on the exact Hybrid-A* lattice point. Once speed has settled, a
 // 0.30 m tracking residual on this 0.40 m base must still arm the next gear.
 DwaPlanner physical_cusp_planner;
 for (const auto& pose : std::vector<EstimatedState>{
 est,
 [&] { auto p = est; p.x_m = 0.10; p.y_m = -0.35;
 p.yaw_rad = -1.35; return p; }()}) {
 const LocalPlan approach = physical_cusp_planner.plan(
 pose, perc, m, route, ack);
 assert(approach.has_plan && approach.best.forward_mps > 0.0);
 }
 EstimatedState settled_near_cusp = est;
 settled_near_cusp.x_m = 0.05;
 settled_near_cusp.y_m = -0.40;
 settled_near_cusp.yaw_rad = -1.40;
 settled_near_cusp.forward_mps = 0.0;
 const LocalPlan captured = physical_cusp_planner.plan(
 settled_near_cusp, perc, m, route, ack);
 assert(captured.has_plan && captured.best.forward_mps < 0.0);
 }

 // A consumed car-like route cannot correct a residual terminal heading by
 // rotating in place. Request a fresh pose route from global Hybrid-A*
 // rather than holding indefinitely at the final coordinate.
 {
 LocalMap m = make_map(120, 120, 0.1, -4.0, -4.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {{0.0, 0.0, 0.0, 1}, {1.0, 0.0, 0.0, 1},
 {2.0, 0.0, 0.0, 1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner planner;
 for (double x : {0.0, 0.5, 1.0, 1.5}) {
 est.x_m = x;
 const LocalPlan approach = planner.plan(est, perc, m, route, ack);
 assert(!approach.route_replan_requested);
 }
 est.x_m = 2.0;
 est.yaw_rad = 0.5;
 const LocalPlan exhausted = planner.plan(est, perc, m, route, ack);
 assert(exhausted.route_replan_requested);
 }

 // Once global planning supplies a real same-position pose maneuver, being
 // physically close to its final point must not retrigger replanning from
 // the route start. The active lookahead is still away from the robot.
 {
 LocalMap m = make_map(160, 160, 0.1, 38.0, -124.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 42.04;
 est.y_m = -119.96;
 est.yaw_rad = -1.95;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {
 {42.040, -119.960, -1.950, 1},
 {41.930, -120.344, -1.748, 1},
 {41.860, -120.738, -1.748, 1},
 {41.750, -121.122, -1.950, 1},
 {41.898, -120.750, -1.950, -1},
 {42.008, -120.366, -1.748, -1},
 {42.000, -120.000, -1.571, -1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner planner;
 for (int cycle = 0; cycle < 5; ++cycle) {
 const LocalPlan maneuver = planner.plan(est, perc, m, route, ack);
 assert(!maneuver.route_replan_requested);
 assert(maneuver.has_plan && maneuver.best.forward_mps > 0.0);
 }
 }

 // Geometric arrival with unresolved ROBOT covariance is not completion,
 // but covariance alone is not motion authority. DWA retains safe moving
 // candidates; the motion pipeline selects one only when localization has
 // explicitly requested a multi-frame observation baseline.
 {
 LocalMap m = make_map(120, 120, 0.1, -4.0, -4.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 2.0;
 est.y_m = 0.0;
 est.yaw_rad = 0.0;
 est.position_sigma_m = 1.0;
 est.heading_sigma_rad = 0.05;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {{0.0, 0.0, 0.0, 1}, {1.0, 0.0, 0.0, 1},
 {2.0, 0.0, 0.0, 1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner planner;
 const LocalPlan observe = planner.plan(est, perc, m, route, ack);
 assert(observe.has_plan && observe.moving_candidate_count > 0);
 assert(std::abs(observe.best.forward_mps) < 1e-9);
 }

 // Pure-pursuit heading for a car-like base must correct cross-track error.
 // The planned body heading plus bounded cross-track feedback must drive the
 // error down without the near-goal bearing singularity.
 {
 LocalMap m = make_map(160, 160, 0.1, -4.0, -8.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 0.35;
 est.y_m = 0.0;
 est.yaw_rad = -M_PI_2;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {
 {0.0, 0.0, -M_PI_2, 1},
 {0.0, -1.0, -M_PI_2, 1},
 {0.0, -2.0, -M_PI_2, 1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner planner;
 const LocalPlan correction = planner.plan(est, perc, m, route, ack);
 assert(correction.has_plan && correction.best.forward_mps > 0.0);
 assert(correction.best.yaw_rate_radps < 0.0);
 }

 // Near an Ackermann goal, centimetres of lateral error must not replace the
 // planned terminal pose with a sideways bearing to the final coordinate.
 // The steering command keeps converging toward the route's south-facing
 // body heading while correcting the small west-side offset.
 {
 LocalMap m = make_map(160, 160, 0.1, 38.0, -124.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 41.83;
 est.y_m = -119.98;
 est.yaw_rad = -1.04;
 est.forward_mps = 0.2;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {
 {40.6, -117.5, -0.18, 1},
 {41.65, -118.08, -0.79, 1},
 {42.04, -118.76, -1.43, 1},
 {42.00, -120.00, -M_PI_2, 1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner planner;
 const LocalPlan terminal = planner.plan(est, perc, m, route, ack);
 assert(terminal.has_plan && terminal.best.forward_mps > 0.0);
 assert(terminal.best.yaw_rate_radps < 0.0);
 }

 // A car-like dynamic window can quantize to hold near the edge of its route
 // tracking tube. Since waiting cannot reduce stationary cross-track error,
 // it must request a new kinematic route instead of holding forever.
 {
 LocalMap m = make_map(160, 160, 0.1, -4.0, -8.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 0.749;
 est.y_m = 0.0;
 est.yaw_rad = -M_PI_2;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {
 {0.0, 0.0, -M_PI_2, 1},
 {0.0, -1.0, -M_PI_2, 1},
 {0.0, -2.0, -M_PI_2, 1}};
 platform::ChassisLimits ack = limits;
 ack.max_curvature_inv_m = 1.2;
 DwaPlanner near_edge;
 const LocalPlan replan = near_edge.plan(est, perc, m, route, ack);
 assert(replan.route_replan_requested);
 assert(replan.has_plan && replan.best.feasible);
 assert(replan.best.forward_mps == 0.0);
 assert(replan.best.yaw_rate_radps == 0.0);
 }

 // Differential reachability is enforced in tread space. A simultaneous
 // body-v/body-w corner that would exceed either tread's one-cycle motor
 // delta is absent from every generated proposal.
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.forward_mps = 0.8;
 est.yaw_rate_radps = 0.4;
 PerceptionResult perc;
 perc.scan_ok = true;
 StraightLinePlanner global;
 Route route = global.plan(est, {4.0, 1.0, 0.0}, iface::MapSnapshot{});
 platform::ChassisLimits diff = limits;
 diff.differential_track_m = 0.5;
 diff.max_wheel_speed_mps = 2.5;
 DwaPlanner planner;
 const LocalPlan plan = planner.plan(est, perc, m, route, diff);
 assert(plan.has_plan && !plan.candidates.empty());
 const double half_track = 0.25;
 const double measured_left =
 est.forward_mps - est.yaw_rate_radps * half_track;
 const double measured_right =
 est.forward_mps + est.yaw_rate_radps * half_track;
 const double max_tread_delta = (2.0 + 8.0 * half_track) * 0.1;
 for (const auto& candidate : plan.candidates) {
 const double left = candidate.forward_mps -
 candidate.yaw_rate_radps * half_track;
 const double right = candidate.forward_mps +
 candidate.yaw_rate_radps * half_track;
 assert(std::abs(left - measured_left) <= max_tread_delta + 1e-9);
 assert(std::abs(right - measured_right) <= max_tread_delta + 1e-9);
 }
 }

 // A rollout that is clear during the nominal horizon but whose braking
 // tail reaches a wall is inadmissible at generation time.
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 fill_box(m, 1.5, -1.0, 1.7, 1.0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.forward_mps = 2.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 StraightLinePlanner global;
 Route route = global.plan(est, {4.0, 0.0, 0.0}, iface::MapSnapshot{});
 DwaPlannerConfig config;
 config.predict_time_s = 0.5;
 DwaPlanner planner(config);
 const LocalPlan plan = planner.plan(est, perc, m, route, limits);
 assert(!plan.has_plan);
 }

 // Static generation uses footprint-plus-margin inflation. The center line
 // is free, but a wall inside the 0.5 m body envelope rejects commands whose
 // rollout and braking tail would enter it before the validator intervenes.
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 fill_box(m, 0.65, -5.0, 0.75, 5.0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 PerceptionResult perc;
 perc.scan_ok = true;
 StraightLinePlanner global;
 Route route = global.plan(est, {4.0, 0.0, 0.0}, iface::MapSnapshot{});
 DwaPlanner planner;
 platform::ChassisLimits wall_limits = limits;
 wall_limits.max_yaw_rate_radps = 0.001;
 const LocalPlan plan = planner.plan(est, perc, m, route, wall_limits);
 for (const auto& candidate : plan.candidates) {
 assert(candidate.forward_mps <= 0.05 + 1e-9);
 }
 }

    // Preserve statically executable proposals even when every rollout
    // overlaps a dynamic forecast. The validator needs the full set to
    // compare motion with the risk of stopping in the actor's path.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 1.0;
    est.y_m = 1.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    DynamicTrack t;
    t.id = 1;
    t.x_m = 1.5;
    t.y_m = 1.0;
    perc.tracks.push_back(t);
    StraightLinePlanner global;
    Route route = global.plan(est, {8.0, 1.0, 0.0}, iface::MapSnapshot{});
    DwaPlanner planner;
    LocalPlan p = planner.plan(est, perc, m, route, limits);
    assert(p.has_plan);
    assert(!p.candidates.empty());
  }

  // Predictor on new types: CV track extrapolates along velocity.
  {
    DynamicTrack t;
    t.id = 3;
    t.x_m = 0.0;
    t.vx_mps = 1.0;
    auto frames =
        predict_frames({predict_input_from_track(t, 0.0)}, 0.0, 1.0, 0.5, 5.0);
    assert(frames.size() == 3);
    assert(std::abs(frames[0][0].x_m - 0.0) < 1e-9);
    assert(std::abs(frames[1][0].x_m - 0.5) < 1e-9);
    assert(std::abs(frames[2][0].x_m - 1.0) < 1e-9);
  }

  // Holonomic DWA strafes: wall dead ahead, goal to the left, open flank.
  // Lateral samples exist only when the envelope allows them.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
    fill_box(m, 2.0, -5.0, 3.0, 5.0);  // wall across the forward path
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 0.0;
    est.y_m = 0.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    StraightLinePlanner global;
    Route route = global.plan(est, {0.0, 4.0, 0.0}, iface::MapSnapshot{});
    assert(route.valid);
    platform::ChassisLimits holo = limits;
    holo.max_lateral_mps = 1.0;
    DwaPlanner planner;
    LocalPlan p = planner.plan(est, perc, m, route, holo);
    assert(p.has_plan && p.best.feasible);
    assert(p.best.lateral_mps > 0.0);  // strafes left around the wall
    // Diff envelope on the same scene: no lateral sampled, ever.
    LocalPlan d = planner.plan(est, perc, m, route, limits);
    assert(d.has_plan && d.best.lateral_mps == 0.0);
    // Controller maps the strafing candidate to the holonomic space.
    DirectController ctl;
    iface::MotionCommand holo_cmd = ctl.track(p.best, est, 1'000'000);
    assert(holo_cmd.space == iface::CommandSpace::kHolonomic);
    assert(holo_cmd.lateral_mps == p.best.lateral_mps);
    assert(iface::valid_motion_command(holo_cmd, 1'000'000));
    iface::MotionCommand yaw_cmd = ctl.track(d.best, est, 1'000'000);
    assert(yaw_cmd.space == iface::CommandSpace::kYawRate);
  }

  // Terrain gate (master's StaticFeasibility doctrine): reject-all reads as
  // no admissible rollout (hold upstream); a fence gate holds the base
  // rather than entering bad terrain; empty gate is legacy exactly.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 1.0;
    est.y_m = 1.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    StraightLinePlanner global;
    Route route = global.plan(est, {8.0, 1.0, 0.0}, iface::MapSnapshot{});
    // Reject-all: every rollout collides at the first segment.
    DwaPlanner shut;
    shut.set_terrain_gate(
        [](double, double, double, double, double) { return false; });
    LocalPlan none = shut.plan(est, perc, m, route, limits);
    assert(!none.has_plan);
    // Fence at x = 1.05: any forward motion is inadmissible, turning in
    // place is not — the planner holds (v = 0) instead of crossing.
    DwaPlanner fenced;
    int calls = 0;
    bool yaw_seen_finite = false;
    fenced.set_terrain_gate([&](double x0, double, double x1, double,
                                double yaw) {
      ++calls;
      yaw_seen_finite |= std::isfinite(yaw) && std::isfinite(x0);
      return x0 <= 1.05 && x1 <= 1.05;
    });
    LocalPlan held = fenced.plan(est, perc, m, route, limits);
    assert(calls > 0 && yaw_seen_finite);  // gate sees every segment + yaw
    assert(held.has_plan && held.best.feasible);
    assert(held.best.forward_mps <= 0.0);  // hold, never cross the fence
    // Empty gate: legacy behavior bit-identical (drives, deterministic).
    DwaPlanner legacy;
    LocalPlan a = legacy.plan(est, perc, m, route, limits);
    assert(a.has_plan && a.best.forward_mps > 0.0);
  }

 // Route progress is lexicographic inside the statically feasible family.
 // Even deliberately neutral critics must not make zero velocity a local
 // optimum while an active route has reachable motion; the independent
 // validator can still reject every moving proposal and fall back to hold.
 {
 LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {{0.0, 0.0, 0.0, 1}, {3.0, 0.0, 0.0, 1}};
 DwaPlannerConfig neutral;
 neutral.weights = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
 DwaPlanner planner(neutral);
 const LocalPlan plan = planner.plan(est, perc, m, route, limits);
 assert(plan.has_plan && plan.moving_candidate_count > 0);
 assert(plan.best.forward_mps > 0.0);
 }

 // Stateful terminal capture prevents a differential base from orbiting a
 // goal while it settles yaw. Entry is confidence-bounded; a later estimate
 // may move outside that smaller entry circle without re-enabling
 // translation, but leaving the declared position tolerance releases it.
 {
 LocalMap m = make_map(120, 120, 0.1, -4.0, -4.0, 0);
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000;
 est.x_m = 1.90;
 est.y_m = 0.0;
 est.yaw_rad = 0.8;
 est.position_sigma_m = 0.05;
 est.heading_sigma_rad = 0.01;
 PerceptionResult perc;
 perc.scan_ok = true;
 Route route;
 route.valid = true;
 route.points = {{0.0, 0.0, 0.0, 1}, {1.0, 0.0, 0.0, 1},
 {2.0, 0.0, 0.0, 1}};
 DwaPlanner planner;
 const LocalPlan captured = planner.plan(est, perc, m, route, limits);
 assert(captured.has_plan);
 assert(std::abs(captured.best.forward_mps) < 1e-9);
 assert(captured.best.yaw_rate_radps < 0.0);

 est.x_m = 1.80; // outside confidence entry, inside raw tolerance
 const LocalPlan latched = planner.plan(est, perc, m, route, limits);
 assert(latched.has_plan);
 assert(std::abs(latched.best.forward_mps) < 1e-9);

 planner.reset_tracking(); // same goal was replanned after a map correction
 const LocalPlan replanned = planner.plan(est, perc, m, route, limits);
 assert(replanned.has_plan);
 assert(std::abs(replanned.best.forward_mps) < 1e-9);

 planner.reset(); // a new task must not inherit the previous goal capture
 const LocalPlan new_task = planner.plan(est, perc, m, route, limits);
 assert(new_task.has_plan);
 assert(new_task.best.forward_mps > 0.0);

 est.x_m = 1.90;
 const LocalPlan recaptured = planner.plan(est, perc, m, route, limits);
 assert(recaptured.has_plan);
 assert(std::abs(recaptured.best.forward_mps) < 1e-9);

 est.x_m = 1.70; // outside the declared terminal region: reacquire
 const LocalPlan released = planner.plan(est, perc, m, route, limits);
 assert(released.has_plan);
 assert(released.best.forward_mps > 0.0);
 }

 std::printf("dwa ok\n");
 return 0;
}
