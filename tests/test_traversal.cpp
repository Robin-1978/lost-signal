// ROBOT traversal beliefs gate: survey-grade math + the roughness belief
// model. All inputs are the surveyed elevation prior and LSRI motion
// evidence; no SIM truth appears (boundary gate enforces it structurally).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "robot/autonomy/mapping/terrain_math.hpp"
#include "robot/autonomy/mapping/traversal.hpp"

using namespace lost_signal;

// Surveyed 0.1 ramp over x in [2, 6], 40x10 cells at 0.25 m.
robot::iface::MapSnapshot ramp_map() {
  robot::iface::MapSnapshot map;
  map.map_id = "test-ramp";
  map.revision = 7;
  map.width = 40;
  map.height = 10;
  map.resolution_m = 0.25;
  map.origin_x_m = 0.0;
  map.origin_y_m = 0.0;
  map.frame_id = "map";
  map.occupancy.assign(400, -1);
  map.elevation_m.assign(400, 0.0f);
  map.elevation_sigma_m.assign(400, 0.0f);
  for (std::uint32_t gy = 0; gy < 10; ++gy) {
    for (std::uint32_t gx = 0; gx < 40; ++gx) {
      const double cx = (gx + 0.5) * 0.25;
      const double h = std::clamp((cx - 2.0) * 0.1, 0.0, 0.4);
      map.elevation_m[gy * 40 + gx] = static_cast<float>(h);
    }
  }
  return map;
}

robot::autonomy::EstimatedState moving(double x, double speed) {
  robot::autonomy::EstimatedState est;
  est.valid = true;
  est.attitude_valid = true;
  est.x_m = x;
  est.y_m = 1.0;
  est.yaw_rad = 0.0;
  est.forward_mps = speed;
  est.roll_rad = 0.0;
  est.pitch_rad = 0.0;
  return est;
}

int main() {
  // Grade math on the surveyed prior.
  {
    const auto map = ramp_map();
    assert(map.has_elevation());
    const double g =
        robot::autonomy::survey_longitudinal_grade(map, 4.0, 1.0, 0.0);
    assert(std::abs(g - 0.1) < 0.02);
    assert(std::abs(robot::autonomy::survey_lateral_grade(map, 4.0, 1.0,
                                                          0.0)) < 0.02);
    assert(std::abs(robot::autonomy::survey_longitudinal_grade(map, 0.5, 1.0,
                                                               0.0)) < 0.02);
    robot::iface::MapSnapshot bare;  // no raster: flat assumption
    assert(robot::autonomy::survey_longitudinal_grade(bare, 4.0, 1.0, 0.0) ==
           0.0);
    // Ground classifier: ramp crossing at sensor height is ground, a prism
    // top above the sensor is not, unknown never is.
    assert(robot::autonomy::endpoint_is_survey_ground(map, 4.0, 1.0, 0.25,
                                                      0.15));
    assert(!robot::autonomy::endpoint_is_survey_ground(map, 4.0, 1.0, 0.9,
                                                       0.15));
    assert(!robot::autonomy::endpoint_is_survey_ground(bare, 4.0, 1.0, 0.25,
                                                       0.15));
  }

  // Rollover envelope + bump speed limit (closed forms, hand-checked).
  {
    using robot::autonomy::static_rollover_angle;
    using robot::autonomy::rollover_risk;
    assert(std::abs(static_rollover_angle(0.5, 0.25) - 0.7853981634) < 1e-6);
    assert(std::abs(rollover_risk(1.0, 0.5, 0.25) - 1.0) < 1e-6);
    assert(rollover_risk(0.0, 0.5, 0.25) == 0.0);
    assert(std::isinf(
        robot::autonomy::roughness_ride_speed_limit(0.0, 0.435, 0.3)));
    const double lim =
        robot::autonomy::roughness_ride_speed_limit(0.02, 0.435, 0.3);
    assert(std::isfinite(lim) && lim > 0.3 && lim < 5.0);
    // Rougher ground is slower, monotonically.
    assert(robot::autonomy::roughness_ride_speed_limit(0.08, 0.435, 0.3) <
           lim);
  }

  // Belief model: unbound reads the prior, feasible, finite.
  {
    robot::autonomy::TraversalModel model;
    assert(!model.ready());
    double conf = -1.0;
    assert(model.roughness_at(0.0, 0.0, &conf) == 0.02 && conf == 0.0);
    assert(model.feasible(0.0, 0.0, 5.0, 0.0, 0.0));
    assert(model.cost(0.0, 0.0, 5.0, 0.0, 0.0).feasible);
  }

  // Motion evidence forms the belief: bump excitation latches up fast,
  // smooth running revises back down, stale evidence expires to the prior,
  // standstill never observes.
  {
    robot::autonomy::TraversalModel model;
    model.bind(ramp_map());
    assert(model.ready() && model.revision() == 7);
    double conf = 0.0;
    assert(model.roughness_at(1.0, 1.0, &conf) == 0.02 && conf == 0.0);
    // Bumpy minutes: 2 s of +2 m/s^2 vertical excitation at 1 m/s.
    for (int k = 1; k <= 100; ++k) {
      model.observe_motion(moving(1.0, 1.0), 9.81 + 6.0, k * 0.02, 0.02);
    }
    double rough = model.roughness_at(1.0, 1.0, &conf);
    assert(rough > 0.02 && conf > 0.5);
    // Smooth running revises down (slower than the rise).
    for (int k = 101; k <= 200; ++k) {
      model.observe_motion(moving(1.0, 1.0), 9.81, k * 0.02, 0.02);
    }
    double calmer = model.roughness_at(1.0, 1.0, nullptr);
    assert(calmer < rough);
    // Stale evidence expires to the prior (clock advances on another cell).
    model.observe_motion(moving(7.0, 1.0), 9.81, 100.0, 0.02);
    assert(model.roughness_at(1.0, 1.0, &conf) == 0.02 && conf == 0.0);
    // Standstill never observes, however violent the IMU.
    robot::autonomy::TraversalModel still;
    still.bind(ramp_map());
    for (int k = 1; k <= 50; ++k) {
      still.observe_motion(moving(2.0, 0.0), 9.81 + 5.0, k * 0.02, 0.02);
    }
    assert(still.roughness_at(2.0, 1.0, nullptr) == 0.02);
  }

  // Cost ordering + feasibility: rough costs more than smooth, cliffs and
  // side-slopes refuse with infinite cost.
  {
    robot::autonomy::TraversalModel model;
    model.bind(ramp_map());
    for (int k = 1; k <= 100; ++k) {
      model.observe_motion(moving(1.0, 1.0), 9.81 + 6.0, k * 0.02, 0.02);
    }
    const auto rough_seg = model.cost(0.5, 1.0, 1.5, 1.0, 0.0);
    const auto smooth_seg = model.cost(7.0, 1.0, 8.0, 1.0, 0.0);
    assert(rough_seg.feasible && smooth_seg.feasible);
    // Short rollout steps across a cell boundary on the gentle grade stay
    // feasible: the grade reads through the survey-resolution window, so
    // nearest-cell stairsteps never alias into phantom cliffs (DWA steps
    // at ~0.1 s cadence; regression pin for the icy-climb stall).
    for (double x = 2.0; x < 5.0; x += 0.05) {
      assert(model.feasible(x, 1.0, x + 0.05, 1.0, 0.0));
    }
    assert(rough_seg.cost > smooth_seg.cost);
    // Cliff: 1 m rise over 1 m run refuses.
    robot::iface::MapSnapshot cliff = ramp_map();
    cliff.elevation_m[4 * 40 + 16] = 1.0f;
    robot::autonomy::TraversalModel cliff_model;
    cliff_model.bind(cliff);
    const double cx = (16 + 0.5) * 0.25, px = (15 + 0.5) * 0.25;
    auto no = cliff_model.cost(px, 1.0, cx, 1.0, 0.0);
    assert(!no.feasible && std::isinf(no.cost));
    assert(!cliff_model.feasible(px, 1.0, cx, 1.0, 0.0));
  }

  std::printf("traversal ok\n");
  return 0;
}
