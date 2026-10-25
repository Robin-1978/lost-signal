#pragma once

#include <array>
#include <vector>

#include "robot/autonomy/planning/local_planner.hpp"

namespace lost_signal::robot::autonomy {

// Lightweight timed elastic band for the native autonomy chain. It optimizes
// a short route-aligned sequence in x/y/time against smoothness, static
// clearance and the shared dynamic prediction batch, then publishes only the
// first chassis-reachable command. The independent validator still owns the
// final safety verdict.
struct TebPlannerConfig {
 int samples = 12;
 int iterations = 24;
 double horizon_s = 2.5;
 double control_dt_s = 0.1;
 double terminal_position_tolerance_m = 0.25;
 double min_band_dt_s = 0.08;
 double max_band_dt_s = 0.40;
 double spatial_step_cap_m = 0.08;
 double extra_clearance_m = platform::kDefaultNavigationClearanceMarginM;
 // Linear preference outside the lethal footprint-plus-clearance layer.
 double soft_inflation_radius_m =
 platform::kDefaultNavigationSoftInflationRadiusM;
 double route_weight = 0.25;
 double smooth_weight = 0.45;
 double static_weight = 0.70;
 double dynamic_weight = 0.85;
 double accel_mps2 = 2.0;
 double decel_mps2 = 2.0;
 double lateral_accel_mps2 = 2.0;
 double yaw_accel_radps2 = 8.0;
 // The optimized geometry yields a family of time-scaled, actuator-reachable
 // first commands. The common validator selects among them.
 std::array<double, 3> candidate_speed_scales{1.0, 0.75, 0.5};
 // Reuse the preceding optimized band when its start still agrees with the
 // current estimate. This preserves temporal continuity across cycles.
 double warm_start_blend = 0.35;
 double warm_start_max_error_m = 0.5;
 // Optimize center/left/right route homotopies. Side seeds are offset by a
 // chassis-derived clearance envelope and ramp in over a finite distance.
 bool topology_enabled = true;
 double topology_lateral_clearance_scale = 1.25;
 double topology_transition_m = 1.0;
};

struct TebBandVertex {
 core::Vec2 position;
 double time_s = 0.0;
};

class TebPlanner : public ILocalPlanner {
 public:
 explicit TebPlanner(TebPlannerConfig config = {});

 void reset() override;
 LocalPlan plan(const EstimatedState& est,
 const PerceptionResult& perception,
 const PredictionBatch& prediction,
 const LocalMap& map, const Route& route,
 const platform::ChassisLimits& limits) override;
 void set_terrain_gate(TerrainGate gate) override { gate_ = std::move(gate); }

 const std::vector<TebBandVertex>& last_band() const { return last_band_; }

 private:
 TebPlannerConfig config_;
 TerrainGate gate_;
 std::vector<TebBandVertex> last_band_;
 double previous_lateral_mps_ = 0.0;
};

} // namespace lost_signal::robot::autonomy
