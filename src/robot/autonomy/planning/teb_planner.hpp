#pragma once

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
 double extra_clearance_m = 0.10;
 double route_weight = 0.25;
 double smooth_weight = 0.45;
 double static_weight = 0.70;
 double dynamic_weight = 0.85;
 double accel_mps2 = 2.0;
 double decel_mps2 = 2.0;
 double lateral_accel_mps2 = 2.0;
 double yaw_accel_radps2 = 8.0;
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
