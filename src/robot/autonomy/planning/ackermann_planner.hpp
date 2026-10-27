#pragma once

#include <cstddef>
#include <memory>

#include "robot/autonomy/planning/astar_planner.hpp"

namespace lost_signal::robot::autonomy {

// Small deterministic Hybrid-A* for car-like bases. States carry position,
// heading bin, and gear; each edge is a swept bicycle primitive checked
// against the ROBOT map. It is intentionally independent of SIM truth.
class AckermannPlanner : public IGlobalPlanner {
 public:
 struct Config {
 double wheel_base_m = 0.56;
 double max_steer_rad = 0.55;
 double primitive_length_m = 0.40;
 int heading_bins = 72;
 double max_curvature_delta_per_m = 2.0;
 // Hybrid-A* control effort costs use curvature normalized by the physical
 // steering envelope. The first term discourages driving at full lock; the
 // second discourages alternating steering commands with the same path
 // length. Both are map- and scenario-independent search costs.
 double curvature_cost_per_m = 0.20;
 double curvature_change_cost_m = 0.35;
 // Search focus for the map-derived guide heuristic. The default keeps path
 // cost and clearance authoritative. Values above one are an explicit
 // bounded-suboptimal mode and may accept a higher-cost steering oscillation
 // before a smoother route reaches the same segment goal.
 double guide_heuristic_weight = 1.0;
 bool allow_reverse = true;
 double reverse_cost = 1.35;
 double gear_change_cost = 0.8;
 double goal_position_tolerance_m = 0.45;
 double goal_heading_tolerance_rad = 0.8;
 // Maximum mismatch between a terminal connector chord and the midpoint
 // tangent implied by its endpoint headings.
 double connector_heading_tolerance_rad = 0.12;
 // Optional deterministic diagnostic ceiling for one local refinement.
 // Production uses -1: route length is handled by local segments and each
 // segment is bounded by cooperative expansion work and sparse-state memory,
 // never by host CPU time.
 int max_expansions = -1;
 // Cooperative work per decision cycle. The 2D guide planner and Hybrid-A*
 // refinement each consume a bounded expansion slice.
 std::size_t expansions_per_cycle = 4096;
 // Fine Hybrid-A* stays within this map-derived corridor around the 2D
 // guide route. This bounds visited states without assuming a scenario.
 double corridor_radius_m = 6.0;
 // Shared global-costmap policy. Unknown remains traversable at a penalty;
 // only measured/map-authored occupied cells seed footprint inflation.
 double unknown_cost_per_m = 0.5;
 double clearance_cost_per_m = 1.0;
 double footprint_radius_m = 0.4;
 double hard_clearance_margin_m =
 platform::kDefaultNavigationClearanceMarginM;
 double soft_inflation_radius_m =
 platform::kDefaultNavigationSoftInflationRadiusM;
 // Bound the sparse states actually visited by Hybrid-A*. This is not a
 // full-raster cells x headings x gears allocation.
 std::size_t max_states = 4'000'000;
 };

 AckermannPlanner();
 explicit AckermannPlanner(Config config);
 ~AckermannPlanner() override;
 AckermannPlanner(AckermannPlanner&&) noexcept;
 AckermannPlanner& operator=(AckermannPlanner&&) noexcept;
 AckermannPlanner(const AckermannPlanner&) = delete;
 AckermannPlanner& operator=(const AckermannPlanner&) = delete;

 Route plan(const EstimatedState& start, const RoutePoint& goal,
 const iface::MapSnapshot& map) override;
 bool supports_incremental_plan() const override { return true; }
 std::size_t incremental_plan_budget() const override {
 return config_.expansions_per_cycle;
 }
 void begin_plan(const EstimatedState& start, const RoutePoint& goal,
 const iface::MapSnapshot& map) override;
 GlobalPlanProgress continue_plan(std::size_t expansion_budget) override;
 void cancel_plan() override;
 void set_traversal(const TraversalModel* model) override {
 traversal_ = model;
 guide_planner_.set_traversal(model);
 }

 private:
 struct SearchState;

 Config config_;
 const TraversalModel* traversal_ = nullptr;
 AStarPlanner guide_planner_;
 std::unique_ptr<SearchState> search_;
 GlobalPlanProgress completed_;
 bool completed_ready_ = false;
};

} // namespace lost_signal::robot::autonomy
