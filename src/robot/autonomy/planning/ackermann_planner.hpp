#pragma once

#include <cstddef>

#include "robot/autonomy/planning/global_planner.hpp"

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
 bool allow_reverse = true;
 double reverse_cost = 1.35;
 double gear_change_cost = 0.8;
 double goal_position_tolerance_m = 0.45;
 double goal_heading_tolerance_rad = 0.8;
 int max_expansions = 120000;
 // Bound memory on large survey rasters. The planner reduces heading
 // resolution to fit this budget and fails closed if even the minimum
 // heading discretization cannot fit.
 std::size_t max_states = 4'000'000;
 };

 AckermannPlanner();
 explicit AckermannPlanner(Config config);

 Route plan(const EstimatedState& start, const RoutePoint& goal,
 const iface::MapSnapshot& map) override;
 void set_traversal(const TraversalModel* model) override {
 traversal_ = model;
 }

 private:
 Config config_;
 const TraversalModel* traversal_ = nullptr;
};

} // namespace lost_signal::robot::autonomy
