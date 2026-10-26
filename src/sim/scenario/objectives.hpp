#pragma once

// Scenario goal driver (SIM orchestration, never ROBOT input). Sequences an
// objective list through the SAME request_goal seam an operator uses; the
// ROBOT map provider resolves every place_id, SIM never hands coordinates
// to the robot. ROBOT Done comes only from its estimate. SIM independently
// records whether the physical body arrived; only both together pass an
// objective. This verdict remains in scenario/eval and is never fed back into
// a ROBOT tick.

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::scenario {

struct ObjectiveResult {
 std::string place_id;
 bool robot_done = false; // ROBOT reported Done through the goal seam
 bool truth_arrived = false; // full SIM acceptance contract satisfied
 double truth_miss_m = 999.0; // truth distance at completion
 double truth_yaw_error_rad = 999.0;
 double truth_linear_speed_mps = 999.0;
 double truth_yaw_rate_radps = 999.0;
 std::uint32_t truth_stable_steps = 0;
 bool abandoned = false; // step budget expired before any report
 std::vector<std::string> failure_reasons;
 bool successful() const {
  return robot_done && truth_arrived && !abandoned;
 }
};

class ObjectiveDriver {
 public:
 void set_objectives(std::vector<std::string> place_ids,
  std::optional<double> arrive_tol_m = std::nullopt);
 bool set_acceptance(eval::ObjectiveAcceptanceConfig acceptance);
 const eval::ObjectiveAcceptanceConfig& acceptance() const {
  return acceptance_;
 }
 void clear();
 // Per-objective step budget (world steps): a live goal older than this
 // is abandoned (recorded, canceled through the seam, next dispatched).
 // 0 = no budget; the mission budget remains the outer backstop.
 void set_step_budget(std::uint64_t steps) { step_budget_ = steps; }
 std::uint64_t step_budget() const { return step_budget_; }
 // One orchestration step (after the app tick): requests the next place
 // when idle, records the result when the ROBOT reports completion.
 void update(robot::app::RobotApplication& app, const world::World& world);

 bool done() const { return next_ >= objectives_.size(); }
 std::size_t total() const { return objectives_.size(); }
 std::size_t completed() const { return results_.size(); }
 // Processing every request and passing the scenario are different: ROBOT
 // can report Done from a drifted estimate, while SIM must independently
 // verify the true pose and motion stability.
 bool passed() const;
 std::size_t successful() const;
 double completion_ratio() const;
 std::size_t truth_arrivals() const;
 const std::vector<ObjectiveResult>& results() const { return results_; }
 const std::string& active_place() const;

 private:
 const world::NamedPlace* find_place(const world::World& world,
  const std::string& place_id) const;
 bool requesting_ = false; // a goal is live through the ROBOT seam
 std::vector<std::string> objectives_;
 std::size_t next_ = 0;
 std::vector<ObjectiveResult> results_;
 eval::ObjectiveAcceptanceConfig acceptance_;
 std::uint32_t truth_stable_steps_ = 0;
 std::uint64_t step_budget_ = 0;
 std::uint64_t dispatched_step_ = 0; // world step at dispatch
 robot::iface::GoalHandle live_handle_{}; // canceled on abandon
};

} // namespace lost_signal::sim::scenario
