#pragma once

// Scenario goal driver (SIM orchestration, never ROBOT input). Sequences an
// objective list through the SAME request_goal seam an operator uses; the
// ROBOT map provider resolves every place_id, SIM never hands coordinates
// to the robot. ROBOT Done comes only from its estimate. SIM independently
// records whether the physical body arrived; only both together pass an
// objective. This verdict remains in scenario/eval and is never fed back into
// a ROBOT tick.

#include <cstddef>
#include <string>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::scenario {

struct ObjectiveResult {
  std::string place_id;
  bool robot_done = false;      // ROBOT reported Done through the goal seam
  bool truth_arrived = false;   // truth body within tolerance at completion
  double truth_miss_m = 999.0;  // truth distance to the place at completion
  bool abandoned = false;       // step budget expired before any report
 bool successful() const {
 return robot_done && truth_arrived && !abandoned;
 }
};

class ObjectiveDriver {
 public:
  void set_objectives(std::vector<std::string> place_ids,
                      double arrive_tol_m = 0.5);
  void clear();
  // Per-objective step budget (world steps): a live goal older than this
  // is abandoned (recorded, canceled through the seam, next dispatched).
  // 0 = no budget (today's behavior exactly: the mission budget backstops).
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
 // verify that the physical body reached the authored place.
 bool passed() const;
 std::size_t successful() const;
 double completion_ratio() const;
  std::size_t truth_arrivals() const;
  const std::vector<ObjectiveResult>& results() const { return results_; }
  const std::string& active_place() const;

 private:
  const world::NamedPlace* find_place(const world::World& world,
                                     const std::string& place_id) const;
  bool requesting_ = false;  // a goal is live through the ROBOT seam
  std::vector<std::string> objectives_;
  std::size_t next_ = 0;
  std::vector<ObjectiveResult> results_;
  double arrive_tol_m_ = 0.5;
  std::uint64_t step_budget_ = 0;
  std::uint64_t dispatched_step_ = 0;  // world step at dispatch
  robot::iface::GoalHandle live_handle_{};  // canceled on abandon
};

}  // namespace lost_signal::sim::scenario
