#pragma once

// Scenario goal driver (SIM orchestration, never ROBOT input). Sequences an
// objective list through the SAME request_goal seam an operator uses; the
// ROBOT map provider resolves every place_id, SIM never hands coordinates
// to the robot. Arrival/completion is counted truth-side for scoring (did
// the body actually get there?) but never fed back into any tick: the
// driver only requests the next goal when the ROBOT reports the active one
// done/failed, and only reads truth for the offline record.

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
};

class ObjectiveDriver {
 public:
  void set_objectives(std::vector<std::string> place_ids,
                      double arrive_tol_m = 0.5);
  void clear();
  // One orchestration step (after the app tick): requests the next place
  // when idle, records the result when the ROBOT reports completion.
  void update(robot::app::RobotApplication& app, const world::World& world);

  bool done() const { return next_ >= objectives_.size(); }
  std::size_t completed() const { return results_.size(); }
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
};

}  // namespace lost_signal::sim::scenario
