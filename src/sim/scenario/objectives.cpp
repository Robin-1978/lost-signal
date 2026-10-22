#include "sim/scenario/objectives.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lost_signal::sim::scenario {

void ObjectiveDriver::set_objectives(std::vector<std::string> place_ids,
                                    double arrive_tol_m) {
  objectives_ = std::move(place_ids);
  arrive_tol_m_ = std::isfinite(arrive_tol_m) && arrive_tol_m > 0.0
                      ? arrive_tol_m
                      : 0.5;
  next_ = 0;
  results_.clear();
  requesting_ = false;
}

void ObjectiveDriver::clear() {
  objectives_.clear();
  next_ = 0;
  results_.clear();
  requesting_ = false;
}

std::size_t ObjectiveDriver::truth_arrivals() const {
  std::size_t n = 0;
  for (const auto& r : results_) {
    if (r.truth_arrived) ++n;
  }
  return n;
}

const std::string& ObjectiveDriver::active_place() const {
  static const std::string kNone;
  if (!requesting_ || next_ == 0 || next_ > objectives_.size()) return kNone;
  return objectives_[next_ - 1];
}

const world::NamedPlace* ObjectiveDriver::find_place(
    const world::World& world, const std::string& place_id) const {
  for (const auto& p : world.places()) {
    if (p.place_id == place_id) return &p;
  }
  return nullptr;
}

void ObjectiveDriver::update(robot::app::RobotApplication& app,
                             const world::World& world) {
  if (done()) return;
  if (!requesting_) {
    // Idle: dispatch the next objective through the operator seam.
    robot::iface::TaskRequest req;
    req.request_id = "scenario-objective-" + std::to_string(next_);
    req.place_id = objectives_[next_];
    if (!app.request_goal(req).valid()) {
      // Unknown place (map provider fails closed): record the miss and
      // move on so one bad objective cannot wedge a headless run.
      ObjectiveResult r;
      r.place_id = objectives_[next_];
      r.truth_miss_m = 999.0;
      results_.push_back(r);
      ++next_;
      return;
    }
    requesting_ = true;
    return;
  }
  // A goal is live: only the ROBOT's report advances the mission.
  const auto state = app.current_goal().state;
  if (state != robot::iface::GoalState::kDone &&
      state != robot::iface::GoalState::kFailed &&
      state != robot::iface::GoalState::kCanceled) {
    return;
  }
  ObjectiveResult r;
  r.place_id = objectives_[next_];
  r.robot_done = (state == robot::iface::GoalState::kDone);
  if (const world::NamedPlace* place = find_place(world, r.place_id)) {
    r.truth_miss_m =
        std::hypot(world.truth().body_x_m - place->x_m,
                   world.truth().body_y_m - place->y_m);
    r.truth_arrived = r.truth_miss_m <= arrive_tol_m_;
  }
  results_.push_back(r);
  ++next_;
  requesting_ = false;
}

}  // namespace lost_signal::sim::scenario
