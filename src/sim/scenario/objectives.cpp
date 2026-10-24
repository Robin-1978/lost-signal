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
  live_handle_ = robot::iface::GoalHandle{};
  dispatched_step_ = 0;
}

std::size_t ObjectiveDriver::truth_arrivals() const {
  std::size_t n = 0;
  for (const auto& r : results_) {
    if (r.truth_arrived) ++n;
  }
  return n;
}

std::size_t ObjectiveDriver::successful() const {
 std::size_t n = 0;
 for (const auto& r : results_) {
 if (r.successful()) ++n;
 }
 return n;
}

bool ObjectiveDriver::passed() const {
 return done() && results_.size() == objectives_.size() &&
 successful() == objectives_.size();
}

double ObjectiveDriver::completion_ratio() const {
 return objectives_.empty()
 ? 1.0
 : static_cast<double>(successful()) /
 static_cast<double>(objectives_.size());
}

const std::string& ObjectiveDriver::active_place() const {
  static const std::string kNone;
  if (!requesting_ || next_ >= objectives_.size()) return kNone;
  return objectives_[next_];
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
    const auto handle = app.request_goal(req);
    if (!handle.valid()) {
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
    live_handle_ = handle;
    dispatched_step_ = world.truth().step;
    return;
  }
  // A goal is live: only the ROBOT's report advances the mission — unless
  // the step budget expired first (abandon: record, cancel, move on so one
  // stuck objective cannot wedge the mission; the mission budget stays the
  // outer backstop). A terminal report on the expiry step wins over
  // abandon (the robot did finish in time).
  const auto state = app.current_goal().state;
  if (state != robot::iface::GoalState::kDone &&
      state != robot::iface::GoalState::kFailed &&
      state != robot::iface::GoalState::kCanceled) {
    if (step_budget_ > 0 &&
        world.truth().step - dispatched_step_ > step_budget_) {
      app.cancel_goal(live_handle_);  // withdraw; a late Done must not
                                      // leak into the next objective
      ObjectiveResult r;
      r.place_id = objectives_[next_];
      r.abandoned = true;
      if (const world::NamedPlace* place = find_place(world, r.place_id)) {
        r.truth_miss_m =
            std::hypot(world.truth().body_x_m - place->x_m,
                       world.truth().body_y_m - place->y_m);
        r.truth_arrived = r.truth_miss_m <= arrive_tol_m_;
      }
      results_.push_back(r);
      ++next_;
      requesting_ = false;
      live_handle_ = robot::iface::GoalHandle{};
    }
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
