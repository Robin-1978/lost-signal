#include "sim/scenario/objectives.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lost_signal::sim::scenario {
namespace {

bool valid_acceptance(const eval::ObjectiveAcceptanceConfig& config) {
 return config.version == 1 &&
  std::isfinite(config.position_tolerance_m) &&
  config.position_tolerance_m > 0.0 &&
  std::isfinite(config.heading_tolerance_rad) &&
  config.heading_tolerance_rad > 0.0 &&
  std::isfinite(config.maximum_linear_speed_mps) &&
  config.maximum_linear_speed_mps > 0.0 &&
  std::isfinite(config.maximum_yaw_rate_radps) &&
  config.maximum_yaw_rate_radps > 0.0 &&
  config.stable_steps > 0;
}

double wrapped_abs_angle(double angle_rad) {
 return std::abs(std::remainder(angle_rad, 2.0 * 3.14159265358979323846));
}

void append_truth_failures(ObjectiveResult& result,
 const world::NamedPlace* place, const eval::ObjectiveAcceptanceConfig& config) {
 if (place == nullptr) {
  result.failure_reasons.push_back("truth_place_missing");
  return;
 }
 if (result.truth_miss_m > config.position_tolerance_m) {
  result.failure_reasons.push_back("truth_position_outside_tolerance");
 }
 if (result.truth_yaw_error_rad > config.heading_tolerance_rad) {
  result.failure_reasons.push_back("truth_heading_outside_tolerance");
 }
 if (result.truth_linear_speed_mps > config.maximum_linear_speed_mps) {
  result.failure_reasons.push_back("truth_linear_speed_above_limit");
 }
 if (result.truth_yaw_rate_radps > config.maximum_yaw_rate_radps) {
  result.failure_reasons.push_back("truth_yaw_rate_above_limit");
 }
 if (result.truth_stable_steps < config.stable_steps) {
  result.failure_reasons.push_back("truth_stability_window_incomplete");
 }
}

bool measure_truth(ObjectiveResult& result, const world::NamedPlace* place,
 const world::World& world, const eval::ObjectiveAcceptanceConfig& config,
 std::uint32_t stable_steps) {
 result.truth_stable_steps = stable_steps;
 if (place == nullptr) return false;
 const auto& truth = world.truth();
 result.truth_miss_m = std::hypot(
  truth.body_x_m - place->x_m, truth.body_y_m - place->y_m);
 result.truth_yaw_error_rad = wrapped_abs_angle(
  truth.body_yaw_rad - place->yaw_rad);
 result.truth_linear_speed_mps = std::hypot(
  truth.forward_mps, truth.lateral_mps);
 result.truth_yaw_rate_radps = std::abs(truth.yaw_rate_radps);
 return result.truth_miss_m <= config.position_tolerance_m &&
  result.truth_yaw_error_rad <= config.heading_tolerance_rad &&
  result.truth_linear_speed_mps <= config.maximum_linear_speed_mps &&
  result.truth_yaw_rate_radps <= config.maximum_yaw_rate_radps;
}

} // namespace

bool ObjectiveDriver::set_acceptance(
 eval::ObjectiveAcceptanceConfig acceptance) {
 if (!valid_acceptance(acceptance)) return false;
 acceptance_ = acceptance;
 truth_stable_steps_ = 0;
 return true;
}

void ObjectiveDriver::set_objectives(std::vector<std::string> place_ids,
 std::optional<double> arrive_tol_m) {
 objectives_ = std::move(place_ids);
 if (arrive_tol_m.has_value() && std::isfinite(*arrive_tol_m) &&
  *arrive_tol_m > 0.0) {
  acceptance_.position_tolerance_m = *arrive_tol_m;
 }
 next_ = 0;
 results_.clear();
 requesting_ = false;
 truth_stable_steps_ = 0;
}

void ObjectiveDriver::clear() {
 objectives_.clear();
 next_ = 0;
 results_.clear();
 requesting_ = false;
 truth_stable_steps_ = 0;
 live_handle_ = robot::iface::GoalHandle{};
 dispatched_step_ = 0;
}

std::size_t ObjectiveDriver::truth_arrivals() const {
 std::size_t n = 0;
 for (const auto& result : results_) {
  if (result.truth_arrived) ++n;
 }
 return n;
}

std::size_t ObjectiveDriver::successful() const {
 std::size_t n = 0;
 for (const auto& result : results_) {
  if (result.successful()) ++n;
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
 for (const auto& place : world.places()) {
  if (place.place_id == place_id) return &place;
 }
 return nullptr;
}

void ObjectiveDriver::update(robot::app::RobotApplication& app,
 const world::World& world) {
 if (done()) return;
 if (!requesting_) {
  robot::iface::TaskRequest request;
  request.request_id = "scenario-objective-" + std::to_string(next_);
  request.place_id = objectives_[next_];
  const auto handle = app.request_goal(request);
  if (!handle.valid()) {
   ObjectiveResult result;
   result.place_id = objectives_[next_];
   const auto* place = find_place(world, result.place_id);
   measure_truth(result, place, world, acceptance_, 0);
   result.failure_reasons.push_back("robot_goal_rejected");
   append_truth_failures(result, place, acceptance_);
   results_.push_back(std::move(result));
   ++next_;
   return;
  }
  requesting_ = true;
  live_handle_ = handle;
  dispatched_step_ = world.truth().step;
  truth_stable_steps_ = 0;
  return;
 }

 const auto* place = find_place(world, objectives_[next_]);
 ObjectiveResult sampled;
 sampled.place_id = objectives_[next_];
 const bool acceptable_now = measure_truth(
  sampled, place, world, acceptance_, truth_stable_steps_);
 if (acceptable_now) {
  if (truth_stable_steps_ < std::numeric_limits<std::uint32_t>::max()) {
   ++truth_stable_steps_;
  }
 } else {
  truth_stable_steps_ = 0;
 }
 measure_truth(sampled, place, world, acceptance_, truth_stable_steps_);
 sampled.truth_arrived = place != nullptr &&
  truth_stable_steps_ >= acceptance_.stable_steps;

 const auto state = app.current_goal().state;
 const bool terminal = state == robot::iface::GoalState::kDone ||
  state == robot::iface::GoalState::kFailed ||
  state == robot::iface::GoalState::kCanceled;
 if (!terminal) {
  if (step_budget_ == 0 ||
   world.truth().step - dispatched_step_ <= step_budget_) {
   return;
  }
  app.cancel_goal(live_handle_);
  sampled.abandoned = true;
  sampled.failure_reasons.push_back("objective_step_budget_exceeded");
  sampled.failure_reasons.push_back("robot_not_done");
  append_truth_failures(sampled, place, acceptance_);
 } else {
  sampled.robot_done = state == robot::iface::GoalState::kDone;
  if (state == robot::iface::GoalState::kFailed) {
   sampled.failure_reasons.push_back("robot_goal_failed");
  } else if (state == robot::iface::GoalState::kCanceled) {
   sampled.failure_reasons.push_back("robot_goal_canceled");
  }
  if (!sampled.robot_done) {
   sampled.failure_reasons.push_back("robot_not_done");
  }
  append_truth_failures(sampled, place, acceptance_);
 }
 results_.push_back(std::move(sampled));
 ++next_;
 requesting_ = false;
 truth_stable_steps_ = 0;
 live_handle_ = robot::iface::GoalHandle{};
}

} // namespace lost_signal::sim::scenario
