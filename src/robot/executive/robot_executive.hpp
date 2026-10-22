#pragma once

#include "robot/autonomy/planning/local/local_planning_system.hpp"
#include "robot/executive/task_executive.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace sim::robot {

// Task-level view of the motion capability proved by the autonomy stack.
// This is deliberately not a velocity command. The local planner owns speed,
// braking and escape selection; the executive only consumes the result when
// deciding whether a task may continue, wait or enter recovery.
enum class MotionCapability {
 Unknown,
 Normal,
 Reduced,
 Hold,
 NoSafeAction,
};

struct MotionCapabilityReport {
 MotionCapability capability{MotionCapability::Unknown};
 double speed_cap_mps{0.0};
 std::string reason;
 std::string recovery_condition;
 double stamp_s{0.0};
};

struct RobotExecutiveStatus {
 executive::TaskState task_state{executive::TaskState::Idle};
 MotionCapability motion_capability{MotionCapability::Unknown};
 double speed_cap_mps{0.0};
 std::string motion_reason;
 std::string recovery_condition;
 double status_stamp_s{0.0};
 std::string active_map_id;
 std::string map_request_id;
 std::string map_request_reason;
};

struct MapSelectionRequest {
 std::string request_id;
 std::string map_id;
};

// Deployable task/health facade above autonomy. RobotExecutive owns task
// intent and lifecycle, but never emits a wheel, steering or actuator
// command. It consumes the typed capability result from local planning and
// leaves the motion response to the caller's task policy.
class RobotExecutive {
 public:
 bool submit(executive::TaskPlan plan) {
 const bool accepted = tasks_.accept(std::move(plan));
 refresh_task_state();
 return accepted;
 }
 void clear() { tasks_.clear(); refresh_task_state(); }
 bool pause() { const bool changed = tasks_.pause(); refresh_task_state(); return changed; }
 bool resume() { const bool changed = tasks_.resume(); refresh_task_state(); return changed; }
 bool cancel() { const bool changed = tasks_.cancel(); refresh_task_state(); return changed; }
 bool fail() { const bool changed = tasks_.fail(); refresh_task_state(); return changed; }

 std::optional<executive::TaskArrival> update(
 const executive::TaskObservation& observation) {
 auto arrival = tasks_.update(observation);
 refresh_task_state();
 return arrival;
 }

 executive::LocalizationDirective localization_directive(
 const executive::TaskObservation& observation) const {
 return tasks_.localization_directive(observation);
 }

 void report_motion(const MotionCapabilityReport& report);

 // Map selection is a resource/task request owned by the executive. The
 // application consumes it only after its safety gate and reports the result;
 // this method never mutates a MapManager or loads a file.
 bool request_map(std::string map_id, std::string request_id = {});
 std::optional<MapSelectionRequest> pending_map_request() const {
 return pending_map_request_;
 }
 void report_map_selection(std::string map_id, bool accepted, std::string reason = {});

 const RobotExecutiveStatus& status() const { return status_; }
 executive::TaskState state() const { return tasks_.state(); }
 std::size_t goals_reached() const { return tasks_.goals_reached(); }
 std::optional<std::size_t> active_goal_index() const { return tasks_.active_goal_index(); }
 bool activate(std::size_t index) { const bool changed = tasks_.activate(index); refresh_task_state(); return changed; }
 bool abandon(std::size_t index) { const bool changed = tasks_.abandon(index); refresh_task_state(); return changed; }
 bool completed(std::size_t index) const { return tasks_.completed(index); }
 bool abandoned(std::size_t index) const { return tasks_.abandoned(index); }
 std::vector<std::size_t> eligible_goals() const { return tasks_.eligible_goals(); }
 std::vector<std::string> completed_goal_ids() const { return tasks_.completed_goal_ids(); }
 std::vector<std::string> abandoned_goal_ids() const { return tasks_.abandoned_goal_ids(); }
 const executive::TaskExecutive& task_kernel() const { return tasks_; }
 executive::TaskExecutive& task_kernel() { return tasks_; }

 private:
 void refresh_task_state() { status_.task_state = tasks_.state(); }

 executive::TaskExecutive tasks_;
 RobotExecutiveStatus status_;
 std::optional<MapSelectionRequest> pending_map_request_;
};

} // namespace sim::robot
