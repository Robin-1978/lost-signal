#pragma once

#include "core/math.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace executive {

enum class TaskOrdering { Authored, Autonomous };
enum class TaskState { Idle, Active, Paused, Succeeded, Failed, Cancelled };

// Middleware-neutral operator intent after an API/UI adapter has resolved
// named places into the active map frame. Scenario actions and presentation
// data are deliberately absent.
struct TaskGoal {
 std::string id;
 sim::Vec2 position;
 double radius_m{0.0};
 bool terminal{false};
 bool requires_stop{false};
 double dwell_s{0.0};
 double max_arrival_speed_mps{0.0};
};

struct TaskPlan {
 std::string request_id;
 TaskOrdering ordering{TaskOrdering::Authored};
 std::vector<TaskGoal> goals;
};

struct TaskObservation {
 sim::Vec2 position;
 double speed_mps{0.0};
 bool completion_allowed{true};
 double dt_s{0.0};
};

struct TaskArrival {
 std::size_t index{0};
 std::string goal_id;
 bool task_complete{false};
};

// Deployable task lifecycle above autonomy. It accepts semantic goals,
// selects/advances them, and reports completion. It cannot command motion or
// observe simulator truth; a caller gives the estimated robot observation and
// passes the active goal to planning.
class TaskExecutive {
 public:
 bool accept(TaskPlan plan);
 void clear();
 bool pause();
 bool resume();
 bool cancel();
 bool fail();

 TaskState state() const { return state_; }
 const TaskPlan& plan() const { return plan_; }
 std::size_t goals_reached() const { return completed_count_; }
 std::optional<std::size_t> active_goal_index() const { return active_goal_; }
 bool activate(std::size_t index);
 bool abandon(std::size_t index);
 bool completed(std::size_t index) const;
 bool abandoned(std::size_t index) const;
 std::vector<std::size_t> eligible_goals() const;
 std::vector<std::string> completed_goal_ids() const;
 std::vector<std::string> abandoned_goal_ids() const;
 std::optional<TaskArrival> update(const TaskObservation& observation);

 private:
 TaskPlan plan_;
 TaskState state_{TaskState::Idle};
 std::vector<bool> completed_;
 std::vector<bool> abandoned_;
 std::vector<std::size_t> completion_order_;
 std::vector<std::size_t> abandonment_order_;
 std::optional<std::size_t> active_goal_;
 std::size_t completed_count_{0};
 double dwell_s_{0.0};
};

} // namespace executive
