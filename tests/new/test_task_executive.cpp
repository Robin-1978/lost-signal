#include "robot/executive/task_executive.hpp"

#include <iostream>
#include <limits>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 executive::TaskExecutive tasks;
 executive::TaskPlan plan{
 "operator-request-7",
 executive::TaskOrdering::Autonomous,
 {
 {"inspect-a", {1.0, 0.0}, 0.4, false, true, 0.10, 0.18},
 {"collect-b", {2.0, 0.0}, 0.4, false, false, 0.0, 1.0},
 {"return-home", {0.0, 0.0}, 0.5, true, false, 0.0, 0.2},
 },
 };
 CHECK(tasks.accept(plan));
 CHECK(tasks.state() == executive::TaskState::Active);
 CHECK(tasks.eligible_goals() == std::vector<std::size_t>({0, 1}));
 CHECK(!tasks.activate(2));
 CHECK(tasks.activate(0));
 CHECK(!tasks.update({{1.0, 0.0}, 0.3, true, 0.02}));
 CHECK(!tasks.update({{1.0, 0.0}, 0.0, true, 0.06}));
 const auto inspected = tasks.update({{1.0, 0.0}, 0.0, true, 0.04});
 CHECK(inspected && inspected->goal_id == "inspect-a");
 CHECK(tasks.pause());
 CHECK(!tasks.activate(1));
 CHECK(tasks.resume());
 CHECK(tasks.activate(1));
 CHECK(tasks.update({{2.0, 0.0}, 0.2, true, 0.02}));
 CHECK(tasks.eligible_goals() == std::vector<std::size_t>({2}));
 CHECK(tasks.activate(2));
 CHECK(!tasks.update({{0.0, 0.0}, 0.3, true, 0.02}));
 const auto home = tasks.update({{0.0, 0.0}, 0.0, true, 0.02});
 CHECK(home && home->task_complete);
 CHECK(tasks.state() == executive::TaskState::Succeeded);

 executive::TaskExecutive cancelled;
 CHECK(cancelled.accept(plan));
 CHECK(cancelled.cancel());
 CHECK(cancelled.state() == executive::TaskState::Cancelled);
 CHECK(!cancelled.activate(0));

 const auto reduced = tasks.localization_directive(
 {{1.0, 0.0}, 0.0, true, 0.0, true, 0.50, 0.0});
 CHECK(reduced.response == executive::LocalizationResponse::ReduceSpeed);

 const auto nominal = tasks.localization_directive(
 {{1.0, 0.0}, 0.0, true, 0.0, true, 0.90, 0.0});
 CHECK(nominal.response == executive::LocalizationResponse::Continue);

 const auto relocalize = tasks.localization_directive(
 {{1.0, 0.0}, 0.0, true, 0.0, true, 0.50, 2.5});
 CHECK(relocalize.response ==
 executive::LocalizationResponse::HoldForRelocalization);

 const auto held = tasks.localization_directive(
 {{1.0, 0.0}, 0.0, true, 0.0, false, 0.90, 0.0});
 CHECK(held.response == executive::LocalizationResponse::StopAndHold);

 const auto stale = tasks.localization_directive(
 {{1.0, 0.0}, 0.0, true, 0.0, true, 0.90, 5.5});
 CHECK(stale.response == executive::LocalizationResponse::StopAndHold);

 const auto invalid_confidence = tasks.localization_directive(
 {{1.0, 0.0}, 0.0, true, 0.0, true,
 std::numeric_limits<double>::quiet_NaN(), 0.0});
 CHECK(invalid_confidence.response ==
 executive::LocalizationResponse::StopAndHold);

 auto invalid = plan;
 invalid.goals[1].id = invalid.goals[0].id;
 CHECK(!cancelled.accept(std::move(invalid)));
 return 0;
}
