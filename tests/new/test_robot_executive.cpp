#include "robot/executive/robot_executive.hpp"

#include <iostream>
#include <utility>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::robot::RobotExecutive executive;
 executive::TaskPlan plan;
 plan.request_id = "robot-executive-test";
 plan.goals.push_back({"goal", {1.0, 2.0}, 0.25, true, true, 0.1, 0.2});
 CHECK(executive.submit(std::move(plan)));
 CHECK(executive.status().task_state == executive::TaskState::Active);

 executive.report_motion({sim::robot::MotionCapability::Reduced, 0.2,
 "localization reduced", "relocalize", 4.0});
 CHECK(executive.status().motion_capability == sim::robot::MotionCapability::Reduced);
 CHECK(executive.status().speed_cap_mps == 0.2);
 CHECK(executive.status().task_state == executive::TaskState::Active);

 CHECK(executive.pause());
 CHECK(executive.status().task_state == executive::TaskState::Paused);
 executive.report_motion({sim::robot::MotionCapability::Hold, 0.0,
 "paused", "resume", 5.0});
 CHECK(executive.status().motion_capability == sim::robot::MotionCapability::Hold);
 CHECK(executive.resume());
 CHECK(executive.task_kernel().state() == executive::TaskState::Active);
 return 0;
}
