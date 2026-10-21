#include "robot/executive/robot_executive.hpp"

#include <utility>

namespace sim::robot {

void RobotExecutive::report_motion(const MotionCapabilityReport& report) {
 status_.motion_capability = report.capability;
 status_.speed_cap_mps = report.speed_cap_mps;
 status_.motion_reason = report.reason;
 status_.recovery_condition = report.recovery_condition;
 status_.status_stamp_s = report.stamp_s;
 status_.task_state = tasks_.state();
}

} // namespace sim::robot
