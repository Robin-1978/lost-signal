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

bool RobotExecutive::request_map(std::string map_id, std::string request_id) {
 if (map_id.empty())
 return false;
 pending_map_request_ = MapSelectionRequest{
 std::move(request_id), std::move(map_id)};
 status_.map_request_id = pending_map_request_->request_id;
 status_.map_request_reason = "pending";
 return true;
}

void RobotExecutive::report_map_selection(
 std::string map_id, bool accepted, std::string reason) {
 if (accepted)
 status_.active_map_id = std::move(map_id);
 status_.map_request_reason = accepted ? "accepted" : std::move(reason);
 pending_map_request_.reset();
 status_.map_request_id.clear();
}

} // namespace sim::robot
