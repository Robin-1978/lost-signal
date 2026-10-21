#include "robot/autonomy/pipeline/robot_motion_runtime.hpp"

namespace sim {
namespace autonomy {

void RobotMotionRuntime::reset() {
 held_command_ = {};
 manual_command_ = {};
 manual_control_active_ = false;
 motion_sequence_ = 0;
 actuator_feedback_sequence_ = 0;
 command_sequence_ = 0;
 crossing_execution_segment_ = 0;
 crossing_alignment_segment_ = 0;
 crossing_segment_alignment_complete_ = false;
}

} // namespace autonomy
} // namespace sim
