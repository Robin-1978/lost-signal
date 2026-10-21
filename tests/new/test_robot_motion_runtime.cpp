#include "robot/autonomy/pipeline/robot_motion_runtime.hpp"

#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::autonomy::RobotMotionRuntime runtime;
 runtime.held_command().target_v = 0.4;
 runtime.manual_command().target_w = 0.2;
 runtime.manual_control_active() = true;
 runtime.crossing_execution_segment() = 3;
 runtime.crossing_alignment_segment() = 2;
 runtime.crossing_segment_alignment_complete() = true;
 runtime.command_sequence() = 11;
 runtime.reset();
 CHECK(runtime.held_command().target_v == 0.0);
 CHECK(runtime.manual_command().target_w == 0.0);
 CHECK(!runtime.manual_control_active());
 CHECK(runtime.crossing_execution_segment() == 0);
 CHECK(runtime.crossing_alignment_segment() == 0);
 CHECK(!runtime.crossing_segment_alignment_complete());
 CHECK(runtime.command_sequence() == 0);
 return 0;
}
