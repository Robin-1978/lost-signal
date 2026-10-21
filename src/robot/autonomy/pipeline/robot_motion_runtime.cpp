#include "robot/autonomy/pipeline/robot_motion_runtime.hpp"
#include "robot/autonomy/pipeline/autonomy_pipeline.hpp"

#include <stdexcept>

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

LocalPlanningOutput RobotMotionRuntime::plan_local(
 AutonomyPipeline& pipeline, const RobotLocalPlanningInput& input) const {
 if (!input.request || !input.planning_map || !input.predicted_frames)
 throw std::invalid_argument("robot local planning transaction has missing input");
 return pipeline.plan_local(*input.request, *input.planning_map,
 *input.predicted_frames, input.terrain_feasible,
 input.physical_collision_map, input.physical_terrain_feasible,
 input.terrain_sample);
}

} // namespace autonomy
} // namespace sim
