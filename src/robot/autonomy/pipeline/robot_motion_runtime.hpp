#pragma once

#include "robot/platform/differential_kinematics.hpp"
#include "robot/autonomy/planning/local/local_planning_system.hpp"

#include <functional>
#include <vector>

#include <cstddef>
#include <cstdint>
#include <string>

namespace sim {
class AutonomyPipeline;
namespace autonomy {

// Provider-neutral local-planning transaction. The composition root may build
// the observation and map views, but the planning call itself belongs to
// ROBOT. Pointers are intentionally non-owning and valid only for the call;
// they avoid copying a large local map on every control tick.
struct RobotLocalPlanningInput {
 const LocalPlanningRequest* request{nullptr};
 const GridMap* planning_map{nullptr};
 const std::vector<std::vector<DynamicObjectState>>* predicted_frames{nullptr};
 StaticFeasibility terrain_feasible;
 const GridMap* physical_collision_map{nullptr};
 StaticFeasibility physical_terrain_feasible;
 TerrainSampleQuery terrain_sample;
};

// Deployable state for the motion transaction.  This state is deliberately
// independent of a mission, level, actor, physics body, or renderer.  A SIM
// composition root and a native robot process use the same object; only the
// provider adapter around it changes.
class RobotMotionRuntime {
 public:
  void reset();

  DiffCmd& held_command() { return held_command_; }
  const DiffCmd& held_command() const { return held_command_; }
  DiffCmd& manual_command() { return manual_command_; }
  const DiffCmd& manual_command() const { return manual_command_; }

  bool& manual_control_active() { return manual_control_active_; }
  bool manual_control_active() const { return manual_control_active_; }

  std::uint64_t& motion_sequence() { return motion_sequence_; }
  std::uint64_t& actuator_feedback_sequence() { return actuator_feedback_sequence_; }
  std::uint64_t& command_sequence() { return command_sequence_; }

  std::size_t& crossing_execution_segment() { return crossing_execution_segment_; }
  std::size_t& crossing_alignment_segment() { return crossing_alignment_segment_; }
  bool& crossing_segment_alignment_complete() {
    return crossing_segment_alignment_complete_;
  }

 // The runtime owns the typed local-planning transaction. It deliberately
 // accepts only ROBOT map, perception and chassis contracts; SIM providers
 // stay outside this boundary.
 LocalPlanningOutput plan_local(
  AutonomyPipeline& pipeline, const RobotLocalPlanningInput& input) const;

 private:
  DiffCmd held_command_{};
  DiffCmd manual_command_{};
  bool manual_control_active_{false};
  std::uint64_t motion_sequence_{0};
  std::uint64_t actuator_feedback_sequence_{0};
  std::uint64_t command_sequence_{0};
  std::size_t crossing_execution_segment_{0};
  std::size_t crossing_alignment_segment_{0};
  bool crossing_segment_alignment_complete_{false};
};

} // namespace autonomy
} // namespace sim
