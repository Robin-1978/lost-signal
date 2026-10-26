#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/motion.hpp"
#include "robot/iface/observation.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::eval {

// One offline step. Truth is recorded here for post-run comparison, scoring,
// and RL labels. It must never be passed to publish_observation(), tick(),
// or any ROBOT input.
struct CollisionOutcome {
  bool static_collision = false;
  bool dynamic_collision = false;
  double static_clearance_m = 999.0;   // robot-center-to-box minus radius
  double dynamic_clearance_m = 999.0;  // center-to-center minus both radii
  // Belly / high-center contact (engine truth-side, offline only): the
  // authored surface under the footprint nears the engine body height.
  bool terrain_collision = false;
  double terrain_clearance_m = 999.0;  // body_z minus surface minus clearance
};

// Lightweight SIM-only state needed to reproduce a historical presentation
// frame. Static geometry remains in World; only dynamic engine/actor state is
// copied. This record is never published to RobotApplication.
struct ActorPresentationState {
 double x_m = 0.0;
 double y_m = 0.0;
 double vx_mps = 0.0;
 double vy_mps = 0.0;
 double radius_m = 0.4;
 double height_m = 1.5;
 bool pedestrian = false;
};

struct PresentationSnapshot {
 bool captured = false;
 std::vector<world::WheelViz> wheels;
 std::vector<world::CasterViz> casters;
 std::vector<ActorPresentationState> actors;
 world::ImpactViz impact;
};

struct EvalStep {
  std::uint64_t step = 0;
  world::WorldTruth truth_before{};
  world::WorldTruth truth_after{};
  robot::iface::ObservationFrame obs{};
  bool obs_accepted = false;
  robot::iface::MotionCommand cmd{};
 // Exact deployable actuator output consumed by the plant. Keeping it next
 // to the body command makes watchdog stops, closed-loop duty and physical
 // brake effects auditable without exposing SIM truth to ROBOT.
 robot::iface::ActuatorCommand actuator{};
  robot::app::RobotApplication::Diagnostics diag{};
  // Truth-side judge outcome (SIM orchestration fills it; scoring reads it).
  CollisionOutcome judge{};
 // Historical rendering evidence from this same completed physics step.
 PresentationSnapshot presentation{};
};

// Append-only episode log, SIM-owned. Export is an offline artifact
// (JSON/JSONL file or RL transition batch), consumed after the run.
class EpisodeRecorder {
 public:
  void record(EvalStep step) { steps_.push_back(step); }
  void clear() { steps_.clear(); }
  std::size_t size() const { return steps_.size(); }
  const std::vector<EvalStep>& steps() const { return steps_; }

 private:
  std::vector<EvalStep> steps_;
};

}  // namespace lost_signal::sim::eval
