#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/observation.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::eval {

// One offline step. Truth is recorded here for post-run comparison, scoring,
// and RL labels. It must never be passed to publish_observation(), tick(),
// or any ROBOT input.
struct EvalStep {
  std::uint64_t step = 0;
  world::WorldTruth truth_before{};
  world::WorldTruth truth_after{};
  robot::iface::ObservationFrame obs{};
  bool obs_accepted = false;
  robot::iface::MotionCommand cmd{};
  robot::app::RobotApplication::Diagnostics diag{};
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
