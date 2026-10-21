#pragma once

#include "robot/autonomy/mapping/pose_graph.hpp"

#include <array>
#include <string>

namespace sim::autonomy {

struct LoopClosureGateConfig {
 double minimum_temporal_separation_s{5.0};
 double maximum_candidate_distance_m{8.0};
 double minimum_overlap{0.45};
 double minimum_score_margin{0.05};
};

struct LoopClosureEvidence {
 double match_score{0.0};
 double overlap{0.0};
 // Difference from the next spatially separated scan-match hypothesis.
 double score_margin{0.0};
 Pose2 measured_relative_pose;
 std::array<double, 9> information{1.0, 0.0, 0.0,
 0.0, 1.0, 0.0,
 0.0, 0.0, 1.0};
};

struct LoopClosureDecision {
 bool accepted{false};
 std::string diagnostic;
};

// A loop candidate must be geometrically supported before it can influence a
// pose graph. This gate is deliberately independent of actor labels, map
// semantics and the optimizer; a scan matcher supplies the evidence and the
// graph backend only receives accepted constraints.
class LoopClosureGate final {
 public:
 explicit LoopClosureGate(LoopClosureGateConfig config = {})
 : config_(config) {}

 LoopClosureDecision validate(const PoseGraphKeyframe& from,
 const PoseGraphKeyframe& to, const LoopClosureEvidence& evidence) const;

 private:
 LoopClosureGateConfig config_;
};

} // namespace sim::autonomy
