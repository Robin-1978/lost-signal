#pragma once

#include "robot/autonomy/mapping/pose_graph.hpp"

#include <array>
#include <string>

namespace lost_signal::robot::autonomy {

struct LoopClosureGateConfig {
 double minimum_temporal_separation_s{5.0};
 // Maximum translation reported by the scan matcher. This is deliberately
 // independent of odometric keyframe separation: a long loop can close after
 // substantial drift.
 double maximum_candidate_distance_m{8.0};
 // Two-way scan support. A one-sided match can align part of a scan against
 // unrelated geometry, so an accepted loop must be explained by both scans.
 // A sparse revisit can expose only a few stable returns (for example an
 // off-path pallet seen from opposite scan headings).  Two-way verification
 // and the score-margin gate remain mandatory; the overlap gate therefore
 // uses the conservative evidence floor supported by the scanner rather
 // than requiring almost the entire scan to agree.
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

}  // namespace lost_signal::robot::autonomy
