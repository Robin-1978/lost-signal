#pragma once

#include "robot/autonomy/mapping/loop_closure.hpp"
#include "robot/autonomy/mapping/mapping_session.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sim::autonomy {

struct LoopClosureDetectorConfig {
 // Number of query returns used for verification. A bounded subset keeps the
 // search cost independent of sensor resolution.
 std::size_t maximum_returns{96};
 double endpoint_tolerance_m{0.25};
 double index_cell_m{0.50};
 // Local search around the odometry-predicted relative transform.
 double search_translation_m{0.60};
 double translation_step_m{0.10};
 double search_heading_rad{0.12};
 double heading_step_rad{0.03};
 // Two solving poses closer than this describe one physical hypothesis.
 double ambiguity_distance_m{0.30};
 double ambiguity_heading_rad{0.10};
 // Odometric proximity used only to decide which keyframes are worth a scan
 // verification attempt. A long loop can close after metres of drift.
 double candidate_search_distance_m{12.0};
 double minimum_temporal_separation_s{5.0};
 std::size_t maximum_candidates{8};
 // Coarse hypotheses are refined on the same scan evidence before ranking.
 double refine_translation_m{0.80};
 double refine_translation_step_m{0.10};
 double refine_heading_rad{0.15};
 double refine_heading_step_rad{0.03};
 std::size_t refined_hypotheses{3};
};

struct LoopClosureCandidate {
 std::uint64_t from_keyframe{0};
 std::uint64_t to_keyframe{0};
 LoopClosureEvidence evidence;
 std::string diagnostic;
};

// Verifies a loop closure by matching the newest keyframe scan against retained
// keyframe scans. The gate in `LoopClosureGate` still decides acceptance; this
// component only produces measured evidence, and it never invents a constraint
// from pose proximity alone.
class LoopClosureDetector final {
 public:
 explicit LoopClosureDetector(LoopClosureDetectorConfig config = {})
 : config_(config) {}

 std::optional<LoopClosureCandidate> detect(
 const std::vector<MappingSessionKeyframe>& keyframes) const;

 private:
 LoopClosureDetectorConfig config_;
};

} // namespace sim::autonomy
