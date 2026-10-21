#pragma once

#include "robot/autonomy/mapping/pose_graph.hpp"
#include "robot/autonomy/mapping/loop_closure.hpp"
#include "robot/runtime/observation.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sim::autonomy {

struct KeyframeTriggerConfig {
 double translation_threshold_m{0.40};
 double heading_threshold_rad{0.20};
 double maximum_interval_s{2.0};
 std::size_t maximum_keyframes{4096};
};

struct MappingSessionConfig {
 KeyframeTriggerConfig keyframes;
 LoopClosureGateConfig loop_closure;
 std::array<double, 9> odometry_information{
 400.0, 0.0, 0.0, 0.0, 400.0, 0.0, 0.0, 0.0, 400.0};
};

struct MappingSessionKeyframe {
 PoseGraphKeyframe graph;
 // The retained scan is session data. It is never copied into a published
 // navigation-map snapshot; a production recorder may replace it with a
 // content-addressed external reference.
 Scan scan;
 // Body twist at the retained scan's end time, used by the reconstruction
 // stage to restore per-return de-skew without storing frontend internals.
 EstimatedState motion;
};

struct MappingSessionUpdate {
 bool accepted{false};
 bool created_keyframe{false};
 std::uint64_t keyframe_id{0};
 std::string diagnostic;
};

// Selects bounded, replayable keyframes and creates the local odometry chain
// consumed by PoseGraphBackend. Ordinary scans remain front-end input. This
// class does not guess loop closures and does not rebuild a raster map; those
// are separate validation and reconstruction stages.
class MappingSession final {
 public:
 explicit MappingSession(MappingSessionConfig config = {});

 void reset();
 MappingSessionUpdate ingest(Pose2 pose, Scan scan, double stamp_s,
 EstimatedState motion = {});
 MappingSessionUpdate add_loop_closure(std::uint64_t from, std::uint64_t to,
 LoopClosureEvidence evidence);

 // Adopts corrected keyframe poses produced by optimizing a copy of this
 // session's graph. It is called only after the matching map revision has
 // been published, so the retained scans and raster semantics cannot diverge
 // from the poses that produced them. Identity, order and stamps are checked.
 bool apply_optimized_poses(const PoseGraphBackend& optimized,
 std::string* reason = nullptr);

 const MappingSessionConfig& config() const { return config_; }
 const std::vector<MappingSessionKeyframe>& keyframes() const { return keyframes_; }
 const PoseGraphBackend& graph() const { return graph_; }
 PoseGraphBackend& graph() { return graph_; }

 private:
 bool valid_config(std::string* reason) const;
 bool valid_scan(const Scan& scan, double stamp_s, std::string* reason) const;
 bool trigger(const Pose2& pose, double stamp_s) const;

 MappingSessionConfig config_;
 LoopClosureGate loop_gate_;
 PoseGraphBackend graph_;
 std::vector<MappingSessionKeyframe> keyframes_;
};

} // namespace sim::autonomy
