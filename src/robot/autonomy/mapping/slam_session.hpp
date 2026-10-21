#pragma once

#include "robot/autonomy/mapping/keyframe_map_rebuilder.hpp"
#include "robot/autonomy/mapping/loop_closure_detector.hpp"
#include "robot/autonomy/mapping/mapping_session.hpp"
#include "robot/autonomy/mapping/online_slam.hpp"
#include "robot/autonomy/mapping/slam_map_publication.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace sim::autonomy {

struct SlamSessionConfig {
 OnlineSlamConfig frontend;
 MappingSessionConfig session;
 KeyframeMapRebuildConfig rebuild;
 SlamMapPublicationConfig publication;
 LoopClosureDetectorConfig loop_closure;
};

struct SlamSessionUpdate {
 MappingState state{MappingState::Uninitialized};
 PoseObservation pose;
 std::shared_ptr<const runtime::MapSnapshot> published_map;
 std::uint64_t map_revision{0};
 bool keyframe_created{false};
 bool loop_closure_accepted{false};
 std::string diagnostic;
};

// End-to-end mapping composition. The frontend produces a pose and a local
// submap, the session retains bounded keyframes, verified loop closures enter
// the pose graph, and a successful solve rebuilds and publishes a complete
// immutable map revision before the frontend adopts the correction. Every
// stage keeps its own tested contract; this class only sequences them and owns
// no planner, game or simulator-truth state.
class SlamSession final {
 public:
 SlamSession(SlamSessionConfig config, runtime::MapManager& maps);

 void reset();

 SlamSessionUpdate update(const ObservationFrame& observations,
 const EstimatedState& prior, runtime::TimePoint sample_time);

 const MappingSession& session() const { return session_; }

 private:
 SlamSessionConfig config_;
 OnlineSlamMapBuilder frontend_;
 MappingSession session_;
 LoopClosureDetector detector_;
 KeyframeMapRebuilder rebuilder_;
 SlamMapPublication publisher_;
 runtime::MapManager& maps_;
};

} // namespace sim::autonomy
