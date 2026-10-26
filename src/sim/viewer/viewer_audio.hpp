#pragma once

#include <cstdint>
#include <filesystem>

#include <raylib.h>

#include "sim/viewer/frame.hpp"

namespace lost_signal::sim::viewer {

struct ViewerAudioConfig {
 double robot_clear_stable_s = 0.8;
 double sim_step_s = 0.02;
};

// Presentation-only audio. Cues consume completed, typed frame evidence and
// terminal GAME state; audio state is never visible to ROBOT or physics.
class ViewerAudio {
 public:
 ViewerAudio(const std::filesystem::path& assets_root, bool enabled,
 ViewerAudioConfig config = {});
 ~ViewerAudio();

 ViewerAudio(const ViewerAudio&) = delete;
 ViewerAudio& operator=(const ViewerAudio&) = delete;

 void reset();
 void update(const Frame& frame, bool terminal, bool successful,
 bool turbo);
 bool ready() const { return ready_; }

 private:
 struct Cue {
 Sound sound{};
 bool loaded = false;
 };

 Cue load(const std::filesystem::path& path, float volume);
 void play(const Cue& cue) const;
 void unload(Cue& cue);

 ViewerAudioConfig config_;
 bool owns_device_ = false;
 bool ready_ = false;
 Cue collision_;
 Cue robot_hold_;
 Cue robot_clear_;
 Cue mission_complete_;
 bool has_frame_ = false;
 std::uint64_t last_step_ = 0;
 bool collision_active_ = false;
 bool robot_hold_active_ = false;
 std::uint64_t clear_candidate_step_ = 0;
 bool clear_candidate_valid_ = false;
 bool terminal_played_ = false;
};

} // namespace lost_signal::sim::viewer
