#pragma once

#include <cstdint>
#include <filesystem>

#include <raylib.h>

#include "sim/viewer/frame.hpp"
#include "sim/viewer/physical_audio.hpp"

namespace lost_signal::sim::viewer {

struct ViewerAudioConfig {
 double robot_clear_stable_s = 0.8;
 double sim_step_s = 0.02;
 // Mix controls only. This floor prevents inaudible streams from occupying
 // audio voices; it does not gate or alter the physical evidence state.
 double physical_loop_minimum_gain = 0.015;
 // Environment bed is presentation-only and deliberately kept below the
 // evidence-driven mechanical buses.
 double environment_loop_gain = 0.10;
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
 bool suppress_audio);
 bool ready() const { return ready_; }
 const PhysicalAudioSample& physical_sample() const {
 return physical_audio_.sample();
 }

 private:
 struct Cue {
 Sound sound{};
 bool loaded = false;
 float base_volume = 1.0f;
 };

 struct Loop {
 Music music{};
 bool loaded = false;
 float base_volume = 1.0f;
 };

 Cue load(const std::filesystem::path& path, float volume);
 Loop load_loop(const std::filesystem::path& path, float volume);
 void play(const Cue& cue, double gain = 1.0) const;
 void update_loop(Loop& loop, double gain, double pitch,
 bool suppress_audio);
 void unload(Cue& cue);
 void unload(Loop& loop);
 void stop_physical_loops();

 ViewerAudioConfig config_;
 PhysicalAudioEvidence physical_audio_;
 bool owns_device_ = false;
 bool ready_ = false;
 Cue collision_;
 Cue robot_hold_;
 Cue robot_clear_;
 Cue mission_complete_;
 Loop motor_;
 Loop rolling_;
 Loop slip_;
 Loop brake_;
 Loop environment_;
 bool has_frame_ = false;
 std::uint64_t last_step_ = 0;
 bool robot_hold_active_ = false;
 std::uint64_t clear_candidate_step_ = 0;
 bool clear_candidate_valid_ = false;
 bool terminal_played_ = false;
};

} // namespace lost_signal::sim::viewer
