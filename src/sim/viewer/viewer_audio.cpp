#include "sim/viewer/viewer_audio.hpp"

#include <cmath>
#include <stdexcept>

namespace lost_signal::sim::viewer {

ViewerAudio::ViewerAudio(const std::filesystem::path& assets_root,
 bool enabled, ViewerAudioConfig config) : config_(config) {
 if (!std::isfinite(config_.robot_clear_stable_s) ||
 config_.robot_clear_stable_s < 0.0 ||
 !std::isfinite(config_.sim_step_s) || !(config_.sim_step_s > 0.0)) {
 throw std::invalid_argument("invalid viewer audio configuration");
 }
 if (!enabled) return;
 if (!IsAudioDeviceReady()) {
 InitAudioDevice();
 owns_device_ = IsAudioDeviceReady();
 }
 ready_ = IsAudioDeviceReady();
 if (!ready_) return;
 const auto audio = assets_root / "audio";
 collision_ = load(audio / "collision.wav", 0.65f);
 robot_hold_ = load(audio / "decision_wait.wav", 0.46f);
 robot_clear_ = load(audio / "decision_clear.wav", 0.42f);
 mission_complete_ = load(audio / "relay_restored.wav", 0.55f);
}

ViewerAudio::~ViewerAudio() {
 if (!ready_) return;
 unload(collision_);
 unload(robot_hold_);
 unload(robot_clear_);
 unload(mission_complete_);
 if (owns_device_) CloseAudioDevice();
}

ViewerAudio::Cue ViewerAudio::load(
 const std::filesystem::path& path, float volume) {
 Cue cue;
 if (!std::filesystem::is_regular_file(path)) return cue;
 cue.sound = LoadSound(path.string().c_str());
 cue.loaded = IsSoundValid(cue.sound);
 if (cue.loaded) SetSoundVolume(cue.sound, volume);
 return cue;
}

void ViewerAudio::play(const Cue& cue) const {
 if (ready_ && cue.loaded) PlaySound(cue.sound);
}

void ViewerAudio::unload(Cue& cue) {
 if (cue.loaded) UnloadSound(cue.sound);
 cue = {};
}

void ViewerAudio::reset() {
 has_frame_ = false;
 last_step_ = 0;
 collision_active_ = false;
 robot_hold_active_ = false;
 clear_candidate_step_ = 0;
 clear_candidate_valid_ = false;
 terminal_played_ = false;
}

void ViewerAudio::update(const Frame& frame, bool terminal, bool successful,
 bool turbo) {
 if (!ready_) return;
 if (has_frame_ && frame.step < last_step_) reset();

 const bool collision = frame.evaluation.static_collision ||
 frame.evaluation.dynamic_collision || frame.evaluation.terrain_collision;
 const bool stopped = frame.decision.valid &&
 frame.decision.safety_state == "STOPPED";

 // Turbo preserves fixed simulation steps but intentionally suppresses a
 // compressed burst of cues. Synchronize edge state so returning to normal
 // playback does not replay history.
 if (turbo) {
 collision_active_ = collision;
 robot_hold_active_ = stopped;
 clear_candidate_valid_ = false;
 terminal_played_ = terminal;
 last_step_ = frame.step;
 has_frame_ = true;
 return;
 }

 if (collision && !collision_active_) play(collision_);
 collision_active_ = collision;

 if (stopped) {
 clear_candidate_valid_ = false;
 if (!robot_hold_active_) play(robot_hold_);
 robot_hold_active_ = true;
 } else if (robot_hold_active_) {
 if (!clear_candidate_valid_) {
 clear_candidate_step_ = frame.step;
 clear_candidate_valid_ = true;
 }
 const double stable_s = frame.step >= clear_candidate_step_
 ? static_cast<double>(frame.step - clear_candidate_step_) *
 config_.sim_step_s : 0.0;
 if (stable_s >= config_.robot_clear_stable_s) {
 play(robot_clear_);
 robot_hold_active_ = false;
 clear_candidate_valid_ = false;
 }
 }

 if (terminal && successful && !terminal_played_) play(mission_complete_);
 if (terminal) terminal_played_ = true;
 last_step_ = frame.step;
 has_frame_ = true;
}

} // namespace lost_signal::sim::viewer
