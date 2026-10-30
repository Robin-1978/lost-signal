#include "sim/viewer/viewer_audio.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lost_signal::sim::viewer {

ViewerAudio::ViewerAudio(const std::filesystem::path& assets_root,
 bool enabled, ViewerAudioConfig config)
 : config_(config),
 physical_audio_(PhysicalAudioConfig{config.sim_step_s}) {
 if (!std::isfinite(config_.robot_clear_stable_s) ||
 config_.robot_clear_stable_s < 0.0 ||
 !std::isfinite(config_.sim_step_s) || !(config_.sim_step_s > 0.0) ||
 !std::isfinite(config_.physical_loop_minimum_gain) ||
 config_.physical_loop_minimum_gain < 0.0 ||
 config_.physical_loop_minimum_gain > 1.0 ||
 !std::isfinite(config_.environment_loop_gain) ||
 config_.environment_loop_gain < 0.0 ||
 config_.environment_loop_gain > 1.0) {
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
 motor_ = load_loop(audio / "motor_drive.wav", 0.30f);
 rolling_ = load_loop(audio / "wheel_roll.wav", 0.22f);
 slip_ = load_loop(audio / "tire_slip.wav", 0.32f);
 brake_ = load_loop(audio / "service_brake.wav", 0.28f);
 environment_ = load_loop(audio / "environment_bed.wav",
                          static_cast<float>(config_.environment_loop_gain));
}

ViewerAudio::~ViewerAudio() {
 if (!ready_) return;
 unload(collision_);
 unload(robot_hold_);
 unload(robot_clear_);
 unload(mission_complete_);
 unload(motor_);
 unload(rolling_);
 unload(slip_);
 unload(brake_);
 unload(environment_);
 if (owns_device_) CloseAudioDevice();
}

ViewerAudio::Cue ViewerAudio::load(
 const std::filesystem::path& path, float volume) {
 Cue cue;
 if (!std::filesystem::is_regular_file(path)) return cue;
 cue.sound = LoadSound(path.string().c_str());
 cue.loaded = IsSoundValid(cue.sound);
 cue.base_volume = volume;
 if (cue.loaded) SetSoundVolume(cue.sound, cue.base_volume);
 return cue;
}

ViewerAudio::Loop ViewerAudio::load_loop(
 const std::filesystem::path& path, float volume) {
 Loop loop;
 if (!std::filesystem::is_regular_file(path)) return loop;
 loop.music = LoadMusicStream(path.string().c_str());
 loop.loaded = IsMusicValid(loop.music);
 loop.base_volume = volume;
 if (loop.loaded) {
 loop.music.looping = true;
 SetMusicVolume(loop.music, 0.0f);
 }
 return loop;
}

void ViewerAudio::play(const Cue& cue, double gain) const {
 if (!ready_ || !cue.loaded) return;
 SetSoundVolume(cue.sound, cue.base_volume * static_cast<float>(
 std::clamp(gain, 0.0, 1.0)));
 PlaySound(cue.sound);
}

void ViewerAudio::update_loop(Loop& loop, double gain, double pitch,
 bool suppress_audio) {
 if (!ready_ || !loop.loaded) return;
 UpdateMusicStream(loop.music);
 const double bounded_gain = std::clamp(gain, 0.0, 1.0);
 if (suppress_audio || bounded_gain < config_.physical_loop_minimum_gain) {
 if (IsMusicStreamPlaying(loop.music)) StopMusicStream(loop.music);
 return;
 }
 SetMusicVolume(loop.music,
 loop.base_volume * static_cast<float>(bounded_gain));
 SetMusicPitch(loop.music, static_cast<float>(std::clamp(pitch, 0.25, 4.0)));
 if (!IsMusicStreamPlaying(loop.music)) PlayMusicStream(loop.music);
}

void ViewerAudio::unload(Cue& cue) {
 if (cue.loaded) UnloadSound(cue.sound);
 cue = {};
}

void ViewerAudio::unload(Loop& loop) {
 if (loop.loaded) UnloadMusicStream(loop.music);
 loop = {};
}

void ViewerAudio::stop_physical_loops() {
 if (!ready_) return;
 for (Loop* loop : {&motor_, &rolling_, &slip_, &brake_, &environment_}) {
 if (loop->loaded && IsMusicStreamPlaying(loop->music)) {
 StopMusicStream(loop->music);
 }
 }
}

void ViewerAudio::reset() {
 physical_audio_.reset();
 stop_physical_loops();
 has_frame_ = false;
 last_step_ = 0;
 robot_hold_active_ = false;
 clear_candidate_step_ = 0;
 clear_candidate_valid_ = false;
 terminal_played_ = false;
}

void ViewerAudio::update(const Frame& frame, bool terminal, bool successful,
 bool suppress_audio) {
 const bool rewound = has_frame_ && frame.step < last_step_;
 if (rewound) {
 stop_physical_loops();
 has_frame_ = false;
 robot_hold_active_ = false;
 clear_candidate_step_ = 0;
 clear_candidate_valid_ = false;
 terminal_played_ = false;
 }
 const PhysicalAudioSample& physical = physical_audio_.advance(frame);

 const bool stopped = frame.decision.valid &&
 frame.decision.safety_state == "STOPPED";

 // Turbo, pause, and non-running Experience phases intentionally suppress a
 // compressed burst of sounds. The pure physical reducer above still sees
 // every completed SIM frame, so returning to live playback cannot replay
 // historical impacts or start from stale wheel/contact state.
 if (suppress_audio || !ready_) {
 stop_physical_loops();
 robot_hold_active_ = stopped;
 clear_candidate_valid_ = false;
 terminal_played_ = terminal;
 last_step_ = frame.step;
 has_frame_ = true;
 return;
 }

 update_loop(motor_, physical.motor_intensity,
 0.70 + 0.75 * physical.wheel_speed_ratio, false);
 update_loop(rolling_, physical.rolling_intensity,
 0.75 + 0.70 * physical.wheel_speed_ratio, false);
 update_loop(slip_, physical.slip_intensity,
 0.85 + 0.30 * physical.slip_intensity, false);
 update_loop(brake_, physical.brake_intensity,
 0.90 + 0.20 * physical.brake_intensity, false);
 // This bed has no gameplay or physical meaning. It is intentionally
 // separate from evidence-driven loops and muted during pause, rewind/Turbo.
 update_loop(environment_, 1.0, 1.0, false);
 if (physical.impact_event) {
 play(collision_, physical.impact_intensity);
 }

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
