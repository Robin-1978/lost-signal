#include "audio_director.hpp"

#include "mission/mission_simulation.hpp"

#include <array>

namespace client {

AudioDirector::AudioDirector(const std::filesystem::path& assets, bool enabled) {
 if (!enabled)
 return;
 InitAudioDevice();
 ready_ = IsAudioDeviceReady();
 if (!ready_)
 return;

 const auto audio = assets / "audio";
 power_pickup_ = load(audio / "power_pickup.wav");
 traffic_warning_ = load(audio / "traffic_warning.wav");
 decision_wait_ = load(audio / "decision_wait.wav");
 decision_clear_ = load(audio / "decision_clear.wav");
 relay_restored_ = load(audio / "relay_restored.wav");
 collision_ = load(audio / "collision.wav");
}

AudioDirector::~AudioDirector() {
 if (!ready_)
 return;
 for (Cue* cue : std::array{
 &power_pickup_, &traffic_warning_, &decision_wait_,
 &decision_clear_, &relay_restored_, &collision_,
 }) {
 if (cue->loaded)
 UnloadSound(cue->sound);
 }
 CloseAudioDevice();
}

AudioDirector::Cue AudioDirector::load(const std::filesystem::path& path) const {
 Cue cue;
 cue.sound = LoadSound(path.string().c_str());
 cue.loaded = IsSoundValid(cue.sound);
 return cue;
}

void AudioDirector::play(const Cue& cue) {
 if (cue.loaded)
 PlaySound(cue.sound);
}

void AudioDirector::reset(const mission::MissionSimulation& mission) {
 mission_ = &mission;
 events_seen_ = 0;
 decisions_seen_ = 0;
 last_time_s_ = mission.snapshot().time_s;
 last_action_.clear();
 wait_cue_active_ = false;
 clear_candidate_s_ = -1.0;
 last_warning_s_ = -1000.0;
}

void AudioDirector::update(const mission::MissionSimulation& mission) {
 if (!ready_)
 return;
 if (mission_ != &mission || mission.snapshot().time_s + 1e-9 < last_time_s_ ||
 events_seen_ > mission.events().size() || decisions_seen_ > mission.decisions().size()) {
 reset(mission);
 }

 for (; events_seen_ < mission.events().size(); ++events_seen_) {
 const auto& code = mission.events()[events_seen_].code;
 if (code == "via-reached")
 play(power_pickup_);
 else if ((code == "dynamic-detected" || code == "emergency-escape") &&
 mission.events()[events_seen_].time_s - last_warning_s_ >= 3.0) {
 play(traffic_warning_);
 last_warning_s_ = mission.events()[events_seen_].time_s;
 }
 else if (code == "mission-complete")
 play(relay_restored_);
 else if (code == "collision-static" || code == "collision-dynamic" || code == "rollover")
 play(collision_);
 }

 for (; decisions_seen_ < mission.decisions().size(); ++decisions_seen_) {
 last_action_ = mission.decisions()[decisions_seen_].action_code;
 }

 // Planner output may alternate between wait and a feasible rollout for a
 // few ticks at a boundary. Latch the audible wait episode until non-waiting
 // has remained stable, so true state changes are heard without cue chatter.
 const bool waiting = last_action_ == "wait";
 if (waiting) {
 clear_candidate_s_ = -1.0;
 if (!wait_cue_active_) {
 play(decision_wait_);
 wait_cue_active_ = true;
 }
 } else if (wait_cue_active_) {
 if (clear_candidate_s_ < 0.0)
 clear_candidate_s_ = mission.snapshot().time_s;
 else if (mission.snapshot().time_s - clear_candidate_s_ >= 0.8) {
 play(decision_clear_);
 wait_cue_active_ = false;
 clear_candidate_s_ = -1.0;
 }
 }
 last_time_s_ = mission.snapshot().time_s;
}

} // namespace client
