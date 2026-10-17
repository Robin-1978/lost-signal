#include "audio_director.hpp"

#include "sim/game/scenario/scenario_session.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

namespace client {

AudioDirector::AudioDirector(const std::filesystem::path& assets, bool enabled) {
 if (!enabled)
 return;
 InitAudioDevice();
 ready_ = IsAudioDeviceReady();
 if (!ready_)
 return;

 const auto audio = assets / "audio";
 std::ifstream input(audio / "cues.json");
 const nlohmann::json config = nlohmann::json::parse(input);
 for (const auto& item : config.at("cues")) {
 const auto id = item.at("id").get<std::string>();
 cues_.emplace(id, load(assets / item.at("file").get<std::string>()));
 }
 for (const auto& item : config.at("events")) {
 for (const auto& code : item.at("codes")) {
 event_bindings_.emplace(code.get<std::string>(),
 EventBinding{item.at("cue").get<std::string>(),
 item.at("cooldown_s").get<double>(), -1000.0});
 }
 }
 const auto& transition = config.at("decision_transition");
 decision_transition_ = {
 transition.at("action_code").get<std::string>(),
 transition.at("enter_cue").get<std::string>(),
 transition.at("exit_cue").get<std::string>(),
 transition.at("exit_stable_s").get<double>(),
 };
}

AudioDirector::~AudioDirector() {
 if (!ready_)
 return;
 for (auto& [id, cue] : cues_) {
 (void)id;
 if (cue.loaded)
 UnloadSound(cue.sound);
 }
 CloseAudioDevice();
}

AudioDirector::Cue AudioDirector::load(const std::filesystem::path& path) const {
 Cue cue;
 cue.sound = LoadSound(path.string().c_str());
 cue.loaded = IsSoundValid(cue.sound);
 return cue;
}

void AudioDirector::play(const std::string& cue_id) const {
 const auto cue = cues_.find(cue_id);
 if (cue != cues_.end() && cue->second.loaded)
 PlaySound(cue->second.sound);
}

void AudioDirector::reset(const mission::ScenarioSession& mission) {
 mission_ = &mission;
 events_seen_ = 0;
 decisions_seen_ = 0;
 last_time_s_ = mission.snapshot().time_s;
 last_action_.clear();
 transition_active_ = false;
 clear_candidate_s_ = -1.0;
 for (auto& [code, binding] : event_bindings_) {
 (void)code;
 binding.last_play_s = -1000.0;
 }
}

void AudioDirector::update(const mission::ScenarioSession& mission) {
 if (!ready_)
 return;
 if (mission_ != &mission || mission.snapshot().time_s + 1e-9 < last_time_s_ ||
 events_seen_ > mission.events().size() || decisions_seen_ > mission.decisions().size()) {
 reset(mission);
 }

 for (; events_seen_ < mission.events().size(); ++events_seen_) {
 const auto& event = mission.events()[events_seen_];
 const auto found = event_bindings_.find(event.code);
 if (found != event_bindings_.end() &&
 event.time_s - found->second.last_play_s >= found->second.cooldown_s) {
 play(found->second.cue);
 found->second.last_play_s = event.time_s;
 }
 }

 for (; decisions_seen_ < mission.decisions().size(); ++decisions_seen_) {
 last_action_ = mission.decisions()[decisions_seen_].action_code;
 }

 // Planner output may alternate between wait and a feasible rollout for a
 // few ticks at a boundary. Latch the audible wait episode until non-waiting
 // has remained stable, so true state changes are heard without cue chatter.
 const bool transition_action = last_action_ == decision_transition_.action_code;
 if (transition_action) {
 clear_candidate_s_ = -1.0;
 if (!transition_active_) {
 play(decision_transition_.enter_cue);
 transition_active_ = true;
 }
 } else if (transition_active_) {
 if (clear_candidate_s_ < 0.0)
 clear_candidate_s_ = mission.snapshot().time_s;
 else if (mission.snapshot().time_s - clear_candidate_s_ >= decision_transition_.exit_stable_s) {
 play(decision_transition_.exit_cue);
 transition_active_ = false;
 clear_candidate_s_ = -1.0;
 }
 }
 last_time_s_ = mission.snapshot().time_s;
}

} // namespace client
