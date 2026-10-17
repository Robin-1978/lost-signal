#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <unordered_map>

#include <raylib.h>

namespace mission {
class ScenarioSession;
}

namespace client {

// Presentation-only sound cues derived from the same mission events and
// decision evidence used by headless output. This class exists only in the
// graphical target; the simulation and headless runner never initialize audio.
class AudioDirector {
 public:
 AudioDirector(const std::filesystem::path& assets, bool enabled);
 ~AudioDirector();

 AudioDirector(const AudioDirector&) = delete;
 AudioDirector& operator=(const AudioDirector&) = delete;

 void update(const mission::ScenarioSession& mission);

 private:
 struct Cue {
 Sound sound{};
 bool loaded{false};
 };

 struct EventBinding {
 std::string cue;
 double cooldown_s{0.0};
 double last_play_s{-1000.0};
 };

 struct DecisionTransition {
 std::string action_code;
 std::string enter_cue;
 std::string exit_cue;
 double exit_stable_s{0.0};
 };

 Cue load(const std::filesystem::path& path) const;
 void play(const std::string& cue_id) const;
 void reset(const mission::ScenarioSession& mission);

 bool ready_{false};
 std::unordered_map<std::string, Cue> cues_;
 std::unordered_map<std::string, EventBinding> event_bindings_;
 DecisionTransition decision_transition_;
 const mission::ScenarioSession* mission_{nullptr};
 std::size_t events_seen_{0};
 std::size_t decisions_seen_{0};
 double last_time_s_{-1.0};
 std::string last_action_;
 bool transition_active_{false};
 double clear_candidate_s_{-1.0};
};

} // namespace client
