#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

#include <raylib.h>

namespace mission {
class MissionSimulation;
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

 void update(const mission::MissionSimulation& mission);

 private:
 struct Cue {
 Sound sound{};
 bool loaded{false};
 };

 Cue load(const std::filesystem::path& path) const;
 static void play(const Cue& cue);
 void reset(const mission::MissionSimulation& mission);

 bool ready_{false};
 Cue power_pickup_;
 Cue traffic_warning_;
 Cue decision_wait_;
 Cue decision_clear_;
 Cue relay_restored_;
 Cue collision_;
 const mission::MissionSimulation* mission_{nullptr};
 std::size_t events_seen_{0};
 std::size_t decisions_seen_{0};
 double last_time_s_{-1.0};
 std::string last_action_;
 bool wait_cue_active_{false};
 double clear_candidate_s_{-1.0};
 double last_warning_s_{-1000.0};
};

} // namespace client
