#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "core/math.hpp"

namespace mission {

struct MapDefinition;

enum class WeatherKind { Clear, Overcast, Drizzle, Fog };
enum class WorldIncidentKind { RoadClosure, FallenCargo, PowerFault };
enum class WorldActivityTransitionKind { WeatherChanged, IncidentOpened, IncidentResolved };

const char* weather_kind_id(WeatherKind kind);
const char* world_incident_kind_id(WorldIncidentKind kind);

struct WorldIncidentState {
 bool active{false};
 WorldIncidentKind kind{WorldIncidentKind::RoadClosure};
 std::string marker_id;
 sim::Vec2 position;
 double radius_m{0.65};
 double opened_at_s{0.0};
 double resolve_progress{0.0};
 int completed_count{0};
};

struct WorldActivityState {
 double local_time_hours{7.5};
 double daylight{0.35};
 WeatherKind weather{WeatherKind::Clear};
 double wetness{0.0};
 double visibility{1.0};
 double wind_mps{0.0};
 int weather_revision{0};
 WorldIncidentState incident;
};

struct WorldActivityTransition {
 WorldActivityTransitionKind kind{WorldActivityTransitionKind::WeatherChanged};
 WeatherKind from{WeatherKind::Clear};
 WeatherKind to{WeatherKind::Clear};
 WorldIncidentKind incident{WorldIncidentKind::RoadClosure};
};

// Serializable continuation point for the regional world. Delays are stored
// relative to the checkpoint so a fresh ScenarioSession can start its own
// clock at zero without changing when the next event occurs. The standard
// mt19937 stream representation preserves the deterministic event sequence.
struct WorldActivityCheckpoint {
 WorldActivityState state;
 double next_weather_delay_s{38.0};
 double next_incident_delay_s{18.0};
 int issued_incidents{0};
 std::string rng_state;
};

// Owns deterministic regional conditions for a continuous field session.
// It produces plain state and transitions; physics, sensing, rendering and UI
// each consume the same snapshot instead of running separate weather clocks.
class WorldActivitySystem {
 public:
 void reset(std::uint64_t seed);
 WorldActivityCheckpoint checkpoint(double time_s) const;
 void restore(const WorldActivityCheckpoint& checkpoint);
 std::vector<WorldActivityTransition> update(double time_s, double dt_s,
 const MapDefinition& map, sim::Vec2 robot_position, double robot_speed_mps);
 const WorldActivityState& state() const { return state_; }

 private:
 WeatherKind choose_next_weather();
 bool open_incident(double time_s, const MapDefinition& map, sim::Vec2 robot_position,
 std::vector<WorldActivityTransition>& transitions);

 std::mt19937 rng_{0};
 WorldActivityState state_;
 double next_weather_s_{38.0};
 double next_incident_s_{18.0};
 int issued_incidents_{0};
 double local_time_origin_hours_{7.5};
};

} // namespace mission
