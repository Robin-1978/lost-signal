#include "mission/systems/world_activity_system.hpp"
#include "mission/content.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 mission::WorldActivitySystem first;
 mission::WorldActivitySystem second;
 mission::MapDefinition map;
 map.markers.emplace("a", mission::Pose{10.0, 4.0, 0.0});
 map.markers.emplace("b", mission::Pose{16.0, 8.0, 0.0});
 first.reset(20261016);
 second.reset(20261016);

 int changes = 0;
 bool became_wet = false;
 for (int tick = 1; tick <= 9000; ++tick) {
 const double time_s = tick * 0.02;
 const auto a = first.update(time_s, 0.02, map, {1.0, 1.0}, 0.5);
 const auto b = second.update(time_s, 0.02, map, {1.0, 1.0}, 0.5);
 CHECK(a.size() == b.size());
 CHECK(first.state().weather == second.state().weather);
 CHECK(std::abs(first.state().wetness - second.state().wetness) < 1e-12);
 CHECK(std::abs(first.state().visibility - second.state().visibility) < 1e-12);
 CHECK(first.state().local_time_hours >= 0.0 && first.state().local_time_hours < 24.0);
 CHECK(first.state().daylight >= 0.0 && first.state().daylight <= 1.0);
 for (const auto& transition : a)
 changes += transition.kind == mission::WorldActivityTransitionKind::WeatherChanged ? 1 : 0;
 became_wet = became_wet || first.state().wetness > 0.05;
 }
 CHECK(changes >= 2);
 CHECK(became_wet);
 CHECK(first.state().weather_revision == changes);
 CHECK(first.state().incident.active);
 const auto incident_position = first.state().incident.position;
 bool resolved = false;
 for (int tick = 0; tick < 80; ++tick) {
 const double time_s = 180.0 + (tick + 1) * 0.02;
 const auto transitions = first.update(time_s, 0.02, map, incident_position, 0.0);
 for (const auto& transition : transitions)
 resolved = resolved ||
 transition.kind == mission::WorldActivityTransitionKind::IncidentResolved;
 }
 CHECK(resolved);
 CHECK(!first.state().incident.active);
 CHECK(first.state().incident.completed_count == 1);
 CHECK(std::string(mission::weather_kind_id(mission::WeatherKind::Drizzle)) == "drizzle");
 CHECK(std::string(mission::world_incident_kind_id(mission::WorldIncidentKind::FallenCargo)) ==
 "fallen-cargo");

 // A restored regional session must continue the same weather/event stream
 // even though the new MissionSimulation clock starts again at zero.
 mission::WorldActivitySystem uninterrupted;
 mission::WorldActivitySystem resumed;
 uninterrupted.reset(778899);
 for (int tick = 1; tick <= 3250; ++tick)
 uninterrupted.update(tick * 0.02, 0.02, map, {1.0, 1.0}, 0.5);
 const double checkpoint_time_s = 65.0;
 const auto checkpoint = uninterrupted.checkpoint(checkpoint_time_s);
 CHECK(!checkpoint.rng_state.empty());
 resumed.restore(checkpoint);
 for (int tick = 1; tick <= 5000; ++tick) {
 const auto direct = uninterrupted.update(checkpoint_time_s + tick * 0.02, 0.02,
 map, {1.0, 1.0}, 0.5);
 const auto continued = resumed.update(tick * 0.02, 0.02, map, {1.0, 1.0}, 0.5);
 CHECK(direct.size() == continued.size());
 CHECK(uninterrupted.state().weather == resumed.state().weather);
 CHECK(std::abs(uninterrupted.state().local_time_hours - resumed.state().local_time_hours) < 1e-10);
 CHECK(std::abs(uninterrupted.state().wetness - resumed.state().wetness) < 1e-10);
 CHECK(uninterrupted.state().incident.active == resumed.state().incident.active);
 CHECK(uninterrupted.state().incident.kind == resumed.state().incident.kind);
 CHECK(uninterrupted.state().incident.marker_id == resumed.state().incident.marker_id);
 }
 return 0;
}
