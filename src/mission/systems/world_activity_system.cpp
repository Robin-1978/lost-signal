#include "world_activity_system.hpp"
#include "mission/content.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace mission {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kIncidentResolveS = 1.2;

double move_toward(double value, double target, double amount) {
 if (value < target)
 return std::min(target, value + amount);
 return std::max(target, value - amount);
}

double target_wetness(WeatherKind weather) {
 switch (weather) {
 case WeatherKind::Clear:
 return 0.0;
 case WeatherKind::Overcast:
 return 0.18;
 case WeatherKind::Drizzle:
 return 1.0;
 case WeatherKind::Fog:
 return 0.32;
 }
 return 0.0;
}

double target_visibility(WeatherKind weather) {
 switch (weather) {
 case WeatherKind::Clear:
 return 1.0;
 case WeatherKind::Overcast:
 return 0.90;
 case WeatherKind::Drizzle:
 return 0.76;
 case WeatherKind::Fog:
 return 0.48;
 }
 return 1.0;
}

} // namespace

const char* weather_kind_id(WeatherKind kind) {
 switch (kind) {
 case WeatherKind::Clear:
 return "clear";
 case WeatherKind::Overcast:
 return "overcast";
 case WeatherKind::Drizzle:
 return "drizzle";
 case WeatherKind::Fog:
 return "fog";
 }
 return "clear";
}

const char* world_incident_kind_id(WorldIncidentKind kind) {
 switch (kind) {
 case WorldIncidentKind::RoadClosure:
 return "road-closure";
 case WorldIncidentKind::FallenCargo:
 return "fallen-cargo";
 case WorldIncidentKind::PowerFault:
 return "power-fault";
 }
 return "road-closure";
}

void WorldActivitySystem::reset(std::uint64_t seed) {
 rng_.seed(static_cast<std::uint32_t>(seed ^ (seed >> 32U)));
 state_ = {};
 next_weather_s_ = 38.0 + static_cast<double>(rng_() % 17U);
 next_incident_s_ = 18.0 + static_cast<double>(rng_() % 9U);
 issued_incidents_ = 0;
 local_time_origin_hours_ = state_.local_time_hours;
}

WorldActivityCheckpoint WorldActivitySystem::checkpoint(double time_s) const {
 std::ostringstream rng_stream;
 rng_stream << rng_;
 return {
 state_,
 std::max(0.0, next_weather_s_ - time_s),
 std::max(0.0, next_incident_s_ - time_s),
 issued_incidents_,
 rng_stream.str(),
 };
}

void WorldActivitySystem::restore(const WorldActivityCheckpoint& checkpoint) {
 if (checkpoint.next_weather_delay_s < 0.0 || checkpoint.next_incident_delay_s < 0.0 ||
 checkpoint.issued_incidents < 0 || checkpoint.state.local_time_hours < 0.0 ||
 checkpoint.state.local_time_hours >= 24.0 || checkpoint.state.wetness < 0.0 ||
 checkpoint.state.wetness > 1.0 || checkpoint.state.visibility < 0.0 ||
 checkpoint.state.visibility > 1.0) {
 throw std::invalid_argument("invalid world activity checkpoint");
 }
 std::istringstream rng_stream(checkpoint.rng_state);
 std::mt19937 restored_rng;
 if (checkpoint.rng_state.empty() || !(rng_stream >> restored_rng))
 throw std::invalid_argument("invalid world activity random state");
 rng_ = restored_rng;
 state_ = checkpoint.state;
 next_weather_s_ = checkpoint.next_weather_delay_s;
 next_incident_s_ = checkpoint.next_incident_delay_s;
 issued_incidents_ = checkpoint.issued_incidents;
 local_time_origin_hours_ = state_.local_time_hours;
}

WeatherKind WorldActivitySystem::choose_next_weather() {
 // Clear intervals remain common, but never select the current condition.
 // Fixed weights and the session seed make a shared world reproducible.
 static constexpr WeatherKind candidates[] = {
 WeatherKind::Clear,
 WeatherKind::Clear,
 WeatherKind::Overcast,
 WeatherKind::Overcast,
 WeatherKind::Drizzle,
 WeatherKind::Fog,
 };
 for (int attempt = 0; attempt < 8; ++attempt) {
 const WeatherKind candidate = candidates[rng_() % (sizeof(candidates) / sizeof(candidates[0]))];
 if (candidate != state_.weather)
 return candidate;
 }
 return state_.weather == WeatherKind::Clear ? WeatherKind::Overcast : WeatherKind::Clear;
}

bool WorldActivitySystem::open_incident(double time_s, const MapDefinition& map,
 sim::Vec2 robot_position, std::vector<WorldActivityTransition>& transitions) {
 std::vector<std::pair<std::string, Pose>> candidates;
 for (const auto& [id, pose] : map.markers) {
 if (std::hypot(pose.x - robot_position.x, pose.y - robot_position.y) >= 3.0)
 candidates.emplace_back(id, pose);
 }
 if (candidates.empty())
 return false;
 const auto& selected = candidates[rng_() % candidates.size()];
 static constexpr WorldIncidentKind kinds[] = {
 WorldIncidentKind::RoadClosure,
 WorldIncidentKind::FallenCargo,
 WorldIncidentKind::PowerFault,
 };
 auto& incident = state_.incident;
 incident.active = true;
 incident.kind = kinds[issued_incidents_ % 3];
 incident.marker_id = selected.first;
 incident.position = {selected.second.x, selected.second.y};
 incident.radius_m = incident.kind == WorldIncidentKind::RoadClosure ? 0.78 : 0.58;
 incident.opened_at_s = time_s;
 incident.resolve_progress = 0.0;
 ++issued_incidents_;
 transitions.push_back({WorldActivityTransitionKind::IncidentOpened,
 state_.weather, state_.weather, incident.kind});
 return true;
}

std::vector<WorldActivityTransition> WorldActivitySystem::update(double time_s, double dt_s,
 const MapDefinition& map, sim::Vec2 robot_position, double robot_speed_mps) {
 std::vector<WorldActivityTransition> transitions;
 // A full day lasts eight real-time minutes at 1x simulation speed.
 state_.local_time_hours = std::fmod(local_time_origin_hours_ + time_s / 20.0, 24.0);
 const double sun_phase = (state_.local_time_hours - 6.0) * kPi / 12.0;
 state_.daylight = std::clamp(std::sin(sun_phase), 0.0, 1.0);

 if (time_s + 1e-9 >= next_weather_s_) {
 const WeatherKind previous = state_.weather;
 state_.weather = choose_next_weather();
 ++state_.weather_revision;
 transitions.push_back({WorldActivityTransitionKind::WeatherChanged,
 previous, state_.weather, state_.incident.kind});
 next_weather_s_ = time_s + 42.0 + static_cast<double>(rng_() % 29U);
 }

 const double safe_dt = std::max(0.0, dt_s);
 const double wet_rate = state_.weather == WeatherKind::Drizzle ? 0.020 : 0.009;
 state_.wetness = move_toward(state_.wetness, target_wetness(state_.weather), wet_rate * safe_dt);
 state_.visibility =
 move_toward(state_.visibility, target_visibility(state_.weather), 0.10 * safe_dt);
 const double gust = 0.5 + 0.5 * std::sin(time_s * 0.17 + static_cast<double>(state_.weather_revision));
 state_.wind_mps = (state_.weather == WeatherKind::Clear ? 0.8 : 1.7) + gust * 1.4;

 auto& incident = state_.incident;
 if (!incident.active && time_s + 1e-9 >= next_incident_s_) {
 if (!open_incident(time_s, map, robot_position, transitions))
 next_incident_s_ = time_s + 8.0;
 } else if (incident.active) {
 const double range = std::hypot(robot_position.x - incident.position.x,
 robot_position.y - incident.position.y);
 if (range <= incident.radius_m + 1.0 && std::abs(robot_speed_mps) <= 0.15)
 incident.resolve_progress += safe_dt / kIncidentResolveS;
 else
 incident.resolve_progress = 0.0;
 if (incident.resolve_progress + 1e-9 >= 1.0) {
 const auto resolved_kind = incident.kind;
 incident.active = false;
 incident.resolve_progress = 0.0;
 ++incident.completed_count;
 next_incident_s_ = time_s + 26.0 + static_cast<double>(rng_() % 17U);
 transitions.push_back({WorldActivityTransitionKind::IncidentResolved,
 state_.weather, state_.weather, resolved_kind});
 }
 }
 return transitions;
}

} // namespace mission
