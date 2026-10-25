#pragma once

#include <string>
#include <vector>

#include "sim/scenario/scenario_session.hpp"

namespace lost_signal::sim::scenario {

struct ActorRouteIssue {
 std::size_t actor_index = 0;
 std::size_t segment_index = 0;
 std::size_t box_index = 0;
 std::string actor_id;
};

// Validate authored starts and every waypoint segment against static volume,
// expanded by each actor's physical radius. This is scenario-authoring truth
// validation only; it is never used by ROBOT planning or safety.
bool validate_actor_routes(
 const std::vector<world::Actor>& actors,
 const std::vector<world::StaticBox>& boxes,
 ActorRouteIssue* issue = nullptr);

// Load actors from <map>.actors.json (or an explicit path) into the
// session's world. The sidecar is SIM-side setup data authored next to the
// world; it never crosses into ROBOT. Returns the number loaded, or -1 on a
// malformed or physically invalid file (a missing default sidecar means zero
// actors). Loading is transactional: validation failure adds no actors.
int load_actors_sidecar(const std::string& map_path,
 ScenarioSession& session,
 const std::string& explicit_path = {},
 std::string* error = nullptr);

} // namespace lost_signal::sim::scenario
