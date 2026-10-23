#pragma once

#include <string>

#include "sim/scenario/scenario_session.hpp"

namespace lost_signal::sim::scenario {

// Load actors from <map>.actors.json (or an explicit path) into the
// session's world. The sidecar is SIM-side setup data authored next to the
// world; it never crosses into ROBOT. Returns the number loaded, or -1 on a
// malformed explicit file (a missing default sidecar means zero actors).
int load_actors_sidecar(const std::string& map_path,
 ScenarioSession& session,
 const std::string& explicit_path = {},
 std::string* error = nullptr);

} // namespace lost_signal::sim::scenario
