#pragma once

#include <string>

namespace game {

struct MapDefinition;

// Canonical map serialization for the content editor (plan §4 item 7).
// Emits the exact file layout the loader reads (key order, 2-space indent,
// raw UTF-8, trailing newline) so load→save is byte-identical: the Python
// round-trip gate stays green after an editor save. Every loader-read field
// round-trips (actor kind/collision/visual/dormant included); nothing the
// schema requires is ever dropped or defaulted on save.
std::string serialize_map_canonical(const MapDefinition& map);

} // namespace game
