#pragma once

#include <string>

namespace game {

struct MapDefinition;

// Canonical map serialization for the content editor (plan §4 item 7).
// Emits the exact file layout the loader reads (key order, 2-space indent,
// raw UTF-8, trailing newline) so load→save is byte-identical: the Python
// round-trip gate stays green after an editor save. Type tags the loader
// ignores ("box", "patrol") are re-emitted as their single schema-allowed
// value; if the schema ever gains variants the loader must read them first.
std::string serialize_map_canonical(const MapDefinition& map);

} // namespace game
