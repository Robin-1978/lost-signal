#include "game/content_catalog.hpp"
#include "game/map_serialize.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

// Editor save-path contract: load→serialize is byte-identical, so an editor
// save keeps the Python round-trip gate green and diffs stay reviewable.
// Any new map field must be added to the serializer in the same commit.
int main() {
	// The editor writes source-language content. Default-locale overlays are
	// for presentation and must not leak into authoring round trips.
	const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS, "zh-CN");
 const auto& map = catalog.map("research_base");
 const std::string saved = game::serialize_map_canonical(map);
 const auto path = std::filesystem::path(LOST_SIGNAL_TEST_ASSETS) / "maps" / "research_base.json";
 std::ifstream file(path);
 std::ostringstream on_disk;
 on_disk << file.rdbuf();
	if (saved != on_disk.str()) {
		const auto expected = on_disk.str();
		std::size_t offset = 0;
		while (offset < saved.size() && offset < expected.size() && saved[offset] == expected[offset]) {
			++offset;
		}
		std::cerr << "map serialization first differs at byte " << offset << "; serialized size "
				  << saved.size() << ", file size " << expected.size() << '\n';
		const auto context_start = offset > 80 ? offset - 80 : 0;
		std::cerr << "serialized context:\n"
				 << saved.substr(context_start, 160) << "\nfile context:\n"
				 << expected.substr(context_start, 160) << '\n';
	}
 CHECK(saved == on_disk.str());
 return 0;
}
