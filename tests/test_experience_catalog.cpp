#include <cassert>
#include <filesystem>
#include <set>
#include <string>

#include "sim/experiment/experience_runtime.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/viewer/asset_manifest.hpp"
#include "sim/viewer/experience_manifest.hpp"
#include "sim/viewer/scene_manifest.hpp"

// Every published chapter is a real runtime, even when it is currently used
// as an engineering course rather than a scored campaign level.  This keeps
// the catalog from drifting into viewer-only labels that fail on first tick.
int main() {
 using namespace lost_signal;
 const std::filesystem::path root = TEST_SOURCE_DIR;
 const std::filesystem::path catalog = root / "configs/experiences";
 std::size_t checked = 0;
 std::set<std::string> ids;
 for (const auto& entry : std::filesystem::directory_iterator(catalog)) {
 if (!entry.is_regular_file() || entry.path().extension() != ".json") {
 continue;
 }
 sim::viewer::ExperienceManifest manifest;
 std::string error;
 assert(sim::viewer::load_experience_manifest(
 entry.path().string(), manifest, &error));
 assert(ids.insert(manifest.experience_id).second);
 assert(!manifest.objectives.empty());
 assert(std::filesystem::is_regular_file(manifest.map_path));
 assert(std::filesystem::is_regular_file(manifest.scene_path));
 // Keep the catalog's presentation binding honest: every published chapter
 // must load its own world and scene before it is exposed by the Viewer.
 sim::eval::WorldFileContent world;
 assert(sim::eval::load_world_file(manifest.map_path, world, &error));
 sim::viewer::AssetManifest assets;
 assert(sim::viewer::load_asset_manifest(
     (root / "assets/models/manifest.json").string(), assets, &error));
 sim::viewer::SceneManifest scene;
 assert(sim::viewer::load_scene_manifest(
     manifest.scene_path, assets, world.boxes, scene, &error));
 assert(scene.scene_id == manifest.experience_id ||
        scene.scene_id ==
            std::filesystem::path(manifest.map_path).stem().string());
 const auto config = sim::experiment::experience_runtime_config(
 manifest, (root / "assets/models/manifest.json").string());
 sim::experiment::ExperienceRuntime runtime;
 assert(sim::experiment::build_experience_runtime(config, runtime, &error));
 assert(runtime.session != nullptr);
 assert(runtime.session->episode().steps().empty());

 // The first transition exercises the actual ROBOT/SIM seam.  In particular,
 // mapless and terrain chapters must publish sensor measurements through the
 // same providers and must not rely on a pre-populated truth scan.
 runtime.session->step_once();
 const auto* step = runtime.session->last_eval_step();
 assert(step != nullptr);
 assert(step->step == 1);
 assert(!step->obs.scan_static_valid);
 assert(step->obs.static_ranges_m.empty());
 assert(runtime.session->episode().steps().front().presentation.captured);
 ++checked;
 }
 // There are fifteen published chapters. Keep this exact so a manifest can
 // never silently disappear from the campaign/catalog test.
 assert(checked == 15);
 assert(ids.size() == checked);
 return 0;
}
