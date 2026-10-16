#pragma once

#include "content.hpp"

#include <stdexcept>

namespace mission {

class ContentError : public std::runtime_error {
 public:
 using std::runtime_error::runtime_error;
};

class ContentCatalog {
 public:
 // An empty locale selects assets/locales.json::default_locale. Locale files
 // are sparse JSON-pointer string overlays over the shared simulation data, so a
 // translation cannot silently fork physics or scoring parameters.
 static ContentCatalog load(const std::filesystem::path& assets_root,
 const std::string& locale = {});

 const std::filesystem::path& root() const { return root_; }
 const std::string& locale() const { return locale_; }
 const std::vector<std::string>& supported_locales() const { return supported_locales_; }
 const CampaignDefinition& campaign() const { return campaign_; }
 const LevelDefinition& level(const std::string& id) const;
 const MapDefinition& map(const std::string& id) const;
 const SceneDefinition& scene(const std::string& id) const;
 const RobotDefinition& robot(const std::string& id) const;
 const ComponentDefinition& component(const std::string& id) const;
 const PolicyDefinition& policy(const std::string& id) const;
 const PresentationDefinition& presentation(const std::string& id) const;
 const PresentationDefinition& presentation_for_level(const std::string& level_id) const;
 const PresentationDefinition& default_presentation() const;
 const StoryBeatDefinition* story_beat(const std::string& level_id, const std::string& when) const;

 const std::unordered_map<std::string, MapDefinition>& maps() const { return maps_; }
 const std::unordered_map<std::string, LevelDefinition>& levels() const { return levels_; }
 const std::unordered_map<std::string, ComponentDefinition>& components() const { return components_; }
 const std::unordered_map<std::string, AbilityDefinition>& abilities() const { return abilities_; }
 const AbilityDefinition& ability(const std::string& id) const;
 const std::vector<ContractJob>& contracts() const { return contracts_; }
 // UI strings (assets/strings.json): all player-facing chrome lives in content,
 // never in C++ literals. tr() throws on missing keys (typos fail loudly);
 // trf() fills {0}..{n} slots with caller-formatted args.
 const std::string& tr(const std::string& key) const;
 std::string trf(const std::string& key, const std::vector<std::string>& args) const;
 const SurfaceMaterialDefinition& surface_material(const std::string& id) const;
 const std::unordered_map<std::string, SurfaceMaterialDefinition>& surface_materials() const {
  return surface_materials_;
 }
 const std::unordered_map<std::string, PolicyDefinition>& policies() const { return policies_; }
 const std::unordered_map<std::string, RobotDefinition>& robots() const { return robots_; }
 const std::unordered_map<std::string, SceneDefinition>& scenes() const { return scenes_; }
 const std::unordered_map<std::string, PresentationDefinition>& presentations() const {
  return presentations_;
 }
 const std::vector<StoryBeatDefinition>& story_beats() const { return story_beats_; }
 const std::vector<ChapterDefinition>& chapters() const { return chapters_; }

 int build_cost(const BuildSelection& build) const;
 void validate_build(const std::string& level_id, const BuildSelection& build,
                      const std::vector<std::string>& owned_abilities = {}) const;
 // Continuous driving style s in [0, 1], piecewise-linear across the policy
 // anchors sorted by style. Anchor positions reproduce the presets bit-for-bit.
 Properties style_behavior(double style) const;

 private:
 std::filesystem::path root_;
 std::string locale_;
 std::vector<std::string> supported_locales_;
 CampaignDefinition campaign_;
 std::unordered_map<std::string, LevelDefinition> levels_;
 std::unordered_map<std::string, MapDefinition> maps_;
 std::unordered_map<std::string, SurfaceMaterialDefinition> surface_materials_;
 std::unordered_map<std::string, SceneDefinition> scenes_;
 std::unordered_map<std::string, RobotDefinition> robots_;
 std::unordered_map<std::string, ComponentDefinition> components_;
 std::unordered_map<std::string, AbilityDefinition> abilities_;
 std::vector<ContractJob> contracts_;
 std::unordered_map<std::string, std::string> strings_;
 std::unordered_map<std::string, PolicyDefinition> policies_;
 std::unordered_map<std::string, PresentationDefinition> presentations_;
 std::vector<StoryBeatDefinition> story_beats_;
 std::vector<ChapterDefinition> chapters_;
 std::string default_presentation_id_;
};

} // namespace mission
