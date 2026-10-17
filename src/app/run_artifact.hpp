#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include <nlohmann/json_fwd.hpp>

namespace mission {
class ContentCatalog;
class MissionSimulation;
}

namespace app {

inline constexpr int kRunArtifactSchemaVersion = 2;

std::string run_content_digest(const std::filesystem::path& assets);

nlohmann::json make_run_artifact(
 const mission::ContentCatalog& catalog,
 const mission::MissionSimulation& simulation,
 const std::filesystem::path& assets,
 const nlohmann::json& result,
 std::string_view selection_label,
 double requested_duration_s);

} // namespace app
