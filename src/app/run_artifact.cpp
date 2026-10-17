#include "run_artifact.hpp"

#include "lost_signal/build_info.hpp"

#include "core/sha256.hpp"
#include "mission/content_catalog.hpp"
#include "scenario/scenario_session.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace app {
namespace {

struct ContentIdentity {
 std::string digest;
 std::size_t file_count{0};
};

ContentIdentity content_identity(const std::filesystem::path& assets) {
 std::vector<std::filesystem::path> files;
 for (const auto& entry : std::filesystem::recursive_directory_iterator(assets)) {
 if (entry.is_regular_file() && entry.path().extension() == ".json")
 files.push_back(entry.path());
 }
 std::sort(files.begin(), files.end(), [&](const auto& left, const auto& right) {
 return left.lexically_relative(assets).generic_string() < right.lexically_relative(assets).generic_string();
 });

 sim::Sha256 hash;
 std::array<unsigned char, 8192> buffer{};
 for (const auto& path : files) {
 const std::string relative = path.lexically_relative(assets).generic_string();
 hash.update(relative);
 hash.update(std::string_view("\0", 1));
 std::ifstream input(path, std::ios::binary);
 if (!input)
 throw std::runtime_error("cannot read content file for run artifact: " + path.string());
 while (input) {
 input.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
 if (input.gcount() > 0)
 hash.update(buffer.data(), static_cast<std::size_t>(input.gcount()));
 }
 hash.update(std::string_view("\0", 1));
 }
 return {hash.hex_digest(), files.size()};
}

} // namespace

std::string run_content_digest(const std::filesystem::path& assets) {
 return content_identity(assets).digest;
}

nlohmann::json make_run_artifact(
 const mission::ContentCatalog& catalog,
 const mission::ScenarioSession& simulation,
 const std::filesystem::path& assets,
 const nlohmann::json& result,
 std::string_view selection_label,
 double requested_duration_s) {
 const auto content = content_identity(assets);
 const nlohmann::json hardware = {
 {"robot", simulation.robot().id},
 {"drive_type", simulation.robot().drive_type},
 {"components", simulation.build().parts},
 {"style", simulation.build().style},
 {"requested_chassis", simulation.build().chassis_robot},
 };
 const std::string task_ordering = simulation.level().objective.goal_order == "autonomous"
 ? simulation.level().objective.task_policy
 : std::string{};
 const nlohmann::json algorithms = {
 {"global_planner", simulation.global_planner_id()},
 {"local_planner", simulation.local_planner_id()},
 {"high_level_policy", simulation.strategy_policy_name()},
 {"task_ordering", task_ordering},
 {"localization", simulation.level().localization_mode},
 {"motion_controller", "pid-feedforward-v1"},
 {"runtime_interface", "lsri-v1"},
 };
 const nlohmann::json experiment = {
 {"mission_id", simulation.level().id},
 {"condition_id", "authored-base"},
 {"mode", simulation.endless_mode() ? "endless" : "mission"},
 {"seed", simulation.effective_seed()},
 {"physics_backend", simulation.physics_backend_name()},
 {"runtime_provider", simulation.runtime_provider_id()},
 {"fixed_step_s", mission::ScenarioSession::fixed_step_s},
 {"requested_duration_s", simulation.endless_mode() ? nlohmann::json(requested_duration_s) : nlohmann::json(nullptr)},
 };
 const nlohmann::json comparable_configuration = {
 {"content_sha256", content.digest},
 {"experiment", experiment},
 {"hardware", hardware},
 {"algorithms", algorithms},
 };

 nlohmann::json metrics = result;
 metrics.erase("events");
 metrics.erase("decisions");
 return {
 {"schema", {{"name", "lost-signal.run"}, {"version", kRunArtifactSchemaVersion}}},
 {"producer", {
 {"project_version", build_info::project_version},
 {"source_revision", build_info::source_revision},
 {"source_dirty", build_info::source_dirty},
 {"build", {
 {"compiler", build_info::compiler_id},
 {"compiler_version", build_info::compiler_version},
 {"type", build_info::build_type},
 {"system", build_info::system_name},
 {"processor", build_info::system_processor},
 }},
 }},
 {"content", {
 {"algorithm", "sha256"},
 {"digest", content.digest},
 {"file_count", content.file_count},
 {"campaign_id", catalog.campaign().id},
 {"campaign_version", catalog.campaign().content_version},
 {"manifest", "sorted-json-path-nul-bytes-nul-v1"},
 }},
 {"run", {
 {"selection_label", selection_label},
 {"experiment", experiment},
 {"hardware", hardware},
 {"algorithms", algorithms},
 }},
 {"configuration", {
 {"algorithm", "sha256"},
 {"digest", sim::sha256(comparable_configuration.dump())},
 {"comparable", comparable_configuration},
 }},
 {"inputs", {{"trace_refs", nlohmann::json::array()}}},
 {"result", metrics},
 {"events", result.at("events")},
 {"decisions", result.at("decisions")},
 };
}

} // namespace app
