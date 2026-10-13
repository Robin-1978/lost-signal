#include "content_catalog.hpp"
#include "mission_simulation.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <iostream>
#include <string>

namespace {

void usage(const char* executable) {
 std::cout << "Usage: " << executable << " [--assets PATH] [--level ID] [--default|--recommended] [--json]\n";
}

} // namespace

int main(int argc, char** argv) {
 try {
  std::filesystem::path assets = "assets";
  std::string level_id;
  bool recommended = true;
  bool json_output = false;
  for (int index = 1; index < argc; ++index) {
   const std::string argument = argv[index];
   if (argument == "--assets" && index + 1 < argc) {
    assets = argv[++index];
   } else if (argument == "--level" && index + 1 < argc) {
    level_id = argv[++index];
   } else if (argument == "--default") {
    recommended = false;
   } else if (argument == "--recommended") {
    recommended = true;
   } else if (argument == "--json") {
    json_output = true;
   } else if (argument == "--help" || argument == "-h") {
    usage(argv[0]);
    return 0;
   } else {
    throw std::runtime_error("unknown argument: " + argument);
   }
  }

  const auto catalog = game::ContentCatalog::load(assets);
  if (level_id.empty()) {
   level_id = catalog.campaign().starting_level;
  }
  const auto& presentation = catalog.presentation_for_level(level_id);
  const auto build = recommended ? presentation.recommended_build : presentation.default_build;
  game::MissionSimulation mission(catalog, level_id, build);
  mission.run_to_completion();
  const auto& result = mission.result();

  if (json_output) {
   nlohmann::json output = {
     {"level", level_id},
     {"status", game::mission_status_name(result.status)},
     {"failure_reason", result.failure_reason},
     {"time_s", result.time_s},
     {"path_length_m", result.path_length_m},
     {"energy_wh", result.energy_wh},
     {"min_clearance_m", result.min_clearance_m},
     {"min_static_clearance_m", result.min_static_clearance_m},
     {"min_static_clearance_time_s", result.min_static_clearance_time_s},
     {"min_static_robot_position",
      {
        result.min_static_robot_position.x,
        result.min_static_robot_position.y,
      }},
     {"min_dynamic_clearance_m", result.min_dynamic_clearance_m},
     {"min_dynamic_clearance_time_s", result.min_dynamic_clearance_time_s},
     {"min_dynamic_robot_position",
      {
        result.min_dynamic_robot_position.x,
        result.min_dynamic_robot_position.y,
      }},
     {"min_dynamic_actor_position",
      {
        result.min_dynamic_actor_position.x,
        result.min_dynamic_actor_position.y,
      }},
     {"odometry_error_m", result.odometry_error_m},
     {"collisions", result.collisions},
     {"astar_expanded", result.astar_expanded},
     {"local_decisions", result.local_decisions},
     {"robot",
      {
        {"x", mission.snapshot().robot.x},
        {"y", mission.snapshot().robot.y},
        {"heading", mission.snapshot().robot.theta},
      }},
     {"via_reached", result.via_reached},
     {"events", nlohmann::json::array()},
     {"scores",
      {
        {"safe", result.safe_score},
        {"fast", result.fast_score},
        {"efficient", result.efficient_score},
      }},
   };
   for (const auto& event : mission.events()) {
    output["events"].push_back({
      {"time_s", event.time_s},
      {"code", event.code},
      {"message", event.message},
    });
   }
   std::cout << output.dump(2) << '\n';
  } else {
   std::cout << "[" << level_id << "] " << game::mission_status_name(result.status)
             << "  time=" << result.time_s << "s"
             << "  path=" << result.path_length_m << "m"
             << "  energy=" << result.energy_wh << "Wh"
             << "  clearance=" << result.min_clearance_m << "m";
   if (!result.failure_reason.empty()) {
    std::cout << "  reason=" << result.failure_reason;
   }
   std::cout << '\n';
  }
  return result.status == game::MissionStatus::Succeeded ? 0 : 2;
 } catch (const std::exception& error) {
  std::cerr << "error: " << error.what() << '\n';
  return 1;
 }
}
