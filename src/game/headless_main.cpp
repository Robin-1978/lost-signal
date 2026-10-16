#include "content_catalog.hpp"
#include "mission_simulation.hpp"
#include "physics/physics_world.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <optional>
#include <iostream>
#include <string>

namespace {

void usage(const char* executable) {
 std::cout << "Usage: " << executable
 << " [--assets PATH] [--level ID] [--default|--recommended] [--seed N]"
 " [--physics domain|shadow|bullet|auto] [--endless [--duration SECONDS]"
 " [--intent follow|go-to|search|wait|return] [--target X Y]] [--json]\n";
}

game::FieldIntentKind parse_field_intent(const std::string& value) {
 if (value == "follow") return game::FieldIntentKind::Follow;
 if (value == "go-to" || value == "go_to") return game::FieldIntentKind::GoTo;
 if (value == "search") return game::FieldIntentKind::Search;
 if (value == "wait") return game::FieldIntentKind::Wait;
 if (value == "return") return game::FieldIntentKind::Return;
 throw std::runtime_error("unknown field intent: " + value);
}

} // namespace

int main(int argc, char** argv) {
 try {
  std::filesystem::path assets = "assets";
  std::string level_id;
  bool recommended = true;
  bool json_output = false;
 bool endless = false;
 double duration_s = 60.0;
  std::optional<uint64_t> seed_override;
 std::optional<game::FieldIntentKind> field_intent;
 std::optional<sim::Vec2> field_target;
 game::PhysicsMode physics_mode = game::PhysicsMode::Bullet;
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
   } else if (argument == "--seed" && index + 1 < argc) {
    seed_override = std::stoull(argv[++index]);
   } else if (argument == "--json") {
    json_output = true;
 } else if (argument == "--endless") {
 endless = true;
 } else if (argument == "--duration" && index + 1 < argc) {
 duration_s = std::stod(argv[++index]);
 if (!(duration_s > 0.0))
 throw std::runtime_error("endless duration must be positive");
 } else if (argument == "--intent" && index + 1 < argc) {
 field_intent = parse_field_intent(argv[++index]);
 } else if (argument == "--target" && index + 2 < argc) {
 field_target = sim::Vec2{std::stod(argv[++index]), std::stod(argv[++index])};
 } else if (argument == "--physics" && index + 1 < argc) {
 physics_mode = game::parse_physics_mode(argv[++index]);
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
 if (field_intent && !endless)
 throw std::runtime_error("--intent requires --endless");
 game::MissionSimulation mission(catalog, level_id, build, seed_override, {},
 game::ComponentRegistry::builtins(), {}, physics_mode);
 if (endless) {
 mission.set_endless_mode(true);
 if (field_intent && !mission.issue_field_intent(*field_intent, field_target))
 throw std::runtime_error("field intent requires a valid target");
 mission.start();
 const int steps = static_cast<int>(std::ceil(duration_s / game::MissionSimulation::fixed_step_s));
 for (int step = 0; step < steps; ++step)
 mission.step();
 } else {
 mission.run_to_completion();
 }
  const auto& result = mission.result();
 const auto runtime_health = mission.runtime_health();

  if (json_output) {
   nlohmann::json output = {
     {"level", level_id},
     {"seed", mission.effective_seed()},
 {"physics_backend", mission.physics_backend_name()},
 {"runtime",
 {
 {"interface", "lsri"},
 {"api_version", mission.runtime_api_version()},
 {"provider", mission.runtime_provider_id()},
 {"state", sim::runtime::lifecycle_state_name(mission.runtime_state())},
 {"health", sim::runtime::health_level_name(runtime_health.level)},
 {"dropped_observations", runtime_health.dropped_observations},
 {"rejected_observations", runtime_health.rejected_observations},
 {"rejected_commands", runtime_health.rejected_commands},
 {"watchdog_stops", runtime_health.watchdog_stops},
 }},
     {"bullet_available", sim::physics::bullet_physics_available()},
     {"status", game::mission_status_name(result.status)},
     {"failure_reason", result.failure_reason},
     {"time_s", result.time_s},
     {"path_length_m", result.path_length_m},
     {"energy_wh", result.energy_wh},
     {"min_clearance_m", result.min_clearance_m},
     {"scored_clearance_m", result.scored_clearance_m},
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
     {"max_odometry_error_m", result.max_odometry_error_m},
     {"collisions", result.collisions},
 {"rigid_contacts", result.rigid_contacts},
 {"shadow_error",
 {
 {"max_position_m", result.max_shadow_position_error_m},
 {"max_heading_rad", result.max_shadow_heading_error_rad},
 {"max_height_m", result.max_shadow_height_error_m},
 }},
     {"slip_events", result.slip_events},
     {"astar_expanded", result.astar_expanded},
     {"local_decisions", result.local_decisions},
     {"robot",
      {
        {"x", mission.snapshot().robot.x},
        {"y", mission.snapshot().robot.y},
        {"z", mission.snapshot().robot_height_m},
        {"heading", mission.snapshot().robot.theta},
        {"roll_rad", mission.snapshot().robot_roll_rad},
        {"pitch_rad", mission.snapshot().robot_pitch_rad},
      }},
     {"terrain",
      {
        {"current_grade", mission.snapshot().terrain_grade},
        {"current_cross_slope", mission.snapshot().cross_slope},
        {"max_grade", result.max_grade},
        {"max_abs_roll_rad", result.max_abs_roll_rad},
        {"current_rollover_risk", mission.snapshot().rollover_risk},
        {"max_rollover_risk", result.max_rollover_risk},
        {"traction_saturated", mission.snapshot().traction_saturated},
        {"traction_saturation_occurred", result.slip_events > 0},
      }},
 {"endless",
 {
 {"enabled", mission.endless_mode()},
 {"autonomous", mission.endless_mode()},
 {"dispatch_active", mission.field_dispatch().active},
 {"dispatch_completed", mission.field_dispatch().completed_count},
 {"route_points", mission.planned_path().size()},
 {"field_operator", {
 {"x", mission.field_operator().position.x},
 {"y", mission.field_operator().position.y},
 {"heading_rad", mission.field_operator().heading_rad},
 {"moving", mission.field_operator().moving},
 }},
 {"field_intent", {
 {"active", mission.field_intent().active},
 {"kind", game::field_intent_kind_id(mission.field_intent().kind)},
 {"revision", mission.field_intent().revision},
 {"acknowledgement",
 game::field_intent_acknowledgement_id(mission.field_intent().acknowledgement)},
 {"target", {mission.field_intent().target.x, mission.field_intent().target.y}},
 }},
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
             << " clearance=" << result.min_clearance_m << "m"
 << " physics=" << mission.physics_backend_name() << " runtime=" << mission.runtime_provider_id()
             << " z=" << mission.snapshot().robot_height_m << "m"
             << " roll=" << mission.snapshot().robot_roll_rad << "rad"
             << " pitch=" << mission.snapshot().robot_pitch_rad << "rad"
             << " grade=" << mission.snapshot().terrain_grade << " max_grade=" << result.max_grade
             << " max_roll=" << result.max_abs_roll_rad << "rad"
             << " rollover_risk=" << result.max_rollover_risk
             << " traction_saturated=" << (mission.snapshot().traction_saturated ? "yes" : "no");
   if (!result.failure_reason.empty()) {
    std::cout << "  reason=" << result.failure_reason;
   }
   std::cout << '\n';
  }
 return endless || result.status == game::MissionStatus::Succeeded ? 0 : 2;
 } catch (const std::exception& error) {
  std::cerr << "error: " << error.what() << '\n';
  return 1;
 }
}
