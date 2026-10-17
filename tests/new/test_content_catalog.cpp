#include "mission/content_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 CHECK(catalog.locale() == "en");
 CHECK(catalog.campaign().title == "Lost Signal");
 CHECK(catalog.campaign().companion_name == "RO-1");
 CHECK(catalog.tr("app.window_title") == "Lost Signal");
 CHECK(catalog.supported_locales().size() == 2);

 const auto source_catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS, "zh-CN");
 CHECK(source_catalog.locale() == "zh-CN");
 CHECK(source_catalog.campaign().title == "\u5931\u843d\u7684\u4fe1\u53f7");
 CHECK(source_catalog.campaign().companion_name == "\u6d1b\u4e00");
 CHECK(source_catalog.tr("app.window_title") ==
 "\u5931\u843d\u7684\u4fe1\u53f7 / Lost Signal");

 // A locale overlay may change player-facing strings, but never the content
 // graph, IDs, paths, physics, scoring, seeds, or algorithm configuration.
 CHECK(source_catalog.levels().size() == catalog.levels().size());
 CHECK(source_catalog.maps().size() == catalog.maps().size());
 CHECK(source_catalog.scenes().size() == catalog.scenes().size());
 CHECK(source_catalog.components().size() == catalog.components().size());
 CHECK(source_catalog.robots().size() == catalog.robots().size());
 CHECK(source_catalog.campaign().levels == catalog.campaign().levels);
 CHECK(source_catalog.campaign().playgrounds == catalog.campaign().playgrounds);
 for (const auto& [id, localized] : catalog.levels()) {
  const auto& source = source_catalog.level(id);
  CHECK(localized.id == source.id);
  CHECK(localized.map == source.map);
  CHECK(localized.control_mode == source.control_mode);
  CHECK(localized.localization_mode == source.localization_mode);
  CHECK(localized.seed == source.seed);
  CHECK(localized.budget == source.budget);
  CHECK(localized.objective.time_limit_s == source.objective.time_limit_s);
  CHECK(localized.objective.max_collisions == source.objective.max_collisions);
  CHECK(localized.safe_min_clearance_m == source.safe_min_clearance_m);
  CHECK(localized.fast_time_s == source.fast_time_s);
  CHECK(localized.efficient_energy_wh == source.efficient_energy_wh);
 }

 bool rejected_unknown_locale = false;
 try {
  (void)mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS, "not-a-locale");
 } catch (const mission::ContentError&) {
  rejected_unknown_locale = true;
 }
 CHECK(rejected_unknown_locale);
 CHECK(!catalog.campaign().levels.empty());
 CHECK(catalog.levels().size() ==
 catalog.campaign().levels.size() + catalog.campaign().playgrounds.size());
 for (const auto& id : catalog.campaign().levels)
  CHECK(catalog.levels().count(id) == 1);
 CHECK(catalog.components().size() == 19);
 CHECK(catalog.policies().size() == 4);
 CHECK(catalog.robots().size() == 5);
 for (const auto& [id, map] : catalog.maps())
  CHECK(catalog.scenes().count(map.scene_3d) == 1);
 for (const auto& [id, level] : catalog.levels()) {
 // Use the catalog-loader invariant: every mission has exactly one
 // presentation (the finale uses the default; IDs are never guessed).
  int serving = 0;
  for (const auto& [pid, presentation] : catalog.presentations()) {
   (void)pid;
   if (presentation.level == id)
    ++serving;
  }
  CHECK(serving == 1);
  CHECK(catalog.maps().count(level.map) == 1);
 }
 for (const auto& beat : catalog.story_beats())
  CHECK(catalog.levels().count(beat.level) == 1);
 CHECK(catalog.campaign().companion_robot == "robot.ro1");

 const auto& robot = catalog.robot(catalog.campaign().companion_robot);
 CHECK(robot.drive_type == "differential");
 CHECK(std::abs(robot.motor.max_wheel_speed_mps - 2.4) < 1e-9);
 CHECK(robot.encoder.counts_per_revolution == 2048);
 CHECK(std::abs(robot.wheel_base_m - 0.58) < 1e-12);
 CHECK(robot.articulation.root_link == "base_link");
 CHECK(robot.articulation.links.size() == 5);
 CHECK(robot.articulation.joints.size() == 4);
 CHECK(robot.articulation.actuators.size() == 2);
 double articulated_mass_kg = 0.0;
 for (const auto& link : robot.articulation.links) {
 articulated_mass_kg += link.mass_kg;
 CHECK(link.inertia_diagonal_kgm2.x <=
 link.inertia_diagonal_kgm2.y + link.inertia_diagonal_kgm2.z + 1e-12);
 CHECK(link.inertia_diagonal_kgm2.y <=
 link.inertia_diagonal_kgm2.x + link.inertia_diagonal_kgm2.z + 1e-12);
 CHECK(link.inertia_diagonal_kgm2.z <=
 link.inertia_diagonal_kgm2.x + link.inertia_diagonal_kgm2.y + 1e-12);
 }
 CHECK(std::abs(articulated_mass_kg - robot.mass_kg) < 1e-12);
 CHECK(std::abs(robot.base_link_height_m - 0.24) < 1e-12);
 CHECK(std::abs(robot.center_of_mass_height_m - 0.2183125) < 1e-9);
 CHECK(std::abs(robot.yaw_inertia_kgm2 - 1.333745) < 1e-9);
 CHECK(std::abs(robot.articulation.actuators.front().armature_inertia_kgm2 - 0.023) < 1e-12);
 const auto caster_wheel_link = std::find_if(robot.articulation.links.begin(),
 robot.articulation.links.end(),
 [](const auto& link) { return link.id == "rear_caster_wheel_link"; });
 const auto caster_link = std::find_if(robot.articulation.links.begin(),
 robot.articulation.links.end(),
 [](const auto& link) { return link.id == "rear_caster_link"; });
 CHECK(caster_wheel_link != robot.articulation.links.end());
 CHECK(caster_link != robot.articulation.links.end());
 CHECK(std::abs((caster_link->position_m.x - caster_wheel_link->position_m.x) - 0.035) < 1e-12);
 // Second vehicle: ackermann geometry with honest steering hardware. No
 // level assigns it yet (slice 2); the catalog contract already holds.
 const auto& second = catalog.robot("robot.ro2");
 CHECK(second.drive_type == "ackermann");
 CHECK(second.steering.max_steer_rad > 0.0);
 CHECK(second.steering.max_steer_rate_radps > 0.0);
 CHECK(second.steering.servo_response_s > 0.0);
 CHECK(second.motor.wheel_equiv_mass_kg > 0.0);
 // Third vehicle: tracked twin-drive on the shared contact with a vendor
 // spin spec. No level assigns it yet; the catalog contract already holds.
 const auto& third = catalog.robot("robot.ro3");
 CHECK(third.drive_type == "tracked");
 CHECK(catalog.robot("robot.ro4").drive_type == "swerve");
 CHECK(catalog.robot("robot.ro5").drive_type == "mecanum");
 CHECK(third.tracked_max_yaw_rate_rads > 0.0);
 CHECK(third.track_length_m > 0.0);
 // Ground material library: the migrated wet_floor resolves to library
 // numbers, so this also pins the zero-behavior-change migration.
 CHECK(catalog.surface_material("standing_water").friction_mu == 0.35);
 CHECK(catalog.surface_material("standing_water").traction_scale == 0.45);
 CHECK(catalog.surface_material("standing_water").drive_effect == "spray");
 CHECK(catalog.surface_material("sand").drive_effect == "dust");
 CHECK(catalog.surface_material("paved").traction_scale == 1.0);
 CHECK(catalog.surface_material("paved").lidar_reflectivity == 0.35);
 CHECK(catalog.surface_material("steel_deck").lidar_reflectivity == 0.75);
 CHECK(robot.mounts.size() == 9);
 CHECK(robot.mounts.back().frame_id == "imu_link");
 CHECK(robot.passive_supports.size() == 1);
 CHECK(robot.passive_supports.front().steer_mount == "caster_rear");
 CHECK(robot.passive_supports.front().roll_mount == "caster_rear_wheel");
 CHECK(robot.base_parts.size() == 3);
 const auto cargo_mount = std::find_if(robot.mounts.begin(), robot.mounts.end(),
 [](const auto& mount) { return mount.id == "cargo_bed"; });
 CHECK(cargo_mount != robot.mounts.end());
 CHECK(cargo_mount->motion == "fixed");
 const auto driven_wheel = std::find_if(robot.mounts.begin(), robot.mounts.end(),
 [](const auto& mount) { return mount.id == "wheel_left"; });
 CHECK(driven_wheel != robot.mounts.end());
 CHECK(driven_wheel->motion == "wheel");
 CHECK(driven_wheel->rolling_radius_m == robot.wheel_radius_m);
 CHECK(std::abs(driven_wheel->position_m.x - 0.12) < 1e-12);
 const auto caster = std::find_if(robot.mounts.begin(), robot.mounts.end(),
 [](const auto& mount) { return mount.id == "caster_rear"; });
 CHECK(caster != robot.mounts.end());
 CHECK(std::abs(caster->position_m.x + 0.28) < 1e-12);
 CHECK(robot.slots.size() == 5);
 CHECK(catalog.component("sensor.short_fast_lidar").visual.primitive_2d == "disc");
 CHECK(catalog.component("sensor.short_fast_lidar").visual.model_3d.ends_with("lidar_fast.glb"));
 CHECK(catalog.story_beat("boot", "before") != nullptr);
 CHECK(catalog.story_beat("lost_signal", "after")->speaker == "Unknown Signal");

 const auto& level = catalog.level("lost_signal");
 CHECK(level.objective.via == "supply_station");
 CHECK(level.objective.goal == "signal_tower");
 CHECK(std::abs(level.objective.time_limit_s - 180.0) < 1e-9);
 CHECK(level.objective.goals.size() == 3);
 CHECK(std::none_of(level.objective.goals.begin(), level.objective.goals.end(),
 [](const auto& goal) { return goal.action == "inspect"; }));

 const auto& map = catalog.map(level.map);
 CHECK(map.markers.count("robot_bay") == 1);
 CHECK(map.markers.count("east_gate") == 1);
 CHECK(map.obstacles.size() == 3);
 CHECK(map.actors.size() == 3);
 CHECK(map.actors[1].id == "forklift_b");
 CHECK(map.actors[2].id == "forklift_finale");
 CHECK(map.actors[2].dormant_until_summoned);
 CHECK(map.scene_3d == "scene.research_base");
 const auto& scene = catalog.scene(map.scene_3d);
 CHECK(scene.prefabs.size() == 8);
 CHECK(scene.instances.size() == 31);
 CHECK(scene.floor.albedo_texture == "textures/environment/concrete_floor.png");
 CHECK(std::abs(scene.camera.fov_degrees - 41.0) < 1e-9);
 CHECK(std::abs(scene.camera.event_shot.fov_degrees - 38.0) < 1e-9);
 CHECK(std::abs(scene.camera.success_shot.goal_focus_weight - 0.52) < 1e-9);
 CHECK(std::abs(scene.effects.prediction_horizon_s - 2.4) < 1e-9);
 CHECK(scene.effects.surface_particle_count == 28);
 CHECK(std::abs(scene.effects.trail_duration_s - 1.9) < 1e-9);
 CHECK(scene.lighting.shadow_map_resolution == 2048);
 CHECK(std::abs(scene.lighting.bloom_strength - 0.2) < 1e-9);

 const auto& presentation = catalog.default_presentation();
 CHECK(presentation.level == "boot");
 CHECK(presentation.default_build.part("sensor").empty());
 CHECK(presentation.default_build.part("computer").empty());
 CHECK(std::abs(presentation.default_build.style - 0.5) < 1e-12);
 CHECK(catalog.policies().size() == 4);
 CHECK(std::abs(catalog.policy("policy.careful").style_anchor - 0.0) < 1e-12);
 CHECK(std::abs(catalog.policy("policy.balanced").style_anchor - 0.5) < 1e-12);
 CHECK(std::abs(catalog.policy("policy.swift").style_anchor - 0.8) < 1e-12);
 CHECK(std::abs(catalog.policy("policy.express").style_anchor - 1.0) < 1e-12);
 CHECK(std::abs(catalog.component("motor.speed").behavior.at("max_wheel_speed_mps") - 3.6) < 1e-12);
 CHECK(std::abs(catalog.component("motor.speed").behavior.at("stall_force_N") - 38.0) < 1e-12);
 const auto& final_presentation = catalog.presentation_for_level("lost_signal");
 CHECK(std::abs(final_presentation.recommended_build.style - 0.0) < 1e-12);
 CHECK(std::abs(final_presentation.default_build.style - 0.0) < 1e-12);
 CHECK(final_presentation.default_build.part("sensor") ==
 final_presentation.recommended_build.part("sensor"));
 CHECK(final_presentation.default_build.part("computer") == "computer.light");
 CHECK(final_presentation.recommended_build.part("computer") == "computer.performance");
 CHECK(final_presentation.goal_lines.size() == level.objective.goals.size() + 1);
 CHECK(!final_presentation.component_columns.front().player_choice);
 CHECK(final_presentation.component_columns[1].player_choice);
 CHECK(final_presentation.component_columns[2].player_choice);
 CHECK(!final_presentation.component_columns[3].player_choice);
 CHECK(!final_presentation.component_columns[4].player_choice);
 CHECK(catalog.build_cost(final_presentation.default_build) <= level.budget);
 CHECK(catalog.build_cost(final_presentation.recommended_build) <= level.budget);
 int mainline = 0;
 int contracts = 0;
 int experiments = 0;
 for (const auto& [id, entry] : catalog.levels()) {
  (void)id;
  mainline += entry.mode == "mainline" ? 1 : 0;
  contracts += entry.mode == "contract" ? 1 : 0;
  experiments += entry.mode == "experiment" ? 1 : 0;
 }
 CHECK(mainline == 8);
 CHECK(contracts == 2);
 CHECK(experiments == 16);
 return 0;
}
