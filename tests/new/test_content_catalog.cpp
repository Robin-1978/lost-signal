#include "game/content_catalog.hpp"

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
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 CHECK(catalog.campaign().companion_name == "洛一");
 CHECK(catalog.campaign().levels.size() == 5);
 CHECK(catalog.levels().size() == 5);
 CHECK(catalog.components().size() == 10);
 CHECK(catalog.policies().size() == 3);
 CHECK(catalog.robots().size() == 1);
 CHECK(catalog.scenes().size() == 1);
 CHECK(catalog.presentations().size() == 5);
 CHECK(catalog.story_beats().size() == 7);
 CHECK(catalog.campaign().companion_robot == "robot.ro1");

 const auto& robot = catalog.robot(catalog.campaign().companion_robot);
 CHECK(robot.drive_type == "differential");
 CHECK(std::abs(robot.motor.max_wheel_speed_mps - 2.4) < 1e-9);
 CHECK(robot.encoder.counts_per_revolution == 2048);
 CHECK(robot.mounts.size() == 5);
 CHECK(robot.base_parts.size() == 1);
 CHECK(robot.slots.size() == 3);
 CHECK(catalog.component("sensor.short_fast_lidar").visual.primitive_2d == "disc");
 CHECK(catalog.component("sensor.short_fast_lidar").visual.model_3d.ends_with("lidar_fast.glb"));
 CHECK(catalog.story_beat("boot", "before") != nullptr);
 CHECK(catalog.story_beat("lost_signal", "after")->speaker == "未知信号");

 const auto& level = catalog.level("lost_signal");
 CHECK(level.objective.via == "supply_station");
 CHECK(level.objective.goal == "signal_tower");
 CHECK(std::abs(level.objective.time_limit_s - 150.0) < 1e-9);

 const auto& map = catalog.map(level.map);
 CHECK(map.markers.count("robot_bay") == 1);
 CHECK(map.obstacles.size() == 3);
 CHECK(map.actors.size() == 1);
 CHECK(map.scene_3d == "scene.research_base");
 const auto& scene = catalog.scene(map.scene_3d);
 CHECK(scene.prefabs.size() == 8);
 CHECK(scene.instances.size() == 31);
 CHECK(scene.floor.albedo_texture == "textures/environment/concrete_floor.png");
 CHECK(std::abs(scene.camera.fov_degrees - 41.0) < 1e-9);
 CHECK(std::abs(scene.camera.event_shot.fov_degrees - 38.0) < 1e-9);
 CHECK(std::abs(scene.camera.success_shot.goal_focus_weight - 0.52) < 1e-9);
 CHECK(std::abs(scene.effects.prediction_horizon_s - 2.4) < 1e-9);
 CHECK(scene.lighting.shadow_map_resolution == 2048);
 CHECK(std::abs(scene.lighting.bloom_strength - 0.2) < 1e-9);

 const auto& presentation = catalog.default_presentation();
 CHECK(presentation.level == "boot");
 CHECK(presentation.default_build.sensor.empty());
 CHECK(presentation.default_build.computer.empty());
 CHECK(std::abs(presentation.default_build.style - 0.5) < 1e-12);
 CHECK(catalog.policies().size() == 3);
 CHECK(std::abs(catalog.policy("policy.careful").style_anchor - 0.0) < 1e-12);
 CHECK(std::abs(catalog.policy("policy.balanced").style_anchor - 0.5) < 1e-12);
 CHECK(std::abs(catalog.policy("policy.swift").style_anchor - 1.0) < 1e-12);
 const auto& final_presentation = catalog.presentation_for_level("lost_signal");
 CHECK(std::abs(final_presentation.recommended_build.style - 0.0) < 1e-12);
 CHECK(std::abs(final_presentation.default_build.style - 1.0) < 1e-12);
 CHECK(catalog.build_cost(final_presentation.default_build) <= level.budget);
 CHECK(catalog.build_cost(final_presentation.recommended_build) <= level.budget);
 return 0;
}
