#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/experience_manifest.hpp"

int main() {
 using namespace lost_signal;
 sim::viewer::ExperienceManifest experience;
 std::string error;
 assert(sim::viewer::load_experience_manifest(
 std::string(TEST_SOURCE_DIR) + "/configs/experiences/first_contact.json",
 experience, &error));

 robot::platform::RobotConfig robot_config;
 assert(robot::platform::load_robot_config(
 experience.robot_path, robot_config, &error));
 sim::scenario::ScenarioSession session(
 robot::platform::description_from_config(robot_config),
 sim::eval::motor_from_config(robot_config));
 sim::eval::SimConfig sim_config;
 assert(sim::eval::load_sim_config(
 experience.sim_path, sim_config, &error));
 session.apply_sim_config(sim_config);

 sim::eval::WorldFileContent world;
 assert(sim::eval::load_world_file(experience.map_path, world, &error));
 session.setup_world().set_static_boxes(std::move(world.boxes));
 session.setup_world().set_places(std::move(world.places));
 session.setup_world().set_materials(std::move(world.materials));
 session.setup_world().set_material_zones(std::move(world.zones));
 session.setup_world().set_terrain_patches(std::move(world.patches));
 session.setup_world().set_support_slabs(std::move(world.slabs));
 session.setup_world().set_environment(std::move(world.environment));
 assert(world.initial_pose.has_value());
 const auto& prior = *world.initial_pose;
 session.set_initial_pose(prior.x_m, prior.y_m, prior.yaw_rad,
 prior.position_sigma_m, prior.heading_sigma_rad);
 session.set_objectives(experience.objectives);
 session.fit_map_window_to_world();
 assert(sim::scenario::load_actors_sidecar(
 experience.map_path, session, {}, &error) >= 0);

 for (int step = 0; step < 1200 && !session.objectives_done(); ++step) {
 session.step_once();
 }
 const auto score = session.score();
 assert(session.objectives_done());
 assert(session.objectives_passed());
 assert(score.collision_steps == 0);
 assert(score.max_pos_err_m < 0.35);
 assert(score.min_clearance_m >=
 robot_config.navigation.clearance_margin_m);
 assert(session.objectives().results().size() == 1);
 assert(session.objectives().results().front().truth_miss_m < 0.30);
 return 0;
}
