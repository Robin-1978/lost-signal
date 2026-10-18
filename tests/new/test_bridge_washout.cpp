#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/mission/construction_plan.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& level = catalog.level("bridge_the_washout");
 CHECK(level.construction.has_value());
 const auto& construction = *level.construction;
 const auto build = catalog.presentation_for_level(level.id).recommended_build;

 mission::ScenarioSession run(catalog, level.id, build, std::nullopt, {},
 mission::ComponentRegistry::builtins(), {}, mission::PhysicsMode::Bullet);
 CHECK(!run.construction_ready());
 CHECK(run.map_revision() == 1);
 CHECK(run.planned_route().empty());
 bool start_rejected = false;
 try {
 run.start();
 } catch (const std::logic_error&) {
 start_rejected = true;
 }
 CHECK(start_rejected);
 CHECK(run.snapshot().status == mission::MissionStatus::Ready);

 std::string reason;
 const auto long_plan = mission::construction_plan_from_option(
 construction, "washout.long_deck");
 CHECK(!run.commit_construction_plan(long_plan, &reason));
 CHECK(!reason.empty());
 CHECK(run.map_revision() == 1);
 CHECK(!run.committed_construction());
 CHECK(run.planned_route().empty());

 const auto short_plan = mission::construction_plan_from_option(
 construction, construction.default_plan);
 CHECK(run.commit_construction_plan(short_plan, &reason));
 CHECK(reason.empty());
 CHECK(run.construction_ready());
 CHECK(run.map_revision() == 2);
 CHECK(run.committed_construction()->plan_sha256 ==
 mission::construction_plan_sha256(short_plan));
 CHECK(!run.planned_route().empty());

 run.run_to_completion();
 CHECK(run.result().status == mission::MissionStatus::Succeeded);
 CHECK(run.result().collisions == 0);
 CHECK(run.result().construction_contact_observed);
 CHECK(run.result().construction_surface_traversed);
 CHECK(run.snapshot().robot.x > 16.0);

 bool physically_supported_on_bridge = false;
 bool lidar_saw_bridge = false;
 for (const auto& frame : run.replay()) {
 if (frame.robot.x > 9.0 && frame.robot.x < 11.0 &&
 frame.robot.y > 4.35 && frame.robot.y < 5.65 &&
 frame.robot_height_m > 0.9 && frame.rigid_contact_count > 0 &&
 !frame.ground_contact_links.empty()) {
 physically_supported_on_bridge = true;
 }
 if (!frame.lidar_point_cloud)
 continue;
 lidar_saw_bridge = lidar_saw_bridge || std::any_of(
 frame.lidar_point_cloud->begin(), frame.lidar_point_cloud->end(),
 [](const auto& point) {
 return point.position_m.x >= 8.5 && point.position_m.x <= 11.5 &&
 point.position_m.y >= 4.35 && point.position_m.y <= 5.65 &&
 point.position_m.z >= 0.76 && point.position_m.z <= 0.84 &&
 point.intensity > 0.2;
 });
 }
 CHECK(physically_supported_on_bridge);
 CHECK(lidar_saw_bridge);

 mission::ScenarioSession grip_run(catalog, level.id, build, std::nullopt, {},
 mission::ComponentRegistry::builtins(), {}, mission::PhysicsMode::Bullet);
 const auto grip_plan = mission::construction_plan_from_option(
 construction, "washout.grip_deck");
 CHECK(grip_run.commit_construction_plan(grip_plan, &reason));
 CHECK(grip_run.committed_construction()->plan_sha256 !=
 run.committed_construction()->plan_sha256);
 grip_run.run_to_completion();
 CHECK(grip_run.result().status == mission::MissionStatus::Succeeded);
 CHECK(grip_run.result().collisions == 0);
 CHECK(grip_run.result().construction_contact_observed);
 CHECK(grip_run.result().construction_surface_traversed);
 CHECK(grip_run.result().max_drive_slip_ratio < run.result().max_drive_slip_ratio);
 return 0;
}
