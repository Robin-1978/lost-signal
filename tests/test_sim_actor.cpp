#include <cassert>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

#include "robot/iface/task.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/robot/sim_robot.hpp"

int main() {
  using namespace lost_signal;

  // Actor crosses the robot's path: drive -> hold while near -> resume.
  sim::robot::SimRobot sim;
  assert(sim.configure().ok && sim.activate().ok);
  sim::world::Actor crosser;
  crosser.x = 2.0;
  crosser.y = -1.5;
  crosser.vx_mps = 0.0;
  crosser.vy_mps = 1.0;
  crosser.radius_m = 0.4;
  // Actors are sim-owned test setup: configured before the run.
  sim.setup_world().add_actor(crosser);
  sim.setup_world().add_place({"dock", 0.0, 0.0, 0.0});

  robot::iface::TaskRequest req;
  req.request_id = "crossing";
  req.place_id = "dock";
  assert(sim.request_goal(req).valid());
  sim.set_goal({6.0, 0.0, 0.0});

  std::vector<double> xs;
  for (int i = 0; i < 280; ++i) {
 sim.tick(static_cast<robot::core::MonotonicNs>(i) * 20'000'000LL);
    xs.push_back(sim.world().truth().body_x_m);
  }
  // Rigid-coupling plant accelerates at ΣF/M (4.5 m/s^2 full duty), so
  // Engine line is lazier off the line; still strictly toward the goal,
  // holds for the actor, resumes after it passes.
  assert(xs[8] > 0.005);              // drove toward the goal initially
  assert(xs[60] < 0.5);               // held while the actor was near
  assert(xs[279] > 2.0);              // resumed after the actor passed
  assert(sim.current_goal().state == robot::iface::GoalState::kActive);

 // Headless sidecar: the same loader used by lost_signal_sim.
 {
 std::ofstream out("/tmp/test_actors_sidecar.json");
 out << R"({"actors":[{"id":"walker","visual_kind":"pedestrian",
 "x":1.5,"y":2.0,"radius_m":0.3,
 "behavior":"wander","speed_mps":0.7,"seed":4}]})";
 out.close();
 sim::scenario::ScenarioSession session;
 std::string err;
 const int n = sim::scenario::load_actors_sidecar(
 "/tmp/test_map.json", session, "/tmp/test_actors_sidecar.json", &err);
 assert(n == 1);
 assert(session.setup_world().actors().size() == 1);
 assert(session.setup_world().actors()[0].radius_m == 0.3);
 assert(session.setup_world().actors()[0].behavior.speed_mps == 0.7);
 assert(session.setup_world().actors()[0].id == "walker");
 assert(session.setup_world().actors()[0].visual_kind == "pedestrian");
 std::remove("/tmp/test_actors_sidecar.json");
 }

 // Invalid authored motion is rejected transactionally. Expanding the wall
 // by the actor radius catches a path that misses the raw rectangle but
 // would still collide physically.
 {
 std::ofstream out("/tmp/test_invalid_actors_sidecar.json");
 out << R"({"actors":[{"id":"wall-crosser","x":0.0,"y":0.0,
 "radius_m":0.3,"behavior":"waypoints","speed_mps":0.7,
 "waypoints":[{"x":2.0,"y":0.0}]}]})";
 out.close();
 sim::scenario::ScenarioSession session;
 session.setup_world().set_static_boxes({{0.9, 0.29, 1.1, 1.0, 1.0}});
 std::string err;
 assert(sim::scenario::load_actors_sidecar(
 "/tmp/test_map.json", session,
 "/tmp/test_invalid_actors_sidecar.json", &err) == -1);
 assert(err.find("wall-crosser") != std::string::npos);
 assert(err.find("static box 0") != std::string::npos);
 assert(session.setup_world().actors().empty());
 std::remove("/tmp/test_invalid_actors_sidecar.json");
 }

 // Every checked-in sidecar must be physically valid against its matching
 // world. This turns path-through-wall mistakes into an authoring error at
 // review time instead of a surprise actor appearing through a wall.
 {
 const std::filesystem::path worlds =
 std::filesystem::path(TEST_CONFIG_DIR) / "worlds";
 std::size_t checked = 0;
 for (const auto& entry : std::filesystem::directory_iterator(worlds)) {
 const std::string filename = entry.path().filename().string();
 constexpr const char* kSuffix = ".actors.json";
 if (!filename.ends_with(kSuffix)) continue;
 const std::string map_name =
 filename.substr(0, filename.size() - std::char_traits<char>::length(
 kSuffix)) + ".json";
 const std::filesystem::path map_path = worlds / map_name;
 assert(std::filesystem::is_regular_file(map_path));
 sim::eval::WorldFileContent content;
 std::string err;
 assert(sim::eval::load_world_file(map_path.string(), content, &err));
 sim::scenario::ScenarioSession session;
 sim::eval::apply_world_content(session.setup_world(), content);
 assert(sim::scenario::load_actors_sidecar(
 map_path.string(), session, entry.path().string(), &err) >= 0);
 ++checked;
 }
 assert(checked >= 5);
 }

  std::printf("sim_actor ok: x8=%.2f x60=%.2f x_end=%.2f\n", xs[8], xs[60],
              xs[279]);
  return 0;
}
