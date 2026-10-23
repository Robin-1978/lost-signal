#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "sim/eval/robot_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/scenario_session.hpp"

int main(int argc, char** argv) {
  using namespace lost_signal;
  // Optional single-source configuration: --robot <config> builds the
  // session from one RobotConfig file (ROBOT description + SIM motor
  // numbers), --map <world file> replaces the demo geometry.
  std::string robot_path, map_path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--robot" && i + 1 < argc) robot_path = argv[++i];
    if (a == "--map" && i + 1 < argc) map_path = argv[++i];
  }
  std::unique_ptr<sim::scenario::ScenarioSession> session;
  if (!robot_path.empty()) {
    sim::eval::RobotConfig config;
    std::string err;
    if (!sim::eval::load_robot_config(robot_path, config, &err)) {
      std::fprintf(stderr, "bad --robot file: %s\n", err.c_str());
      return 1;
    }
    session = std::make_unique<sim::scenario::ScenarioSession>(
        sim::eval::description_from_config(config),
        sim::eval::motor_from_config(config));
  } else {
    session = std::make_unique<sim::scenario::ScenarioSession>();
  }
  if (!map_path.empty()) {
    std::vector<sim::world::StaticBox> boxes;
    std::vector<sim::world::NamedPlace> places;
    std::string err;
    if (!sim::eval::load_world_file(map_path, boxes, places, &err)) {
      std::fprintf(stderr, "bad --map file: %s\n", err.c_str());
      return 1;
    }
    session->setup_world().set_static_boxes(std::move(boxes));
    session->setup_world().set_places(std::move(places));
    robot::iface::TaskRequest req;
    req.request_id = "demo";
    req.place_id =
        session->setup_world().places().empty()
            ? "dock"
            : session->setup_world().places().front().place_id;
    session->request_goal(req);
  } else {
    robot::iface::TaskRequest req;
    req.request_id = "demo";
    req.place_id = "dock";
    session->request_goal(req);
  }
  for (int i = 0; i < 5; ++i) session->step_once();
  std::printf("sim ran 5 steps\n");
  return 0;
}
