#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/scenario_session.hpp"

int main(int argc, char** argv) {
  using namespace lost_signal;
  // Optional single-source configuration: --robot <config> builds the
  // session from one RobotConfig file (ROBOT description + SIM motor
  // numbers), --map <world file> replaces the demo geometry.
  std::string robot_path, sim_path, map_path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--robot" && i + 1 < argc) robot_path = argv[++i];
    if (a == "--sim" && i + 1 < argc) sim_path = argv[++i];
    if (a == "--map" && i + 1 < argc) map_path = argv[++i];
  }
  std::string used_config, config_err;
  auto robot_config = robot::platform::load_default_robot_config(
      robot_path, &used_config, &config_err);
  if (!robot_path.empty() && !robot_config.has_value()) {
    std::fprintf(stderr, "bad --robot file: %s\n", config_err.c_str());
    return 1;
  }
  std::unique_ptr<sim::scenario::ScenarioSession> session;
  if (robot_config.has_value()) {
    std::fprintf(stderr, "robot config: %s\n", used_config.c_str());
    session = std::make_unique<sim::scenario::ScenarioSession>(
        robot::platform::description_from_config(*robot_config),
        sim::eval::motor_from_config(*robot_config));
  } else {
    session = std::make_unique<sim::scenario::ScenarioSession>();
  }
  std::string used_sim, sim_err;
  auto sim_config = sim::eval::load_default_sim_config(sim_path, &used_sim,
                                                       &sim_err);
  if (!sim_path.empty() && !sim_config.has_value()) {
    std::fprintf(stderr, "bad --sim file: %s\n", sim_err.c_str());
    return 1;
  }
  if (sim_config.has_value()) {
    std::fprintf(stderr, "sim config: %s\n", used_sim.c_str());
    session->apply_sim_config(*sim_config);
  }
  if (!map_path.empty()) {
    sim::eval::WorldFileContent content;
    std::string err;
    if (!sim::eval::load_world_file(map_path, content, &err)) {
      std::fprintf(stderr, "bad --map file: %s\n", err.c_str());
      return 1;
    }
    sim::eval::apply_world_content(session->setup_world(), content);
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
