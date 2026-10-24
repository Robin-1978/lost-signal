#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>

#include "sim/eval/recording.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/providers/replay.hpp"
#include "sim/scenario/scenario_session.hpp"

int main() {
 using namespace lost_signal;
 const std::string path = "/tmp/robot_simu_artifact.jsonl";
 sim::scenario::ScenarioSession session;
 session.setup_world().add_place({"dock", 3.0, 0.0, 0.0});
 session.set_objectives({"dock"});
 for (int i = 0; i < 25; ++i) session.step_once();
 assert(!session.episode().steps().empty());
 const auto score = sim::eval::score_episode(session.episode());
 assert(score.path_length_m >= score.distance_m);

 std::string error;
 assert(sim::providers::export_episode(path, "sim-world",
 session.setup_world().geometry_version(), session.episode(), &error));
 sim::providers::ReplayProvider replay;
 assert(replay.load(path, &error));
 assert(replay.size() == session.episode().steps().size());
 robot::iface::ObservationFrame obs;
 std::size_t count = 0;
 while (replay.poll(obs)) {
 assert(robot::iface::valid_observation_frame(obs));
 ++count;
 }
 assert(count == replay.size());

 std::ifstream in(path);
 std::string header;
 assert(std::getline(in, header));
 assert(header.find("\"kind\":\"episode\"") != std::string::npos);
 // Truth labels must never appear in the replay transport artifact.
 const std::string all((std::istreambuf_iterator<char>(in)), {});
 assert(all.find("truth_after") == std::string::npos);
 assert(all.find("terrain_collision") == std::string::npos);
 std::remove(path.c_str());
 std::puts("artifact ok");
 return 0;
}
