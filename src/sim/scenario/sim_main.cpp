#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "robot/core/json.hpp"
#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"

namespace {

using namespace lost_signal;

struct Options {
 std::string robot_path;
 std::string sim_path;
 std::string map_path;
 std::string out_path;
 std::string actors_path;
 std::vector<std::string> objectives;
 long steps = -1;
 double duration_s = -1.0;
 bool no_actors = false;
 bool help = false;
};

void usage() {
 std::fprintf(
 stderr,
 "usage: lost_signal_sim [--robot FILE] [--sim FILE] [--map FILE]\n"
 " [--objectives a,b,...] [--steps N | --duration S]\n"
 " [--actors FILE] [--no-actors] [--out FILE.json]\n");
}

std::vector<std::string> split_csv(const std::string& text) {
 std::vector<std::string> out;
 std::size_t begin = 0;
 while (begin <= text.size()) {
 const std::size_t comma = text.find(',', begin);
 const std::string item =
 text.substr(begin, comma == std::string::npos ? std::string::npos
 : comma - begin);
 if (!item.empty()) out.push_back(item);
 if (comma == std::string::npos) break;
 begin = comma + 1;
 }
 return out;
}

bool parse_args(int argc, char** argv, Options& opt) {
 for (int i = 1; i < argc; ++i) {
 const std::string a = argv[i];
 auto value = [&](std::string& dst) {
 if (i + 1 >= argc) return false;
 dst = argv[++i];
 return true;
 };
 if (a == "--robot") {
 if (!value(opt.robot_path)) return false;
 } else if (a == "--sim") {
 if (!value(opt.sim_path)) return false;
 } else if (a == "--map") {
 if (!value(opt.map_path)) return false;
 } else if (a == "--out") {
 if (!value(opt.out_path)) return false;
 } else if (a == "--actors") {
 if (!value(opt.actors_path)) return false;
 } else if (a == "--no-actors") {
 opt.no_actors = true;
 } else if (a == "--objectives") {
 std::string list;
 if (!value(list)) return false;
 opt.objectives = split_csv(list);
 } else if (a == "--steps") {
 std::string n;
 if (!value(n)) return false;
 char* end = nullptr;
 opt.steps = std::strtol(n.c_str(), &end, 10);
 if (end == n.c_str() || *end != '\0' || opt.steps < 0) return false;
 } else if (a == "--duration") {
 std::string n;
 if (!value(n)) return false;
 char* end = nullptr;
 opt.duration_s = std::strtod(n.c_str(), &end);
 if (end == n.c_str() || *end != '\0' || !(opt.duration_s >= 0.0)) {
 return false;
 }
 } else if (a == "--help" || a == "-h") {
 opt.help = true;
 } else {
 std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
 return false;
 }
 }
 return true;
}

bool write_summary(const std::string& path, const std::string& map_path,
 const sim::scenario::ScenarioSession& session) {
 using robot::core::Json;
 const sim::eval::EpisodeScore score = sim::eval::score_episode(session.episode());
 Json::Object root;
 root.emplace_back("map", Json::Str(map_path));
 root.emplace_back("steps", Json::Num(static_cast<double>(score.steps)));
 root.emplace_back("objectives_done",
 Json::Bool(session.objectives_done()));
 Json::Object s;
 s.emplace_back("safe_stop_steps",
 Json::Num(static_cast<double>(score.safe_stop_steps)));
 s.emplace_back("obs_accepted",
 Json::Num(static_cast<double>(score.obs_accepted)));
 s.emplace_back("obs_rejected",
 Json::Num(static_cast<double>(score.obs_rejected)));
 s.emplace_back("collision_steps",
 Json::Num(static_cast<double>(score.collision_steps)));
 s.emplace_back("distance_m", Json::Num(score.distance_m));
 s.emplace_back("mean_pos_err_m", Json::Num(score.mean_pos_err_m));
 s.emplace_back("max_pos_err_m", Json::Num(score.max_pos_err_m));
 s.emplace_back("mean_yaw_err_rad", Json::Num(score.mean_yaw_err_rad));
 s.emplace_back("max_yaw_err_rad", Json::Num(score.max_yaw_err_rad));
 root.emplace_back("score", Json::Obj(std::move(s)));
 Json::Array results;
 for (const auto& r : session.objectives().results()) {
 Json::Object o;
 o.emplace_back("place_id", Json::Str(r.place_id));
 o.emplace_back("robot_done", Json::Bool(r.robot_done));
 o.emplace_back("truth_arrived", Json::Bool(r.truth_arrived));
 o.emplace_back("truth_miss_m", Json::Num(r.truth_miss_m));
 o.emplace_back("abandoned", Json::Bool(r.abandoned));
 results.push_back(Json::Obj(std::move(o)));
 }
 root.emplace_back("objectives", Json::Arr(std::move(results)));
 const std::string text = robot::core::json_dump(Json::Obj(std::move(root)));
 std::ofstream out(path, std::ios::binary | std::ios::trunc);
 if (!out.is_open()) return false;
 out << text << '\n';
 out.flush();
 return out.good();
}

} // namespace

int main(int argc, char** argv) {
 using namespace lost_signal;
 Options opt;
 if (!parse_args(argc, argv, opt)) {
 usage();
 return 2;
 }
 if (opt.help) {
 usage();
 return 0;
 }
 if (opt.steps >= 0 && opt.duration_s >= 0.0) {
 std::fprintf(stderr, "--steps and --duration are mutually exclusive\n");
 return 2;
 }

 std::string used_config, config_err;
 auto robot_config = robot::platform::load_default_robot_config(
 opt.robot_path, &used_config, &config_err);
 if (!opt.robot_path.empty() && !robot_config.has_value()) {
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
 auto sim_config = sim::eval::load_default_sim_config(opt.sim_path, &used_sim,
 &sim_err);
 if (!opt.sim_path.empty() && !sim_config.has_value()) {
 std::fprintf(stderr, "bad --sim file: %s\n", sim_err.c_str());
 return 1;
 }
 if (sim_config.has_value()) {
 std::fprintf(stderr, "sim config: %s\n", used_sim.c_str());
 session->apply_sim_config(*sim_config);
 }

 if (!opt.map_path.empty()) {
 sim::eval::WorldFileContent content;
 std::string err;
 if (!sim::eval::load_world_file(opt.map_path, content, &err)) {
 std::fprintf(stderr, "bad --map file: %s\n", err.c_str());
 return 1;
 }
 sim::eval::apply_world_content(session->setup_world(), content);
 // Survey covers the authored map (far-field goals need a route).
 session->fit_map_window_to_world();
 if (!opt.no_actors) {
 std::string actor_err;
 const int actors = sim::scenario::load_actors_sidecar(
 opt.map_path, *session, opt.actors_path, &actor_err);
 if (actors < 0) {
 std::fprintf(stderr, "bad actors sidecar: %s\n", actor_err.c_str());
 return 1;
 }
 }
 }

 std::vector<std::string> objectives = opt.objectives;
 if (objectives.empty()) {
 const auto& places = session->setup_world().places();
 objectives.push_back(places.empty() ? "dock" : places.front().place_id);
 }
 session->set_objectives(objectives, 0.5);

 long budget = 0;
 if (opt.steps >= 0) {
 budget = opt.steps;
 } else if (opt.duration_s >= 0.0) {
 budget = static_cast<long>(std::ceil(opt.duration_s / 0.02));
 } else {
 budget = opt.objectives.empty() ? 5 : 4000;
 }
 long ran = 0;
 for (; ran < budget && !session->objectives_done(); ++ran) {
 session->step_once();
 }
 const sim::eval::EpisodeScore score =
 sim::eval::score_episode(session->episode());
 std::printf(
 "sim ran %ld steps; done=%s collisions=%zu distance=%.2f m "
 "max_pos_err=%.3f m\n",
 ran, session->objectives_done() ? "yes" : "no", score.collision_steps,
 score.distance_m, score.max_pos_err_m);
 if (!opt.out_path.empty() &&
 !write_summary(opt.out_path, opt.map_path, *session)) {
 std::fprintf(stderr, "cannot write --out file: %s\n",
 opt.out_path.c_str());
 return 1;
 }
 return 0;
}
