#include <algorithm>
#include <cmath>
#include <array>
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
#include "sim/providers/replay.hpp"

namespace {

using namespace lost_signal;

struct Options {
 std::string robot_path;
 std::string sim_path;
 std::string map_path;
 std::string out_path;
 std::string episode_path;
 std::string actors_path;
 std::vector<std::string> objectives;
 long steps = -1;
 double duration_s = -1.0;
 bool no_actors = false;
 bool verbose = false;
 bool profile = false;
 bool help = false;
};

struct SemanticsStats {
 std::size_t conflict_zones = 0;
 std::size_t non_box_collisions = 0;
 std::size_t dormant_actors = 0;
 std::size_t conflict_zone_steps = 0;
 std::size_t conflict_zone_entries = 0;
 std::vector<std::array<double, 4>> conflict_boxes;
};

bool load_semantics_sidecar(const std::string& map_path,
 SemanticsStats& out) {
 if (map_path.empty()) return false;
 std::string path = map_path;
 const std::size_t suffix = path.rfind(".json");
 if (suffix != std::string::npos) path.replace(suffix, 5, ".semantics.json");
 else path += ".semantics.json";
 std::ifstream in(path);
 if (!in.is_open()) return false;
 std::string text((std::istreambuf_iterator<char>(in)), {});
 std::string error;
 const auto json = robot::core::json_parse(text, &error);
 if (json == nullptr || !json->is_object()) return false;
 auto array_size = [&](const char* key) {
 const auto* value = json->find(key);
 return value != nullptr && value->is_array() ? value->as_array().size() : 0U;
 };
 out.non_box_collisions = array_size("non_box_collisions");
 out.dormant_actors = array_size("dormant_actors");
 const auto* zones = json->find("conflict_zones");
 if (zones == nullptr || !zones->is_array()) return true;
 for (const auto& zone : zones->as_array()) {
 const auto* min = zone.find("min");
 const auto* max = zone.find("max");
 if (min == nullptr || max == nullptr || !min->is_array() ||
 !max->is_array() || min->as_array().size() < 2 ||
 max->as_array().size() < 2 || !min->as_array()[0].is_num() ||
 !min->as_array()[1].is_num() || !max->as_array()[0].is_num() ||
 !max->as_array()[1].is_num()) continue;
 out.conflict_boxes.push_back({min->as_array()[0].as_num(),
 min->as_array()[1].as_num(), max->as_array()[0].as_num(),
 max->as_array()[1].as_num()});
 }
 out.conflict_zones = out.conflict_boxes.size();
 return true;
}

void score_semantics_step(const sim::eval::EvalStep& step,
 SemanticsStats& stats, bool& was_inside) {
 bool inside = false;
 for (const auto& box : stats.conflict_boxes) {
 if (step.truth_after.body_x_m >= std::min(box[0], box[2]) &&
 step.truth_after.body_x_m <= std::max(box[0], box[2]) &&
 step.truth_after.body_y_m >= std::min(box[1], box[3]) &&
 step.truth_after.body_y_m <= std::max(box[1], box[3])) {
 inside = true;
 break;
 }
 }
 if (inside) ++stats.conflict_zone_steps;
 if (inside && !was_inside) ++stats.conflict_zone_entries;
 was_inside = inside;
}

void usage() {
 std::fprintf(
 stderr,
 "usage: lost_signal_sim [--robot FILE] [--sim FILE] [--map FILE]\n"
 " [--objectives a,b,...] [--steps N | --duration S]\n"
 " [--actors FILE] [--no-actors] [--out FILE.json]\n"
 " [--episode FILE.jsonl] [--profile]\n");
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
 } else if (a == "--episode") {
 if (!value(opt.episode_path)) return false;
 } else if (a == "--actors") {
 if (!value(opt.actors_path)) return false;
 } else if (a == "--no-actors") {
 opt.no_actors = true;
 } else if (a == "--verbose") {
 opt.verbose = true;
 } else if (a == "--profile") {
 opt.profile = true;
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
 const sim::scenario::ScenarioSession& session,
 const SemanticsStats* semantics) {
 using robot::core::Json;
 const sim::eval::EpisodeScore score = session.score();
 Json::Object root;
 root.emplace_back("map", Json::Str(map_path));
 root.emplace_back("steps", Json::Num(static_cast<double>(score.steps)));
 root.emplace_back("elapsed_s", Json::Num(score.elapsed_s));
 root.emplace_back("objectives_done",
 Json::Bool(session.objectives_done()));
 root.emplace_back("objectives_passed",
 Json::Bool(session.objectives_passed()));
 root.emplace_back("objectives_total",
 Json::Num(static_cast<double>(session.objectives().total())));
 root.emplace_back("objectives_completed",
 Json::Num(static_cast<double>(session.objectives().completed())));
 root.emplace_back("objective_completion_ratio",
 Json::Num(session.objectives().completion_ratio()));
 root.emplace_back("mission_status",
 Json::Str(!session.objectives_done()
 ? "incomplete"
 : session.objectives_passed() ? "completed" : "failed"));
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
 s.emplace_back("path_length_m", Json::Num(score.path_length_m));
 s.emplace_back("dynamic_hold_steps",
 Json::Num(static_cast<double>(score.dynamic_hold_steps)));
 s.emplace_back("terrain_collision_steps",
 Json::Num(static_cast<double>(score.terrain_collision_steps)));
 s.emplace_back("mean_pos_err_m", Json::Num(score.mean_pos_err_m));
 s.emplace_back("max_pos_err_m", Json::Num(score.max_pos_err_m));
 s.emplace_back("mean_yaw_err_rad", Json::Num(score.mean_yaw_err_rad));
 s.emplace_back("max_yaw_err_rad", Json::Num(score.max_yaw_err_rad));
 s.emplace_back("min_clearance_m", Json::Num(score.min_clearance_m));
 s.emplace_back("mean_clearance_m", Json::Num(score.mean_clearance_m));
 s.emplace_back("min_terrain_clearance_m",
 Json::Num(score.min_terrain_clearance_m));
 s.emplace_back("mean_terrain_clearance_m",
 Json::Num(score.mean_terrain_clearance_m));
 s.emplace_back("energy_proxy", Json::Num(score.energy_proxy));
 root.emplace_back("score", Json::Obj(std::move(s)));
 Json::Array results;
 for (const auto& r : session.objectives().results()) {
 Json::Object o;
 o.emplace_back("place_id", Json::Str(r.place_id));
 o.emplace_back("robot_done", Json::Bool(r.robot_done));
 o.emplace_back("truth_arrived", Json::Bool(r.truth_arrived));
 o.emplace_back("successful", Json::Bool(r.successful()));
 o.emplace_back("truth_miss_m", Json::Num(r.truth_miss_m));
 o.emplace_back("abandoned", Json::Bool(r.abandoned));
 results.push_back(Json::Obj(std::move(o)));
 }
 root.emplace_back("objectives", Json::Arr(std::move(results)));
 if (semantics != nullptr) {
 Json::Object semantic;
 semantic.emplace_back("conflict_zones",
 Json::Num(static_cast<double>(semantics->conflict_zones)));
 semantic.emplace_back("conflict_zone_steps",
 Json::Num(static_cast<double>(semantics->conflict_zone_steps)));
 semantic.emplace_back("conflict_zone_entries",
 Json::Num(static_cast<double>(semantics->conflict_zone_entries)));
 semantic.emplace_back("non_box_collisions",
 Json::Num(static_cast<double>(semantics->non_box_collisions)));
 semantic.emplace_back("dormant_actors",
 Json::Num(static_cast<double>(semantics->dormant_actors)));
 root.emplace_back("sim_semantics", Json::Obj(std::move(semantic)));
 }
 const std::string text = robot::core::json_dump(Json::Obj(std::move(root)));
 std::ofstream out(path, std::ios::binary | std::ios::trunc);
 if (!out.is_open()) return false;
 out << text << '\n';
 out.flush();
 return out.good();
}

struct SlowTimingSample {
 std::size_t step = 0;
 sim::scenario::ScenarioSession::StepTiming timing;
 std::string localization = "unrecorded";
 std::string reason = "unrecorded";
 std::string track = "unrecorded";
 std::string slam = "unrecorded";
 double position_sigma_m = -1.0;
};

void retain_slowest(
 std::vector<SlowTimingSample>& slowest, std::size_t step,
 const sim::scenario::ScenarioSession::StepTiming& timing,
 const sim::eval::EvalStep* recorded) {
 SlowTimingSample sample;
 sample.step = step;
 sample.timing = timing;
 if (recorded != nullptr) {
 sample.localization = recorded->diag.localization_action;
 sample.reason = recorded->diag.evidence.reason_code;
 sample.track = recorded->diag.track_diagnostic;
 sample.slam = recorded->diag.slam_loop_diagnostic;
 sample.position_sigma_m = recorded->diag.estimate.position_sigma_m;
 }
 slowest.push_back(std::move(sample));
 std::stable_sort(slowest.begin(), slowest.end(),
 [](const SlowTimingSample& a, const SlowTimingSample& b) {
 return a.timing.robot_tick_ms > b.timing.robot_tick_ms;
 });
 constexpr std::size_t kSlowestCount = 5;
 if (slowest.size() > kSlowestCount) slowest.resize(kSlowestCount);
}

void print_timing_profile(
 const std::vector<sim::scenario::ScenarioSession::StepTiming>& samples,
 const std::vector<SlowTimingSample>& slowest) {
 using Timing = sim::scenario::ScenarioSession::StepTiming;
 if (samples.empty()) return;
 struct Stage {
 const char* name;
 double Timing::*member;
 };
 const Stage stages[] = {
 {"physics", &Timing::physics_ms},
 {"sensors", &Timing::sensors_ms},
 {"robot_tick", &Timing::robot_tick_ms},
 {"actuation", &Timing::actuation_ms},
 {"record", &Timing::record_ms},
 {"total", &Timing::total_ms},
 };
 std::fprintf(stderr, "timing_ms stage p50 p95 p99 max\n");
 for (const Stage& stage : stages) {
 std::vector<double> values;
 values.reserve(samples.size());
 for (const Timing& sample : samples) values.push_back(sample.*stage.member);
 std::sort(values.begin(), values.end());
 const auto percentile = [&](double fraction) {
 const std::size_t rank = static_cast<std::size_t>(
 std::ceil(fraction * static_cast<double>(values.size())));
 return values[std::min(values.size() - 1,
 rank > 0 ? rank - 1 : std::size_t{0})];
 };
 std::fprintf(stderr, "timing_ms %-10s %.3f %.3f %.3f %.3f\n",
 stage.name, percentile(0.50), percentile(0.95), percentile(0.99),
 values.back());
 }
 for (std::size_t rank = 0; rank < slowest.size(); ++rank) {
 const auto& sample = slowest[rank];
 std::fprintf(stderr,
 "timing_slowest rank=%zu step=%zu robot_tick_ms=%.3f total_ms=%.3f "
 "localization=%s sigma=%.3f track=%s slam=%s reason=%s\n",
 rank + 1, sample.step, sample.timing.robot_tick_ms,
 sample.timing.total_ms, sample.localization.c_str(),
 sample.position_sigma_m, sample.track.c_str(), sample.slam.c_str(),
 sample.reason.c_str());
 }
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
 // A headless score/profile run is a streaming reduction. Preserve the
 // append-only episode only when the caller explicitly requests a replay
 // artifact; the viewer and direct ScenarioSession users retain it by
 // default. The latest step remains available for live diagnostics.
 session->set_episode_history_enabled(!opt.episode_path.empty());

 if (!opt.map_path.empty()) {
 sim::eval::WorldFileContent content;
 std::string err;
 if (!sim::eval::load_world_file(opt.map_path, content, &err)) {
 std::fprintf(stderr, "bad --map file: %s\n", err.c_str());
 return 1;
 }
 sim::eval::apply_world_content(session->setup_world(), content);
 if (content.initial_pose.has_value()) {
 const auto& prior = *content.initial_pose;
 session->set_initial_pose(
 prior.x_m, prior.y_m, prior.yaw_rad, prior.position_sigma_m,
 prior.heading_sigma_rad);
 }
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
 std::vector<sim::scenario::ScenarioSession::StepTiming> timing_samples;
 if (opt.profile) timing_samples.reserve(static_cast<std::size_t>(budget));
 std::vector<SlowTimingSample> slowest_timing;
 SemanticsStats semantics;
 const bool have_semantics = load_semantics_sidecar(opt.map_path, semantics);
 bool was_inside_semantic_zone = false;
 std::size_t semantic_steps = 0;
 std::size_t verbose_loop_count = 0;
 std::string verbose_localization_event;
 for (; ran < budget && !session->objectives_done(); ++ran) {
 session->step_once();
 const sim::eval::EvalStep* recorded = session->last_eval_step();
 if (recorded != nullptr && recorded->step >= semantic_steps) {
 if (have_semantics) {
 score_semantics_step(
 *recorded, semantics, was_inside_semantic_zone);
 }
 semantic_steps = recorded->step + 1;
 }
 if (opt.profile) {
 const auto timing = session->last_step_timing();
 timing_samples.push_back(timing);
 retain_slowest(
 slowest_timing, static_cast<std::size_t>(ran), timing, recorded);
 }
 if (opt.verbose) {
 if (recorded != nullptr) {
 const auto& diag = recorded->diag;
 if (diag.slam_loops_closed > verbose_loop_count) {
 std::fprintf(stderr,
 "step=%ld loop-closed from=%llu to=%llu correction=%.3fm/%.3frad "
 "score=%.3f/%.3f overlap=%.3f margin=%.3f/%.3f "
 "est=(%.3f,%.3f,%.3f) evidence=%s\n",
 ran,
 static_cast<unsigned long long>(diag.slam_loop_from_id),
 static_cast<unsigned long long>(diag.slam_loop_to_id),
 diag.slam_loop_correction_m, diag.slam_loop_correction_rad,
 diag.slam_loop_forward_score, diag.slam_loop_reverse_score,
 diag.slam_loop_overlap, diag.slam_loop_forward_margin,
 diag.slam_loop_reverse_margin,
 diag.estimate.x_m, diag.estimate.y_m, diag.estimate.yaw_rad,
 diag.slam_loop_diagnostic.c_str());
 verbose_loop_count = diag.slam_loops_closed;
 }
 }
 }
 const bool collision_now = recorded != nullptr &&
 (recorded->judge.static_collision ||
 recorded->judge.dynamic_collision ||
 recorded->judge.terrain_collision);
 const std::string localization_action = recorded == nullptr
 ? std::string{}
 : recorded->diag.localization_action;
 const bool localization_event =
 localization_action.rfind("global_relocalization:", 0) == 0 &&
 localization_action != "global_relocalization:search pending" &&
 localization_action != verbose_localization_event;
 if (opt.verbose && (ran % 100 == 0 || collision_now || localization_event)) {
 const auto ev = session->last_evidence();
 const auto& truth = recorded == nullptr
 ? sim::world::WorldTruth{}
 : recorded->truth_after;
 const auto diag = recorded == nullptr
 ? robot::app::RobotApplication::Diagnostics{}
 : recorded->diag;
 std::fprintf(stderr,
 "step=%ld objective=%s goal=%s reason=%s action=%s v=%.2f w=%.2f "
 "truth=(%.2f,%.2f) est=(%.2f,%.2f) clr=%.2f safety=%s loc=%s "
 "collision=%s/%s/%s dyn_clr=%.2f localized=%s "
 "loops=%zu loop_corr=%.3f/%.3f tracks=%zu "
 "track_corr=%.3f/%.3f grip=%.2f slip=%s\n",
 ran, session->objectives().active_place().c_str(),
 session->current_goal().state ==
 robot::iface::GoalState::kActive ? "active" : "idle",
 ev.reason_code.c_str(), ev.action_code.c_str(),
 ev.target_speed_mps, ev.target_yaw_rate_radps, truth.body_x_m,
 truth.body_y_m, diag.estimate.x_m, diag.estimate.y_m,
 recorded == nullptr ? 999.0 : recorded->judge.static_clearance_m,
 diag.safety_reason.c_str(), diag.localization_action.c_str(),
 recorded == nullptr || !recorded->judge.static_collision ? "no" : "static",
 recorded == nullptr || !recorded->judge.dynamic_collision ? "no" : "dynamic",
 recorded == nullptr || !recorded->judge.terrain_collision ? "no" : "terrain",
 recorded == nullptr ? 999.0 : recorded->judge.dynamic_clearance_m,
 diag.localized ? "yes" : "no", diag.slam_loops_closed,
 diag.slam_loop_correction_m, diag.slam_loop_correction_rad,
 diag.slam_tracks, diag.track_correction_m,
 diag.track_correction_rad, diag.grip_factor, diag.slip_quality.c_str());
 if (localization_event) verbose_localization_event = localization_action;
 }
 }
 const sim::eval::EpisodeScore score = session->score();
 std::printf(
 "sim ran %ld steps; done=%s passed=%s collisions=%zu distance=%.2f m "
 "max_pos_err=%.3f m\n",
 ran, session->objectives_done() ? "yes" : "no",
 session->objectives_passed() ? "yes" : "no", score.collision_steps,
 score.distance_m, score.max_pos_err_m);
 if (opt.profile) print_timing_profile(timing_samples, slowest_timing);
 if (!opt.out_path.empty() &&
 !write_summary(opt.out_path, opt.map_path, *session,
 have_semantics ? &semantics : nullptr)) {
 std::fprintf(stderr, "cannot write --out file: %s\n",
 opt.out_path.c_str());
 return 1;
 }
 if (!opt.episode_path.empty()) {
 std::string episode_err;
 const auto map = session->active_map();
 if (!sim::providers::export_episode(opt.episode_path, map.map_id,
 map.revision, session->episode(), &episode_err)) {
 std::fprintf(stderr, "cannot write --episode file: %s\n",
 episode_err.c_str());
 return 1;
 }
 }
 return 0;
}
