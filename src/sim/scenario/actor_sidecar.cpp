#include "sim/scenario/actor_sidecar.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

#include "robot/core/json.hpp"

namespace lost_signal::sim::scenario {

int load_actors_sidecar(const std::string& map_path,
 ScenarioSession& session,
 const std::string& explicit_path, std::string* error) {
 std::string side = explicit_path;
 if (side.empty()) {
 const std::string::size_type dot = map_path.rfind('.');
 side = (dot == std::string::npos ? map_path : map_path.substr(0, dot)) +
 ".actors.json";
 } else {
 // Explicit path: missing is an error, unlike the optional sibling.
 std::ifstream probe(side, std::ios::binary);
 if (!probe.is_open()) {
 if (error) *error = "cannot open actor sidecar: " + side;
 return -1;
 }
 }
 std::ifstream in(side, std::ios::binary);
 if (!in.is_open()) return 0;
 std::ostringstream ss;
 ss << in.rdbuf();
 std::string parse_error;
 auto root = robot::core::json_parse(ss.str(), &parse_error);
 if (root == nullptr || !root->is_object()) {
 if (error) *error = "bad actor sidecar " + side + ": " + parse_error;
 return -1;
 }
 const auto* arr = root->find("actors");
 if (arr == nullptr || !arr->is_array()) {
 if (error) *error = "actor sidecar has no actors array: " + side;
 return -1;
 }
 int loaded = 0;
 for (const auto& e : arr->as_array()) {
 if (!e.is_object()) continue;
 const auto* wx = e.find("x");
 const auto* wy = e.find("y");
 if (wx == nullptr || wy == nullptr || !wx->is_num() || !wy->is_num()) {
 continue;
 }
 world::Actor a;
 a.x = wx->as_num();
 a.y = wy->as_num();
 const auto* r = e.find("radius_m");
 if (r != nullptr && r->is_num()) a.radius_m = r->as_num();
 const auto* h = e.find("height_m");
 if (h != nullptr && h->is_num()) a.height_m = h->as_num();
 const auto* beh = e.find("behavior");
 const std::string kind =
 (beh != nullptr && beh->is_str()) ? beh->as_str() : "waypoints";
 if (!world::ActorBehaviorRegistry::global().create(
 kind == "wander" ? "wander" : "waypoints", a.behavior)) {
 continue;
 }
 const auto* sp = e.find("speed_mps");
 if (sp != nullptr && sp->is_num()) a.behavior.speed_mps = sp->as_num();
 const auto* lp = e.find("loop");
 if (lp != nullptr && lp->is_bool()) a.behavior.loop = lp->as_bool();
 const auto* sd = e.find("seed");
 if (sd != nullptr && sd->is_num()) {
 a.behavior.seed = static_cast<std::uint64_t>(sd->as_num());
 }
 const auto* wps = e.find("waypoints");
 if (wps != nullptr && wps->is_array()) {
 for (const auto& w : wps->as_array()) {
 const auto* qx = w.find("x");
 const auto* qy = w.find("y");
 if (qx == nullptr || qy == nullptr || !qx->is_num() ||
 !qy->is_num()) {
 continue;
 }
 a.behavior.waypoints.push_back({qx->as_num(), qy->as_num()});
 }
 }
 const auto* wt = e.find("waits");
 if (wt != nullptr && wt->is_array()) {
 for (const auto& v : wt->as_array()) {
 if (v.is_num()) a.behavior.waits_s.push_back(v.as_num());
 }
 }
 session.setup_world().add_actor(a);
 ++loaded;
 }
 std::fprintf(stderr, "actors sidecar: %s (%d actors)\n", side.c_str(),
 loaded);
 return loaded;
}

} // namespace lost_signal::sim::scenario
