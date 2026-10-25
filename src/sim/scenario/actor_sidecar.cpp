#include "sim/scenario/actor_sidecar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <utility>

#include "robot/core/json.hpp"

namespace lost_signal::sim::scenario {

namespace {

bool segment_intersects_expanded_box(double x0, double y0,
 double x1, double y1, const world::StaticBox& box, double radius_m) {
 if (!(box.height_m > 0.0)) return false;
 const double minimum_x = std::min(box.x0, box.x1) - radius_m;
 const double maximum_x = std::max(box.x0, box.x1) + radius_m;
 const double minimum_y = std::min(box.y0, box.y1) - radius_m;
 const double maximum_y = std::max(box.y0, box.y1) + radius_m;
 const double dx = x1 - x0;
 const double dy = y1 - y0;
 double enter = 0.0;
 double leave = 1.0;
 const auto clip = [&](double origin, double delta,
 double minimum, double maximum) {
 if (std::abs(delta) < 1e-12) {
 return origin >= minimum && origin <= maximum;
 }
 double a = (minimum - origin) / delta;
 double b = (maximum - origin) / delta;
 if (a > b) std::swap(a, b);
 enter = std::max(enter, a);
 leave = std::min(leave, b);
 return enter <= leave;
 };
 return clip(x0, dx, minimum_x, maximum_x) &&
 clip(y0, dy, minimum_y, maximum_y);
}

bool finite(double value) { return std::isfinite(value); }

} // namespace

bool validate_actor_routes(
 const std::vector<world::Actor>& actors,
 const std::vector<world::StaticBox>& boxes, ActorRouteIssue* issue) {
 for (std::size_t actor_index = 0; actor_index < actors.size();
 ++actor_index) {
 const auto& actor = actors[actor_index];
 std::vector<world::ActorWaypoint> path{{actor.x, actor.y}};
 path.insert(path.end(), actor.behavior.waypoints.begin(),
 actor.behavior.waypoints.end());
 if (actor.behavior.loop && actor.behavior.waypoints.size() > 1) {
 path.push_back(actor.behavior.waypoints.front());
 }

 // Check points separately so stationary and wandering actors cannot start
 // inside geometry even when they have no authored segment.
 for (std::size_t point = 0; point < path.size(); ++point) {
 for (std::size_t box = 0; box < boxes.size(); ++box) {
 if (!segment_intersects_expanded_box(
 path[point].x_m, path[point].y_m,
 path[point].x_m, path[point].y_m, boxes[box],
 std::max(0.0, actor.radius_m))) {
 continue;
 }
 if (issue != nullptr) {
 *issue = {actor_index, point == 0 ? 0 : point - 1, box, actor.id};
 }
 return false;
 }
 }
 for (std::size_t point = 1; point < path.size(); ++point) {
 for (std::size_t box = 0; box < boxes.size(); ++box) {
 if (!segment_intersects_expanded_box(
 path[point - 1].x_m, path[point - 1].y_m,
 path[point].x_m, path[point].y_m, boxes[box],
 std::max(0.0, actor.radius_m))) {
 continue;
 }
 if (issue != nullptr) {
 *issue = {actor_index, point - 1, box, actor.id};
 }
 return false;
 }
 }
 }
 return true;
}

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

 std::vector<world::Actor> actors;
 actors.reserve(arr->as_array().size());
 std::set<std::string> ids;
 for (const auto& e : arr->as_array()) {
 if (!e.is_object()) {
 if (error) *error = "actor entry is not an object: " + side;
 return -1;
 }
 const auto* wx = e.find("x");
 const auto* wy = e.find("y");
 if (wx == nullptr || wy == nullptr || !wx->is_num() || !wy->is_num() ||
 !finite(wx->as_num()) || !finite(wy->as_num())) {
 if (error) *error = "actor entry has invalid x/y: " + side;
 return -1;
 }
 world::Actor actor;
 actor.x = wx->as_num();
 actor.y = wy->as_num();
 const auto* id = e.find("id");
 actor.id = id != nullptr && id->is_str()
 ? id->as_str() : "actor_" + std::to_string(actors.size());
 if (actor.id.empty() || !ids.insert(actor.id).second) {
 if (error) *error = "actor id is empty or duplicated: " + side;
 return -1;
 }
 const auto* visual = e.find("visual_kind");
 if (visual != nullptr && visual->is_str() && !visual->as_str().empty()) {
 actor.visual_kind = visual->as_str();
 }
 const auto* radius = e.find("radius_m");
 if (radius != nullptr && radius->is_num()) {
 actor.radius_m = radius->as_num();
 }
 const auto* height = e.find("height_m");
 if (height != nullptr && height->is_num()) {
 actor.height_m = height->as_num();
 }
 if (!finite(actor.radius_m) || actor.radius_m <= 0.0 ||
 !finite(actor.height_m) || actor.height_m <= 0.0) {
 if (error) *error = "actor " + actor.id + " has invalid dimensions";
 return -1;
 }

 const auto* behavior = e.find("behavior");
 const std::string kind = behavior != nullptr && behavior->is_str()
 ? behavior->as_str() : "waypoints";
 if (!world::ActorBehaviorRegistry::global().create(
 kind, actor.behavior)) {
 if (error) {
 *error = "actor " + actor.id + " has unknown behavior " + kind;
 }
 return -1;
 }
 const auto* speed = e.find("speed_mps");
 if (speed != nullptr && speed->is_num()) {
 actor.behavior.speed_mps = speed->as_num();
 }
 if (!finite(actor.behavior.speed_mps) ||
 actor.behavior.speed_mps < 0.0) {
 if (error) *error = "actor " + actor.id + " has invalid speed";
 return -1;
 }
 const auto* loop = e.find("loop");
 if (loop != nullptr && loop->is_bool()) {
 actor.behavior.loop = loop->as_bool();
 }
 const auto* seed = e.find("seed");
 if (seed != nullptr && seed->is_num() && finite(seed->as_num())) {
 actor.behavior.seed = static_cast<std::uint64_t>(seed->as_num());
 }
 const auto* waypoints = e.find("waypoints");
 if (waypoints != nullptr && waypoints->is_array()) {
 for (const auto& waypoint : waypoints->as_array()) {
 if (!waypoint.is_object()) {
 if (error) *error = "actor " + actor.id + " has invalid waypoint";
 return -1;
 }
 const auto* x = waypoint.find("x");
 const auto* y = waypoint.find("y");
 if (x == nullptr || y == nullptr || !x->is_num() || !y->is_num() ||
 !finite(x->as_num()) || !finite(y->as_num())) {
 if (error) *error = "actor " + actor.id + " has invalid waypoint";
 return -1;
 }
 actor.behavior.waypoints.push_back({x->as_num(), y->as_num()});
 }
 }
 const auto* waits = e.find("waits");
 if (waits != nullptr && waits->is_array()) {
 for (const auto& wait : waits->as_array()) {
 if (!wait.is_num() || !finite(wait.as_num())) {
 if (error) *error = "actor " + actor.id + " has invalid wait";
 return -1;
 }
 actor.behavior.waits_s.push_back(wait.as_num());
 }
 }
 actors.push_back(std::move(actor));
 }

 ActorRouteIssue issue;
 if (!validate_actor_routes(
 actors, session.setup_world().static_boxes(), &issue)) {
 if (error) {
 *error = "actor " + issue.actor_id + " route segment " +
 std::to_string(issue.segment_index) + " intersects static box " +
 std::to_string(issue.box_index) + " after radius expansion: " + side;
 }
 return -1;
 }

 for (auto& actor : actors) {
 session.setup_world().add_actor(actor);
 }
 const int loaded = static_cast<int>(actors.size());
 std::fprintf(stderr, "actors sidecar: %s (%d actors)\n", side.c_str(),
 loaded);
 return loaded;
}

} // namespace lost_signal::sim::scenario
