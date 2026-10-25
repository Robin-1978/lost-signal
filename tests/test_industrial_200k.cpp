#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/planning/astar_planner.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"

namespace {

lost_signal::robot::autonomy::EstimatedState at(double x_m, double y_m) {
 lost_signal::robot::autonomy::EstimatedState state;
 state.valid = true;
 state.x_m = x_m;
 state.y_m = y_m;
 return state;
}

const lost_signal::robot::iface::Place& place(
 const lost_signal::robot::iface::MapSnapshot& map,
 const std::string& place_id) {
 const auto found = std::find_if(
 map.places.begin(), map.places.end(), [&](const auto& candidate) {
 return candidate.place_id == place_id;
 });
 assert(found != map.places.end());
 return *found;
}

void assert_route_clear(
 const lost_signal::robot::autonomy::Route& route,
 const lost_signal::robot::autonomy::LocalMap& lethal) {
 assert(route.valid && route.points.size() >= 2);
 for (std::size_t point = 1; point < route.points.size(); ++point) {
 const auto& from = route.points[point - 1];
 const auto& to = route.points[point];
 const double length = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
 const int samples = std::max(
 1, static_cast<int>(std::ceil(length / (0.25 * lethal.resolution_m))));
 for (int sample = 0; sample <= samples; ++sample) {
 const double blend = static_cast<double>(sample) / samples;
 const double x_m = from.x_m + (to.x_m - from.x_m) * blend;
 const double y_m = from.y_m + (to.y_m - from.y_m) * blend;
 assert(!lethal.is_blocked(x_m, y_m));
 }
 }
}

} // namespace

int main() {
 using namespace lost_signal;
 const std::string map_path =
 std::string(TEST_CONFIG_DIR) + "/worlds/industrial_200k.json";

 sim::eval::WorldFileContent content;
 std::string error;
 assert(sim::eval::load_world_file(map_path, content, &error));
 assert(content.places.size() == 5);

 double min_x = content.boxes.front().x0;
 double min_y = content.boxes.front().y0;
 double max_x = content.boxes.front().x1;
 double max_y = content.boxes.front().y1;
 for (const auto& box : content.boxes) {
 min_x = std::min(min_x, box.x0);
 min_y = std::min(min_y, box.y0);
 max_x = std::max(max_x, box.x1);
 max_y = std::max(max_y, box.y1);
 }
 assert(max_x - min_x == 500.0);
 assert(max_y - min_y == 400.0);
 assert((max_x - min_x) * (max_y - min_y) == 200000.0);

 sim::scenario::ScenarioSession session;
 sim::eval::apply_world_content(session.setup_world(), content);
 session.fit_map_window_to_world();
 assert(sim::scenario::load_actors_sidecar(
 map_path, session, {}, &error) == 4);
 assert(session.setup_world().actors().size() == 4);

 const robot::iface::MapSnapshot map = session.active_map();
 assert(map.has_raster());
 assert(map.width == 2032);
 assert(map.height == 1632);
 assert(static_cast<std::size_t>(map.width) * map.height > 3200000);
 assert(map.places.size() == 5);

 robot::autonomy::AStarPlanner planner;
 const auto& central = place(map, "central_gate");
 const auto& east = place(map, "east_terminal");
 const robot::autonomy::Route to_central = planner.plan(
 at(0.0, 0.0),
 {central.x_m, central.y_m, central.yaw_rad}, map);
 const robot::autonomy::Route to_east = planner.plan(
 at(central.x_m, central.y_m),
 {east.x_m, east.y_m, east.yaw_rad}, map);
 assert(to_central.valid);
 assert(to_east.valid);

 const robot::autonomy::LocalCostmap costmap =
 robot::autonomy::make_global_costmap(
 map, 0.4 + robot::platform::kDefaultNavigationClearanceMarginM,
 robot::platform::kDefaultNavigationSoftInflationRadiusM, true);
 assert(costmap.valid());
 assert_route_clear(to_central, costmap.lethal);
 assert_route_clear(to_east, costmap.lethal);

 std::printf(
 "industrial_200k ok: extent=%.0fx%.0f cells=%ux%u actors=%zu "
 "routes=%zu,%zu\n",
 max_x - min_x, max_y - min_y, map.width, map.height,
 session.setup_world().actors().size(), to_central.points.size(),
 to_east.points.size());
 return 0;
}
