#include "autonomy/planning/global/global_planner.hpp"
#include "autonomy/control/follow.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

class FixtureGlobalPlanner final : public autonomy::IGlobalPlanner {
 public:
 std::string_view id() const noexcept override { return "fixture-straight"; }
 autonomy::GlobalPlannerCapabilities capabilities() const noexcept override { return {}; }

 autonomy::GlobalPlan plan(const autonomy::GlobalPlannerRequest& request) const override {
 if (!request.map.isFree(request.start.position.x, request.start.position.y) ||
 !request.map.isFree(request.goal.position.x, request.goal.position.y)) {
 return {{}, 1, false, "blocked-endpoint"};
 }
 return {{
 {request.start.position, request.start.heading_rad, sim::TravelDirection::Unspecified},
 {request.goal.position, request.goal.heading_rad, sim::TravelDirection::Forward},
 }, 1, true, {}};
 }
};

class InvalidRoutePlanner final : public autonomy::IGlobalPlanner {
 public:
 std::string_view id() const noexcept override { return "fixture-invalid-route"; }
 autonomy::GlobalPlannerCapabilities capabilities() const noexcept override { return {}; }

 autonomy::GlobalPlan plan(const autonomy::GlobalPlannerRequest& request) const override {
 return {{
 {request.start.position, request.start.heading_rad, sim::TravelDirection::Unspecified},
 {request.goal.position, request.goal.heading_rad, sim::TravelDirection::Unspecified},
 }, 1, true, {}};
 }
};

class FixtureCuspPlanner final : public autonomy::IGlobalPlanner {
 public:
 std::string_view id() const noexcept override { return "fixture-cusp"; }
 autonomy::GlobalPlannerCapabilities capabilities() const noexcept override {
 return {true};
 }

 autonomy::GlobalPlan plan(const autonomy::GlobalPlannerRequest& request) const override {
 const auto direction = request.goal.position.x >= request.start.position.x
 ? sim::TravelDirection::Forward
 : sim::TravelDirection::Reverse;
 return {{
 {request.start.position, request.start.heading_rad, sim::TravelDirection::Unspecified},
 {request.goal.position, request.goal.heading_rad, direction},
 }, 1, true, {}};
 }
};

} // namespace

int main() {
 CHECK(autonomy::kGlobalPlannerApiMajor == 2);
 CHECK(autonomy::kGlobalPlannerApiMinor == 0);

 auto registry = autonomy::GlobalPlannerRegistry::with_builtins();
 CHECK(registry.contains("astar-grid"));
 registry.register_factory("fixture-straight",
 [] { return std::make_shared<FixtureGlobalPlanner>(); });
 CHECK(registry.contains("fixture-straight"));

 const auto planner = registry.create("fixture-straight");
 CHECK(planner->id() == "fixture-straight");
 CHECK(!planner->capabilities().heading_aware);

 sim::GridMap map(12, 12, 1.0);
 autonomy::GlobalPlanningSystem planning(planner);
 const std::vector<autonomy::PlanningPose> waypoints{
 {{0.5, 0.5}, 0.0},
 {{4.5, 0.5}, 0.0},
 {{4.5, 4.5}, 1.57},
 };
 const auto route = planning.plan(map, waypoints, {});
 CHECK(route.ok);
 CHECK(route.expanded == 2);
 CHECK(route.path.size() == 3);
 CHECK(route.path.front().position.x == 0.5);
 CHECK(route.path.back().position.y == 4.5);

 // Segment de-duplication removes only the new segment's Unspecified start.
 // Its first motion direction remains on the following point, so lookahead
 // ends exactly at the shared waypoint before the forward/reverse cusp.
 autonomy::GlobalPlanningSystem cusp_planning(std::make_shared<FixtureCuspPlanner>());
 const std::vector<autonomy::PlanningPose> cusp_waypoints{
 {{0.5, 2.5}, 0.0},
 {{6.5, 2.5}, 0.0},
 {{2.5, 2.5}, 0.0},
 };
 const auto cusp_route = cusp_planning.plan(map, cusp_waypoints, {});
 CHECK(cusp_route.ok);
 CHECK(cusp_route.path.size() == 3);
 CHECK(cusp_route.path[1].arrival_direction == sim::TravelDirection::Forward);
 CHECK(cusp_route.path[2].arrival_direction == sim::TravelDirection::Reverse);
 const auto cusp_end = sim::path_direction_section_end(cusp_route.path, 0);
 CHECK(cusp_end == 2);
 const auto cusp_target =
 sim::lookahead_pt_until(cusp_route.path, 0, cusp_end, 0.5, 2.5, 20.0, map);
 CHECK(cusp_target.x == 6.5 && cusp_target.y == 2.5);

 autonomy::GlobalPlanningSystem invalid_planning(std::make_shared<InvalidRoutePlanner>());
 const auto invalid_route = invalid_planning.plan(map, waypoints, {});
 CHECK(!invalid_route.ok);
 CHECK(invalid_route.path.empty());
 CHECK(invalid_route.diagnostic == "invalid-route-contract");

 map.setOccupied(4.5, 4.5);
 const auto blocked = planning.plan(map, waypoints, {});
 CHECK(!blocked.ok);
 CHECK(blocked.path.empty());
 CHECK(blocked.diagnostic == "blocked-endpoint");

 bool threw = false;
 try {
 registry.register_factory("fixture-straight",
 [] { return std::make_shared<FixtureGlobalPlanner>(); });
 } catch (const std::invalid_argument&) {
 threw = true;
 }
 CHECK(threw);

 threw = false;
 try {
 (void)registry.create("missing");
 } catch (const std::invalid_argument&) {
 threw = true;
 }
 CHECK(threw);

 std::cout << "global planner extension tests passed\n";
 return 0;
}
