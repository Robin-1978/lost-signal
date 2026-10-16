#include "autonomy/planning/global/global_planner.hpp"

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
 return {{request.start.position, request.goal.position}, 1, true, {}};
 }
};

} // namespace

int main() {
 CHECK(autonomy::kGlobalPlannerApiMajor == 1);

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
 CHECK(route.path.front().x == 0.5);
 CHECK(route.path.back().y == 4.5);

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
