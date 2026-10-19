#pragma once

#include "robot/autonomy/planning/global/astar.hpp"
#include "robot/autonomy/planning/global/hybrid_astar.hpp"
#include "robot/autonomy/mapping/terrain.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace autonomy {

inline constexpr std::uint32_t kGlobalPlannerApiMajor = 4;
inline constexpr std::uint32_t kGlobalPlannerApiMinor = 0;

struct PlanningPose {
 sim::Vec2 position;
 double heading_rad{0.0};
 // A mission goal is a region, not necessarily a point. Zero requests the
 // exact containing grid cell. Positive values let a planner finish at a
 // collision-free pose inside the authored arrival region.
 double position_tolerance_m{0.0};
};

struct GlobalPlannerRequest {
 const sim::GridMap& map;
 PlanningPose start;
 PlanningPose goal;
 const sim::TraversalCost& traversal_cost;
};

struct GlobalPlan {
 std::vector<sim::RoutePoint> path;
 int expanded{0};
 bool ok{false};
 std::string diagnostic;
};

struct GlobalPlannerCapabilities {
 bool heading_aware{false};
};

// Stable extension boundary for in-process global planners. Implementations
// consume only the request and return owned result data; mission state and
// simulator internals do not cross this interface.
class IGlobalPlanner {
 public:
 virtual ~IGlobalPlanner() = default;
 virtual std::string_view id() const noexcept = 0;
 virtual GlobalPlannerCapabilities capabilities() const noexcept = 0;
 virtual GlobalPlan plan(const GlobalPlannerRequest& request) const = 0;
};

class AStarGlobalPlanner final : public IGlobalPlanner {
 public:
 std::string_view id() const noexcept override { return "astar-grid"; }
 GlobalPlannerCapabilities capabilities() const noexcept override { return {}; }
 GlobalPlan plan(const GlobalPlannerRequest& request) const override;
};

class HybridAStarGlobalPlanner final : public IGlobalPlanner {
 public:
 explicit HybridAStarGlobalPlanner(sim::HybridAStarConfig config) : config_(config) {}
 std::string_view id() const noexcept override { return "hybrid-astar"; }
 GlobalPlannerCapabilities capabilities() const noexcept override { return {true}; }
 GlobalPlan plan(const GlobalPlannerRequest& request) const override;

 private:
 sim::HybridAStarConfig config_;
};

using GlobalPlannerFactory = std::function<std::shared_ptr<const IGlobalPlanner>()>;

class GlobalPlannerRegistry {
 public:
 void register_factory(std::string id, GlobalPlannerFactory factory);
 bool contains(std::string_view id) const;
 std::shared_ptr<const IGlobalPlanner> create(std::string_view id) const;
 std::vector<std::string> ids() const;

 static GlobalPlannerRegistry with_builtins();

 private:
 std::map<std::string, GlobalPlannerFactory, std::less<>> factories_;
};

std::shared_ptr<const IGlobalPlanner> default_global_planner();
std::shared_ptr<const IGlobalPlanner> ackermann_global_planner(
 double wheel_base_m, double max_steer_rad, double map_resolution_m);

// Joins one or more planner segments into a route while keeping the selected
// planner independent of mission progression and presentation. A planner may
// end inside a goal tolerance; the next segment begins at that returned pose.
class GlobalPlanningSystem {
 public:
 explicit GlobalPlanningSystem(std::shared_ptr<const IGlobalPlanner> planner);

 GlobalPlan plan(const sim::GridMap& map, const std::vector<PlanningPose>& waypoints,
 const sim::TraversalCost& traversal_cost) const;

 const IGlobalPlanner& planner() const noexcept { return *planner_; }

 private:
 std::shared_ptr<const IGlobalPlanner> planner_;
};

} // namespace autonomy
