# Extending autonomy

Lost Signal keeps algorithm extension inside a small, middleware-neutral C++ stack. ROS 2, DDS, pluginlib, and a dynamic loader are not required. An optional bridge may translate data at the LSRI boundary without changing an algorithm.

## Module boundaries

| Directory | Responsibility | Typical output |
| --- | --- | --- |
| `autonomy/localization/` | estimate robot state from observations | estimated pose and motion |
| `autonomy/mapping/` | static and observed spatial models | occupancy and terrain cost |
| `autonomy/perception/` | track dynamic objects | neutral object states and tracks |
| `autonomy/behavior/` | select intent and apply behavior constraints | route intent, hold, or subgoal |
| `autonomy/planning/global/` | find a map-scale route | typed route points |
| `autonomy/planning/local/` | choose feasible short-horizon motion | velocity, yaw-rate, or steering candidate |
| `autonomy/planning/validation/` | assess planned motion without taking control | evidence and diagnostics |
| `autonomy/control/` | track the selected path or motion target | body or actuator target |

DWA and the Ackermann and holonomic rollout samplers are local planners. Path following and PID belong to control. Deterministic patrol and wander motion describe simulated world actors, so they live under `mission/world/` rather than the autonomy stack.

The CMake targets make the same boundaries visible. `lost_signal_autonomy`, `lost_signal_vehicle`, and `lost_signal_runtime` are separate targets. `lost_signal_stack` is a convenience target that aggregates them. `lost_signal_mission` composes the stack with authored content and simulation.

This organization follows two useful industry patterns while remaining smaller than either framework:

- [Autoware's architecture](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/design/autoware-architecture-v1/index.md) separates sensing, map, localization, perception, planning, control, and vehicle interfaces so modules can be replaced behind defined contracts.
- [Autoware's planning design](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/design/autoware-architecture-v1/components/planning/index.md) separates mission, behavior, motion, and validation concerns. It sends a trajectory from planning to control.
- [Nav2 Core](https://github.com/ros-navigation/navigation2/tree/main/nav2_core) gives algorithm authors narrow global-planner and controller interfaces while task servers own framework plumbing.

Lost Signal adopts the boundaries and narrow interfaces. It does not adopt their middleware, process model, package count, or deployment machinery.

## Global planner API v2

`IGlobalPlanner` is the extension boundary for map-scale planning. A planner receives a `GlobalPlannerRequest` containing only:

- the planning map;
- start and goal poses;
- the traversal-cost function used by the current robot and terrain.

It returns an owned `GlobalPlan`: success, a route, expanded-node count, and an English diagnostic. Every
`RoutePoint` contains position, nominal body heading, and the forward or reverse direction used to arrive at that point.
The first point uses `Unspecified`; every later point must name a direction. `GlobalPlanningSystem` rejects a
successful result that violates this contract. The planner cannot access mission progression, renderer state,
Bullet bodies, future actor truth, or actuator internals.

The API version is `kGlobalPlannerApiMajor == 2`. This is a source-level C++ contract. It is not a stable
shared-library ABI across compilers. Lost Signal is under active development, so unreleased API versions are replaced
directly rather than preserved through compatibility shims. The registry provides deterministic in-process
selection; loading untrusted shared libraries at runtime is outside v2.

The built-in planners share this result contract without sharing an algorithm. Grid A* searches an eight-connected
2D cost map, emits forward segments, and derives the nominal body heading from the route tangent. Its final point
inherits the incoming grid edge heading; it does not claim to satisfy `goal.heading_rad`. Hybrid A* searches position,
heading, and gear under Ackermann curvature constraints. Its exact terminal anchor inherits the final searched
state's heading and gear rather than replacing them with an unearned requested goal heading.

## Add a planner

Implement the interface in your own source file:

```cpp
#include "autonomy/planning/global/global_planner.hpp"

class ThetaStarPlanner final : public autonomy::IGlobalPlanner {
 public:
 std::string_view id() const noexcept override { return "theta-star"; }
 autonomy::GlobalPlannerCapabilities capabilities() const noexcept override {
 return {};
 }

 autonomy::GlobalPlan plan(
 const autonomy::GlobalPlannerRequest& request) const override {
 // Read request.map, request.start, request.goal, and
 // request.traversal_cost. Return owned typed route points and diagnostics.
 return {{
 {request.start.position, request.start.heading_rad,
 sim::TravelDirection::Unspecified},
 {request.goal.position, request.goal.heading_rad,
 sim::TravelDirection::Forward},
 }, 1, true, {}};
 }
};
```

Register and inject it at the composition root:

```cpp
auto planners = autonomy::GlobalPlannerRegistry::with_builtins();
planners.register_factory("theta-star", [] {
 return std::make_shared<ThetaStarPlanner>();
});

mission::MissionSimulationOptions options;
options.global_planner = planners.create("theta-star");
mission::MissionSimulation simulation(catalog, level_id, build, options);
```

No branch is added to `MissionSimulation`. The same injected planner runs through graphical missions and headless experiments. Chassis-specific defaults still select grid A* or Hybrid A* when no custom planner is supplied.

A planner should keep all tuning explicit, respect map occupancy and the supplied traversal cost, return deterministic results for identical inputs, and explain failures through `diagnostic`. Add focused contract tests and at least one mission-level test when behavior changes. `tests/new/test_global_planner_extension.cpp` is the smallest working provider example.

## Future extension seams

Global planning is the first versioned algorithm registry. Local planners and controllers already have clear directory and data boundaries, but they do not yet expose equivalent versioned registries. Add those only when a second implementation needs selection; avoid abstract interfaces with one consumer and no replacement pressure.

Hardware remains separate. Algorithms emit neutral motion targets, LSRI validates and transports commands, and a vehicle adapter converts them to platform-specific actuation. A ROS 2 bridge can wrap these contracts later without moving the algorithms into ROS packages.
