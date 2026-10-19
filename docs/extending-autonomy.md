# Extending autonomy

Lost Signal keeps algorithm extension inside a small, project-owned C++ stack. External middleware, a plugin framework, and a dynamic loader are not required.

## Module boundaries

| Directory | Responsibility | Typical output |
| --- | --- | --- |
| `src/robot/autonomy/localization/` | estimate robot state from observations | estimated pose and motion |
| `src/robot/autonomy/mapping/` | static and observed spatial models | occupancy and terrain cost |
| `src/robot/autonomy/perception/` | track dynamic objects | neutral object states and tracks |
| `src/robot/autonomy/behavior/` | select intent and apply behavior constraints | route intent, hold, or subgoal |
| `src/robot/autonomy/planning/global/` | find a map-scale route | typed route points |
| `src/robot/autonomy/planning/local/` | choose feasible short-horizon motion | velocity, yaw-rate, or steering candidate |
| `src/robot/autonomy/planning/validation/` | assess planned motion without taking control | evidence and diagnostics |
| `src/robot/autonomy/control/` | track the selected path or motion target | body or actuator target |

DWA and the Ackermann and holonomic rollout samplers are local planners. Path following and PID belong to control. Deterministic patrol and wander motion describe simulated world actors, so they live under `src/sim/game/mission/world/` rather than the autonomy stack.

Local planners must distinguish a rolling lookahead from a terminal pose. The latter carries position and speed
tolerances and may carry a heading tolerance. For holonomic command space, terminal sampling must include zero and
small signed body-frame translations so an overshoot can be corrected without turning the body through 180 degrees.
Tests should assert final planar speed, yaw rate and heading as well as position; `inside radius` alone is not an
arrival oracle.

Waiting at a safe pocket is a behavior decision. The behavior layer selects the pocket, decides when to hold,
commits to a crossing, and releases or falls back when observations change. A local planner such as DWA only checks
and executes the short-horizon motion toward the selected pocket or through the committed crossing; it does not own
the long-duration traffic state machine.

That separation also creates a strict promotion gate: behavior admission evaluates a time-parameterized route
schedule, while the local validator independently checks every exact executable rollout. The research-base
crossing uses `control` authority only because stop-pose, braking-tail, prediction, and deterministic mission
contracts are present. See [Behavior architecture](behavior-architecture.md).

The CMake targets make the same boundaries visible. `lost_signal_robot_stack` aggregates only deployable ROBOT targets. `lost_signal_sim_vehicle`, `lost_signal_sim_sensors`, and `lost_signal_sim_physics` depend on it through `lost_signal_sim_hardware`; `lost_signal_sim_game` adds authored content and simulation composition. The reverse dependency is rejected by tests and by the robot-only build.

This organization follows two useful industry patterns while remaining smaller than either framework:

- [Autoware's architecture](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/design/autoware-architecture-v1/index.md) separates sensing, map, localization, perception, planning, control, and vehicle interfaces so modules can be replaced behind defined contracts.
- [Autoware's planning design](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/design/autoware-architecture-v1/components/planning/index.md) separates mission, behavior, motion, and validation concerns. It sends a trajectory from planning to control.
- [Nav2 Core](https://github.com/ros-navigation/navigation2/tree/main/nav2_core) gives algorithm authors narrow global-planner and controller interfaces while task servers own framework plumbing.

Lost Signal adopts the boundaries and narrow interfaces. It does not adopt their middleware, process model, package count, or deployment machinery.

## Global planner API v4

`IGlobalPlanner` is the extension boundary for map-scale planning. A planner receives a `GlobalPlannerRequest` containing only:

- the planning map;
- start and goal poses;
- the traversal-cost function used by the current robot and terrain.

It returns an owned `GlobalPlan`: success, a route, expanded-node count, and an English diagnostic. Every
`RoutePoint` contains position, nominal body heading, and the forward or reverse direction used to arrive at that point.
The first point uses `Unspecified`; every later point must name a direction. `GlobalPlanningSystem` rejects a
successful result that violates this contract. The planner cannot access mission progression, renderer state,
MuJoCo bodies, future actor truth, or actuator internals.

The API version is `kGlobalPlannerApiMajor == 4`. This is a source-level C++ contract. It is not a stable
shared-library ABI across compilers. Lost Signal is under active development, so unreleased API versions are replaced
directly rather than preserved through compatibility shims. The registry provides deterministic in-process
selection; loading untrusted shared libraries at runtime is outside v4.

The built-in planners share this result contract without sharing an algorithm. Grid A* searches an eight-connected
2D cost map, emits forward segments, and derives the nominal body heading from the route tangent. Its final point
inherits the incoming grid edge heading; it does not claim to satisfy `goal.heading_rad`. Hybrid A* searches position,
heading, and gear under Ackermann curvature constraints. Its returned endpoint is the final collision-checked searched
state inside the requested goal tolerance. It never appends an unchecked straight edge to the authored point.
When a route has multiple authored waypoints, `GlobalPlanningSystem` starts each new segment at the preceding
segment's returned terminal pose. This keeps the composite route continuous even when a tolerance-aware planner does
not end at the waypoint centre.

## Add a planner

Implement the interface in your own source file:

```cpp
#include "robot/autonomy/planning/global/global_planner.hpp"

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

mission::ScenarioSessionOptions options;
options.global_planner = planners.create("theta-star");
mission::ScenarioSession simulation(catalog, level_id, build, options);
```

No branch is added to `ScenarioSession`. The same injected planner runs through graphical missions and headless experiments. Chassis-specific defaults still select grid A* or Hybrid A* when no custom planner is supplied.

A planner should keep all tuning explicit, respect map occupancy and the supplied traversal cost, return deterministic results for identical inputs, and explain failures through `diagnostic`. Add focused contract tests and at least one mission-level test when behavior changes. `tests/new/test_global_planner_extension.cpp` is the smallest working provider example.

## Future extension seams

Global planning is the first versioned algorithm registry. Local planners and controllers already have clear directory and data boundaries, but they do not yet expose equivalent versioned registries. Add those only when a second implementation needs selection; avoid abstract interfaces with one consumer and no replacement pressure.

### TEB evaluation

Timed Elastic Band is a useful future local-planner experiment, but it is not
part of the current release. The maintained ROS implementation describes TEB
as an online optimization of trajectory time, obstacle separation and
kinodynamic constraints, with additional work for distinctive topologies and
car-like robots. That makes it especially relevant to narrow passages,
explicit arrival timing and Ackermann maneuvers. It also introduces a nonlinear
optimizer, substantially more tuning and a new failure surface; importing the
ROS package would additionally bring ROS and g2o-facing integration that Lost
Signal does not need.

The current safety defect class is independent of the candidate optimizer:
state estimation, actuator response, stopping authority and the final swept
trajectory validator must agree for DWA, TEB or any later solver. Adding TEB
before that contract is closed could produce smoother paths while preserving
the same unsafe stopping assumption. Lost Signal therefore keeps the classic
DWA and chassis-specific rollout planners for the release gate.

When a named teaching experiment requires TEB, implement a project-owned
provider behind a versioned local-planner API. It must consume
`LocalPlanningRequest`, emit the same `ExecutableTrajectory`, use the runtime
chassis capability contract, and pass the existing independent validator. Its
acceptance experiment compares DWA and TEB on identical map, observation trace,
seed and build using completion, minimum static/dynamic clearance, solve time,
oscillation count, path length, final pose error and command continuity. A TEB
trajectory receives no collision or stopping exception merely because its
optimizer reports convergence.

References used for this decision:

- [TEB local planner](https://github.com/rst-tu-dortmund/teb_local_planner) and
  its cited trajectory-optimization and car-like-robot publications;
- [Nav2 DWB controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_dwb_controller),
  which separates trajectory generators from critic plugins;
- [Nav2 MPPI controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller),
  which uses motion-model plugins, forward simulation, critics and a published
  optimal trajectory for downstream checking.

The DWB/MPPI separation is closer to Lost Signal's existing sampler plus
independent-validator architecture than a direct port of the ROS TEB package.
It is an architectural reference, not a ROS2 integration plan.

Hardware remains separate. Algorithms emit neutral motion targets, LSRI validates and transports commands, and a vehicle adapter converts them to platform-specific actuation. Native providers implement that boundary without moving algorithms into device-specific packages.
