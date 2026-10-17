# Content direction: autonomy playgrounds

Lost Signal keeps its existing chapters as the **Guided Campaign**. Their story, order, prerequisites, and teaching
progression remain intact. New systemic experiences live in an optional **Autonomy Playgrounds** collection selected
from the product entry flow. A playground has no campaign prerequisite and does not become a mandatory chapter.

## Product promise

A playground gives the player a physical problem whose difficult actions belong to autonomy. The player chooses the
intent, equipment, priorities, and constraints; the robot senses, orders tasks, plans, controls, waits, recovers, and
drives. Every run should answer four questions without requiring documentation:

1. What is the robot trying to achieve?
2. What changed in the physical world?
3. Why did the robot choose this action?
4. What single change should I try on the same seed?

The content design is allowed to require capabilities that do not exist yet. Mission structure, interactive objects,
task allocation, terrain semantics, sensors, and algorithms follow the experience rather than limiting it. Runtime
work must still preserve truthful state, deterministic evidence, and headless execution. Development-time interfaces
may be replaced instead of wrapped for compatibility.

## Experience rules

- **Begin with a fantasy and a verb.** Recover, search, guide, build, rescue, and compare produce different play;
  changing the destination of another delivery does not.
- **Ask for commitment before explanation.** The player chooses a build or priority and predicts a consequence before
  dispatch. The debrief then compares that idea with recorded evidence.
- **Show the consequence in the world first.** Mud throws the vehicle off line, fruit shifts the center of gravity,
  a cart occupies a future gap, or a battery reserve forces a return. AI View explains the same event afterward.
- **Change several variables in the world, but one variable per comparison.** A rich encounter may include terrain,
  load, and traffic. The retry flow holds scenario and seed fixed and names the changed build, policy, or parameter.
- **Make success graded.** Completion alone is too weak. Time, energy, damage, coverage, risk, missed tasks, and
  recovery quality let two safe strategies remain meaningfully different.
- **Keep presentation downstream of simulation.** Animation, particles, audio, and prose may amplify a measured event;
  they never create a pickup, collision, risk value, or decision.

These rules combine exploration with explicit reflection. Research on interactive multivariable simulations reports
that instruction-first can improve immediate performance while exploration-first can improve transfer; Lost Signal
uses a short pre-run prediction, autonomous exploration, and a causal debrief instead of relying on unguided play
alone. Explorable Explanations similarly motivates direct manipulation and immediate visible consequence. These are
design inputs, not validation evidence for this product.

## Playground portfolio

| Playground | Player fantasy and verb | Autonomous problem | Physical reveal | Comparison |
|---|---|---|---|---|
| **Orchard Scramble** | Recover windfall fruit before the storm; collect | terrain-aware task ordering under changing payload | mud slip, stacked load, fruit shock, rollover margin | chassis, tires, harvest doctrine |
| **Firefly Survey** | Reconnect intermittent field beacons in fog; search | coverage, belief update, return reserve | observations appear and disappear with viewpoint and range | sensor pose, search policy, reserve |
| **Runaway Carts** | Guide depot carts through a set of gates; shepherd | temporal reservation, gap selection, deadlock recovery | predicted paths cross before bodies do | priority rule, planner horizon |
| **Bridge the Washout** | Spend a few supports, then prove the route; build | traversability and route feasibility after world edits | a support changes slope, clearance, and reachable space | placement plan, chassis |
| **Floodplain Rescue** | Restore stranded pumps before batteries expire; rescue | task scheduling with deadlines, payload, recharge, and wet grip | water and load change traction and energy reserve | task priority, tire, reserve |
| **Robot Field Day** | Enter one seeded course with different machines; compare | feasible motion through a shared semantic task | each chassis exposes a different path and actuator strategy | chassis or planner pair |

The second playground must exercise a different verb before a general content framework is declared stable. Reuse is
earned when two experiences need the same semantic contract.

## First graybox: Orchard Scramble

### Fantasy and learning question

A storm has knocked ripe fruit into collection pockets across a hillside orchard. RO-1 must recover the fruit and
return to the packing pad before conditions close. The learning question is:

> Why can the best next target change when terrain, chassis feasibility, payload, and center of gravity are part of
> the cost rather than distance alone?

Agricultural ground-robot literature commonly separates point-to-point navigation from coverage planning and treats
safe autonomous navigation as a system of localization, mapping, motion control, and planning. Field-robot research
also demonstrates LiDAR navigation through occluded crop rows. Orchard Scramble turns those real constraints into a
short comparative experience, while avoiding a claim that the graybox validates an agricultural robot.

### Player contract

The player chooses:

- a chassis and tire set;
- one harvest doctrine: **Safest Fruit**, **Fastest Basket**, or **Best Fruit**;
- a pre-run prediction about the first target or the effect of the final load.

RO-1 owns target selection, global and local planning, steering, braking, pickup, the decision to stop collecting,
and the return to the packing pad. There is no manual driving around mud, slopes, trees, or failed approaches.

### Ninety-second loop

1. **Read the hillside.** The camera reveals the packing pad, firm outer row, muddy swale, side-slope pocket, and
   narrow ridge. Fruit clusters pulse once, then become ordinary physical landmarks.
2. **Commit.** The player chooses a build and doctrine, predicts a consequence, and dispatches.
3. **Collect.** RO-1 selects a fruit cluster, plans a feasible route, stops inside its pickup zone, and loads a harvest
   tray. The next target is selected from the new position and new load state.
4. **Feel accumulation.** Each tray increases mass and raises the combined center of gravity. Acceleration, stopping,
   energy, cross-slope margin, and route cost respond to the updated state.
5. **Return under pressure.** The planner preserves enough time to reach the packing pad. It may leave low-priority
   fruit rather than turn a good run into a timeout.
6. **Debrief.** The result shows apples recovered, fruit integrity, time, energy, slip exposure, maximum rollover risk,
   first-target choice, visit order, and the reason collection stopped.

### Graybox topology

The first seed uses five authored fruit regions and a final packing pad:

| Region | Route character | Intended tension |
|---|---|---|
| Outer row | long, firm, low slope | dependable but slow |
| Muddy swale | short and low | tire grip versus time |
| Side hill | moderate distance and cross-slope | load height versus rollover margin |
| Orchard aisle | tight tree spacing | Ackermann turning feasibility versus compact chassis |
| Ridge pocket | high-value fruit behind a narrow approach | whether value justifies risk and return cost |

Each pickup represents a visible tray of multiple apples so its mass can truthfully affect a 20–30 kg rover. Content
records apple count, tray mass, load height, fruit value, fragility, pickup radius, and an authored marker. The current
graybox uses fixed marker positions so the route comparison is directly reproducible. A later scenario variant may
use the seed to select positions from validated regions, but no such random placement is claimed here.

### Required runtime semantics

- **Selectable task set:** collection goals are unordered candidates; the packing pad is a terminal goal. Completion
  tracking uses stable task IDs and cannot equate “number completed” with “next array index.”
- **Deterministic task ordering:** a selector evaluates route feasibility, traversal cost, expected time, risk, value,
  and return reserve according to the chosen doctrine. Equal scores break by stable task ID.
- **Dynamic payload state:** current payload mass, first moment, and center-of-gravity height belong to runtime state
  and replay snapshots. A pickup updates chassis dynamics, rigid-body mass/inertia, terrain capability, rollover
  calculation, energy, and later route costs before replanning.
- **Collectible truth:** a collectible has available, carried, delivered, or damaged state. Rendering reads that state;
  proximity animation cannot award it.
- **Fruit integrity:** measured collision impulse, acceleration/jerk exposure, roll exposure, and configured fragility
  reduce integrity. A decorative shake has no scoring effect.
- **Run evidence:** headless output records selected doctrine, first target, visit order, abandoned tasks, payload
  history, apples, integrity, slip exposure, and rollover margin.

### Graybox acceptance gate

- Guided Campaign content remains byte-for-byte addressable and playable; Orchard is selected independently as a
  Playground and has no campaign prerequisite.
- The same scenario and seed reproduce task positions, visit order, payload history, metrics, and terminal result.
- At least two validated builds or doctrines produce different first targets or visit orders for an explainable
  physical reason.
- Pickup changes mass and center of gravity in the authoritative physics path before the next route is planned.
- The robot returns autonomously; the player never supplies steering or throttle.
- Failure distinguishes timeout, no feasible remaining task, failure to preserve return reserve, collision, rollover,
  and fruit damage.
- The complete loop works headlessly with simple geometry. Permanent models, materials, particles, weather rendering,
  and audio wait for the M2 visual and audio pass.

## Editor-facing capability palette

The editor composes registered, deterministic capabilities. C++ owns algorithms, physical interaction classes, and
stable state contracts; data owns their selection, parameters, actors, teams, triggers, objectives, scoring,
presentation, and level layout. A new map or game theme must not require a C++ branch. A new interaction class, such
as a projectile or a movable construction piece, is implemented once as a generic mechanic and can then be reused by
data-authored games.

The planned palette is grouped by contract rather than by game:

| Catalog | Algorithms and mechanics exposed to data |
|---|---|
| Route and task planning | grid A*, Hybrid A*, bounded multi-goal A*, greedy/value/risk task ordering, coverage lanes, frontier search, deadline scheduling, assignment by Hungarian or auction methods |
| Local motion and coordination | DWA and chassis-feasible rollout, collision prediction, time reservations, reciprocal avoidance, convoy/follow, formation, pursuit, evade, intercept, and deadlock recovery |
| World-object motion | patrol, loop, dwell, seeded wander, spline/rail, orbit, waypoint follow, target pursuit, flee, formation slot, and physics-driven free body |
| Projectiles and moving tools | constant-velocity ray or body, gravity ballistics, drag and wind profile, analytic lead intercept, proportional-navigation homing, fuse/lifetime, impact, bounce, and material-driven response |
| Scenario rules | zones, triggers, timers, counters, spawn/despawn, teams, inventory/payload, capture/escort, health or integrity, damage filters, scoring, success/failure expressions, and deterministic timelines |
| Estimation and prediction | constant velocity/acceleration and turn-rate predictors, Kalman-family tracking where a lesson needs it, occupancy/belief updates, and uncertainty-aware evidence |
| Sensors | timed rotating single-line LiDAR, later multi-line and solid-state scan patterns, IMU/encoder/GNSS/camera profiles, material return and intensity, occlusion, beam divergence, noise, bias, dropout, latency, and saturation |
| Presentation | data-bound models, materials, decals, trails, particles, lights, cameras, animation state, audio cues, decision traces, and localized hints |

This palette is a roadmap, not a request to implement every algorithm before making another game. The first vertical
slice should add only the smallest reusable set it actually exercises. A future autonomous arena, for example, can
combine teams, target tracking, occlusion, pursuit, lead interception, ballistic projectiles, collision and scoring
without introducing an `arena` or `tank` branch in the runtime. The same mechanics can support harmless pulse tags,
foam-ball robotics, rescue launchers, or other themes.

The editor should expose algorithm IDs from runtime registries instead of duplicating hard-coded enums. It validates
units, required parameters, capability compatibility, deterministic seeds, and referenced assets before a scenario
can run. Headless execution consumes the same authored scenario as the graphical client.

## Sources used for this direction

- [PhET research](https://phet.colorado.edu/en/research) on research-based interactive simulation design.
- [Explorable Explanations](https://worrydream.com/ExplorableExplanations/) on active reading and changing a system
  while seeing its consequence.
- [Learning about multivariable causality with interactive simulations](https://doi.org/10.1007/s11251-025-09726-7)
  on the tradeoff between exploration-first immediate performance and later transfer.
- [Path Planning for Ground Robots in Agriculture: a Short Review](https://doi.org/10.1109/ICARSC49921.2020.9096177)
  on navigation, coverage, and planning as linked agricultural autonomy problems.
- [Simulation of an Autonomous Mobile Robot for LiDAR-Based In-Field Phenotyping and Navigation](https://doi.org/10.3390/robotics9020046)
  on simulated LiDAR operation in occluded crop rows.

External references motivate questions and conventions. Only Lost Signal's deterministic tests, calibrated profiles,
held-out measurements, HIL, or real trials can validate its own behavior.
