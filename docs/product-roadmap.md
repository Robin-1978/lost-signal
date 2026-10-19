# Product roadmap

This roadmap turns the current simulation into a polished, open-source autonomy learning experience. Work proceeds in the order below. A later stage starts only after the preceding stage has a reviewable result and passes its acceptance gate.

## Product outcome

The first public release should let a new player give RO-1 a high-level intention, watch it handle a dynamic encounter autonomously, understand why it waited or moved, change one relevant variable, and reproduce the result. The experience must remain useful at three depths:

- **Explore:** the field scene communicates the immediate situation through motion, landmarks, light, and sound.
- **Learn:** AI View connects observations, predictions, constraints, and actions.
- **Verify:** headless output and a run artifact make the same encounter reproducible and comparable.

The campaign remains deterministic and fully usable without a language model. New sensors, general scenario overlays, full SLAM, VLA, multi-process transport, HIL, and native robot drivers remain outside this release path until a concrete learning or deployment case requires them.

## Content formats beyond point-to-point delivery

The runtime should support different kinds of play rather than reskinning every lesson as a trip from A to B. Each format must give autonomy a different question to solve while the player continues to choose intent, equipment, priorities, or constraints instead of steering the robot through the answer.

| Format | Example experience | Player decisions | Autonomy and physics lesson |
|---|---|---|---|
| Directed recovery under a changing world | **Stormglass Protocol:** choose the recovery order of three relay cores during an ion storm | next core, hardware, risk hypothesis | human goal selection versus autonomous motion, payload/CoG, grip, slope, occlusion, prediction and time pressure |
| Collection and route ordering | **Orchard Scramble:** recover fallen apples across a muddy hillside before weather closes in | harvest zone, chassis, tires, collection priority, safety/time preference | multi-goal ordering, terrain cost, feasible motion, traction, payload growth, rollover and fruit-integrity tradeoffs |
| Search and discovery | **Firefly Survey:** locate intermittent signal nodes in fog | search boundary, sensor package, coverage preference, return reserve | coverage planning, observation range, uncertainty, map discovery, energy-aware return |
| Herding and flow | **Runaway Carts:** guide autonomous carts through gates without entering their predicted paths | gate order, waiting zones, traffic priority | multi-agent prediction, temporal reservations, safe gaps, deadlock recovery |
| Construction and verification | **Bridge the Washout:** place a limited set of ramps or supports, then dispatch RO-1 | where to spend construction pieces, chassis and load | traversability, slope and clearance, route feasibility, experiment/retry |
| Recovery under constraints | **Floodplain Rescue:** collect scattered power cells and serve stranded equipment | task priority, energy reserve, route risk | scheduling, recharging, wet traction, payload and time windows |
| Comparative challenge | **Robot Field Day:** run one seeded course with different chassis or planners | comparison pair and metric target | feasible-motion contracts, algorithm comparison, regression evidence |

The first systems prototype after the release foundation was **Orchard Scramble** because collection, changing load,
terrain choice, and fruit damage can create a physical and strategic loop that is unlike point-to-point delivery.
The design is not limited to features the current runtime already exposes. If truthful apples, payload, terrain, or
autonomous task ordering require a new contract, the runtime changes with the slice; unreleased interfaces receive
no compatibility shim.

Its core loop is:

1. Inspect a seeded orchard with apples distributed across paved rows, mud, side slopes, and a narrow ridge.
2. Choose a chassis/build and a collection priority such as safest fruit, fastest basket, or lowest energy.
3. Dispatch the autonomous robot; it selects the visit order and feasible local motion.
4. Watch the carried mass and center of gravity change after each pickup.
5. Debrief fruit integrity, apples collected, route time, energy, slip, and rollover margin.
6. Retry the same seed with one changed variable or compare another chassis through the run-artifact tooling.

Apples are physical mission items with deterministic pickup radii and payload effects. Their score responds to measured impact, roll, and acceleration; decorative animation does not alter the result. The mode earns inclusion only if at least two builds produce visibly different, explainable strategies on the same seed.

The implementation boundary is data first. Level data owns the task set, payload values, return reserve, and
deterministic environment keyframes. Map data owns 2.5D terrain, surface materials, obstacles, and actors. Scene data
owns prefabs, goal-state visuals, attached payload layout, effects, lighting, and instances. Robot data owns the
physical assembly, motion roles, and cargo attachment frame. Task-policy and audio catalogs own the offered
algorithms and event-to-cue mappings. C++ supplies reusable loaders, registries, simulation contracts, and renderers;
it does not name apples, orchard crates, or harvest strategies.

## Current status and canonical execution plan

Lost Signal is an open-source **alpha platform** with a strong simulation and autonomy foundation. It is not yet the
polished first public product described above. The platform can already support deterministic learning experiments;
the remaining work is to close one autonomy safety gap, rerun the release evidence, and turn one implemented scenario
into a clear, attractive, explainable experience.

The completed foundation includes:

- MuJoCo 3.16.0 as the single production physics engine, with articulated bodies, height fields, physical contacts,
 and a 1 ms contact substep;
- the deployable ROBOT / LSRI / SIM boundary, including mixed provider ownership and a ROBOT-only build;
- the unloaded RO-1 `typical` reference envelope, the 16-line LiDAR, RO-2 articulated Ackermann and service brake,
 and independent RO-3, RO-4 and RO-5 reference gates;
- the LSRM 2.5D map contract and editor output for occupancy, elevation, surface classes and semantic places;
- the Vehicle Proving Ground, Orchard Scramble, Bridge regression fixture, Manipulation Lab and Stormglass Protocol;
- deterministic headless execution, replay/run artifacts, content validation, localization, mapping, prediction,
 global planning, local rollout planning, independent trajectory validation and actuator control.

The [reference platform](reference-platform.md) and [simulation model audit](simulation-model-audit.md) define the
limits of these claims. Typical parameters are engineering assumptions until a named physical target supplies
calibration data and held-out validation. The architecture supports transfer experiments; it does not prove
sim-to-real accuracy.

Existing campaign chapters remain available. The Vehicle Proving Ground is an engineering instrument, Orchard is a
systems prototype, Bridge is retained as a construction/map/physics regression, and Manipulation Lab demonstrates a
platform capability. **Stormglass Protocol is the sole candidate for the first polished gameplay experience.**

Work proceeds through the following queue. A later package starts only when the preceding exit gate is met.

### P0 — Close the current autonomy safety gap

The remaining known integration blocker is `town_freight --recommended`. Collision avoidance is conservative, but
the robot can wait until timeout at a safe pose that cannot continuously observe the traffic corridor. A vanished
track correctly becomes unknown; absence of a track must never be treated as proof that the crossing is clear.

Implement a ROBOT-side observability contract that records which conflict-zone entrances and corridor cells are
covered by current LiDAR free-space rays. Select holding poses from static map geometry, route geometry, sensor field
of view and occlusion so a pose must be both safe and capable of collecting release evidence. Keep uncovered or
occluded space unknown. Simulation actor truth, authored actor IDs and a pre-scripted stop point remain outside this
decision.

Add focused tests for an occluded safe holding pose, a visible corridor with consecutive clear observations, a target
that leaves the field of view before the corridor is observed clear, and the fallback path when conflict-zone
semantics are absent. Then rerun `town_freight` with fixed seed and require success, zero collisions, declared dynamic
clearance and no timeout.

**Exit:** crossing release is based on positive, time-aligned observation evidence; the scenario completes without
weakening collision margins, changing world truth or inserting scenario-specific decision code.

### P1 — Re-establish the release candidate evidence

After P0 is frozen, run the gates in [Validation](validation.md): GCC 13 Release and the complete CTest suite with
parallelism 8; ROBOT-only build and tests; schema, content, locale and generated-artifact checks; identical-seed run
artifact reproduction; graphical input, shutdown and recommended-experience smoke tests; Clang 18 compatibility; and
ASan/UBSan last. Record the exact revision, commands, counts and results. A historical green baseline does not cover a
new autonomy or simulation revision.

**Exit:** the current clean revision has one complete release record. It may be published as an Alpha / Developer
Preview with the claim boundaries stated in the simulation audit.

### P2 — Validate Stormglass as the gameplay vertical slice

Stormglass already provides three player-selected signal cores, autonomous goal execution, payload and centre-of-
gravity changes, authored storm keyframes, changing grip and a moving service cart. Use the playtest protocol to find
where a new player cannot identify the danger, choose a goal, understand waiting, complete or diagnose the run, and
explain one causal effect. Revise the data-authored task loop and information rhythm only where evidence shows a
problem. Existing campaign content remains selectable.

**Exit:** three fresh players can complete or diagnose the loop without steering through the autonomy problem; at
least two target orders are safely completable on one seed and produce measurably different route, time, payload,
clearance or grip evidence.

### P3 — Give the approved loop a distinctive presentation

Refine Stormglass with authored models, readable landmarks, vehicle details, materials, weather, bounded particles,
lighting, camera staging and state-driven sound. Preparation, target choice, risk, pickup, waiting, release, damage and
debrief must remain readable without opening AI View. Every effect reads snapshot state and cannot manufacture a
physics or autonomy outcome. Headless execution remains independent of graphical assets.

Apply the responsive information-design gate below to real flows: compact symbols, measured localized text, hover
hints, responsive panels, immediate mouse input and clean shutdown. Broad screenshot work remains secondary to a
concrete blocked flow.

**Exit:** the complete experience is visually coherent, responsive and understandable in English and Simplified
Chinese; deterministic graphical smoke tests contain no clipping, missing assets or state/render disagreement.

### P4 — Complete the causal learning and comparison loop

Upgrade AI View and the debrief from internal-state display to a causal chain: observation, prediction, rejected
candidates, selected constraint, action and measured outcome. Let a player rerun the same seed after changing one
declared variable and compare complete run artifacts and key trajectory metrics. Keep simulator truth and onboard-
observable estimates visually distinct.

**Exit:** a player can explain why the robot waited, moved or stopped, change one variable, and see a reproducible
causal difference rather than only a different success boolean.

### P5 — Generalize experiments and deterministic fault injection

Extract only the needs demonstrated by the finished content into Scenario, Experiment, Baseline, Metric and Diff
contracts. Add seeded provider-level faults for delay, loss, reordering, stale timestamps, sensor noise, actuator lag,
saturation, slip and changing load. Preserve the in-process fixed-step runtime while keeping LSRI provider boundaries
suitable for future threads, processes and native devices.

**Exit:** a content-authored experiment can compare algorithm or configuration variants without UI-specific mission
logic, and faults traverse the same validation and safety paths as ordinary observations and commands.

### P6 — Expand capabilities only for a named question

Add further sensors, algorithms or execution modes only when a specific learning experience or measured platform
target supplies acceptance criteria. TEB remains a planned DWA comparison provider behind the shared
`LocalPlanningRequest` → `ExecutableTrajectory` → `TrajectoryValidator` contract. Camera, production SLAM,
solid-state LiDAR, multi-process LSRI Wire, HIL, VLA, native drivers, ABS, parking brakes, identified tire laws and
more complex manipulation remain candidate work rather than release dependencies.

**Exit:** each new capability ships with a named scenario, quantitative evidence, documented limits and a smaller
project-owned interface than any external system it may bridge to.

The [verification strategy](verification-strategy.md) is a continuous gate across P0--P6. Contract correctness,
mathematical invariants, independent oracles, deterministic system evidence, fault boundaries, and physical
validation accumulate in that order. Correct an unreleased algorithm or interface directly, update every caller and
test, and then update the content expectation; do not preserve an incorrect result through compatibility behavior.

### Responsive information-design gate

Player-facing layouts measure localized text instead of assuming Chinese-length copy. Chapter, garage, prediction,
running, and debrief panels use vertical flow or scrolling; configuration choices remain readable at the minimum
1100×720 window and at the reference 1440×900 capture size. English and Simplified Chinese screenshots must show no
clipping or overlap.

Persistent screen space is reserved for the current goal, active safety state, and decision evidence. Repeated
controls, component families, map layers, and diagnostic categories use a small shared symbol and color vocabulary.
Hover hints expose the full localized name, units, and consequence; the compact label still communicates the action
for screenshots, keyboard navigation, and users who never hover. A tooltip cannot be the only place that contains a
safety-critical value or the current autonomous action.

The AI view applies that vocabulary as a compact legend: cyan identifies the global route and LiDAR evidence,
neutral white records actual motion, violet shows local-planner samples, amber marks the selected local trajectory,
and red identifies predicted conflict. Hovering a legend row explains the producer and semantic meaning; hovering a
terrain-layer chip explains the active measurement. The current decision and safety clearance remain visible.

## Completed alpha baseline

The following stages record work already completed for the original `lost_signal` vertical slice. They are retained
as release evidence and are not the queue for upcoming work.

### Alpha Stage 1 — Design and implement the flagship encounter

Status: **complete**

`lost_signal` is the content target. Its learning question is:

> Why must an autonomous robot predict future motion instead of reacting only to an obstacle's current distance?

The mission uses Predict–Observe–Explain as its learning rhythm. The player commits to an idea before launch, watches a physical consequence, and then tests the explanation against recorded evidence. This follows the exploration and discovery principles documented by [PhET research](https://phet.colorado.edu/en/research) and the low-barrier model inspection described in [Explorable Explanations](https://worrydream.com/ExplorableExplanations/).

### Player contract

The player chooses the sensing/computing setup and driving policy, answers one concrete prediction question, and dispatches RO-1. RO-1 owns route selection, sensing, motion prediction, waiting, crossing, steering, braking, and actuator output. The authored mission never asks the player to steer around the difficult part.

The garage should emphasize variables that can explain this encounter. Unrelated component choices remain in the build but do not compete for attention on the flagship preparation screen.

### Three-beat experience

| Beat | Simulation trigger | What the player sees | Learning purpose |
|---|---|---|---|
| Carry the spark | Mission start through power-module pickup | A dark signal tower establishes the destination. RO-1 routes around the storage racks and retrieves a visible power module. | Establish intention, route, and the cause of the coming world change. |
| The blind crossing | Power pickup summons depot traffic; the planned route approaches the conflict region | The depot wakes up, a forklift emerges from behind an occluding rack, and RO-1 derives a safe stop pose from current route and vehicle state, predicts, brakes, and waits when needed. | Turn an apparent hesitation into a readable safety decision. |
| Send the signal | The predicted conflict clears and RO-1 reaches the tower | RO-1 commits to the crossing, docks at the relay, and the tower changes from dark to a valley-wide pulse. | Pay off the prediction with a visible outcome and a concise evidence chain. |

The route and timing come from the simulation. Presentation may amplify a real state transition, but it may not invent traffic, detection, clearance, or success.

### Information rhythm

The field view must answer “what is happening?” before a panel is opened. It uses short, event-driven cues:

1. Power module collected; depot traffic is now active.
2. Moving object observed; motion is being estimated.
3. Predicted path conflict; holding outside the crossing.
4. Conflict cleared; crossing committed.
5. Relay powered; signal restored.

AI View answers “what evidence caused it?” with the same event IDs and measurements. The debrief answers “was the prediction supported?” and offers one relevant variable for a retry. Repeated frame-by-frame chatter is not content.

### Failure and retry design

- A dynamic collision rewinds to the earliest useful observation, then shows the prediction and rejected or unsafe action.
- A timeout identifies the longest constrained interval and whether sensing, policy, or route progress caused it.
- A static collision identifies the occupied geometry and commanded local motion.
- A successful run still exposes the wait duration, closest predicted gap, and crossing decision so safety does not look like luck.
- Retry keeps mission and seed fixed and encourages one change among sensor rate/range, prediction horizon/computer, or driving policy.

### Stage 1 acceptance gate

- The preparation screen asks one question and foregrounds only relevant choices.
- The three beats are distinguishable without AI View or documentation.
- All player-facing English is concise and natural; Simplified Chinese covers the same keys.
- The recommended build completes deterministically with zero collisions.
- At least one controlled alternative produces a different, explainable observation or decision without relying on random timing.
- Success, dynamic collision, static collision, and timeout have specific debrief paths.
- Content, locale, headless, and mission regression checks pass.

The original clearance comparison was invalidated by the corrected nominal/conservative prediction split. The
current deterministic MuJoCo evidence keeps both builds safe: the performance computer preserves a larger dynamic
clearance, while the otherwise identical light-computer build finishes slightly sooner and consumes less energy.
This is now taught as a clearance/compute/energy tradeoff. A future fault-injection experiment will provide the unsafe
counterexample without depending on a known collision in the normal controller.

### Alpha Stage 2 — Make decision evidence a shared runtime contract

Status: **complete**

Introduce a compact, structured decision record produced by autonomy and mission systems rather than inferred by UI code. A record contains:

- simulation and observation timestamps;
- observation and tracked-object references;
- prediction horizon and measured/predicted separation;
- active constraint and stable reason code;
- selected action and relevant target values;
- outcome or superseding decision.

The field cue, AI View, debrief, headless JSON, and tests consume this same record. Localized prose maps from stable reason codes; algorithms do not emit presentation sentences. Replay seeking must reproduce the record visible at that simulation time.

The gate is a deterministic `lost_signal` run whose observation → prediction → constraint → action → outcome chain is identical in the graphical client, headless result, and regression assertions.

The implemented contract records compact decision transitions rather than every controller tick. The
recommended seed-47 run currently produces 72 records for 892 local-planning cycles. Replay snapshots carry
the record active at that simulation time; headless JSON emits the closed transition chain; live field copy,
AI View, debrief, and regression tests read the same fields and localize only stable reason/action codes.

### Alpha Stage 3 — Build the state-driven audiovisual vertical slice

Status: **complete**

Improve the research-base scene after the encounter and evidence timing are stable:

- create distinct silhouettes for the power cradle, conflict region, occluding cargo rack, and signal tower;
- improve RO-1 and forklift materials, readable fronts/rears, wheels, mast, work lights, and warning beacon;
- use worn metal, safety paint, damp concrete, screen emission, and controlled color contrast to separate navigable space from industrial clutter;
- bind tower power-up, forklift beacon, RO-1 waiting lights, tire marks, dust, and water effects to real snapshot state;
- add a small original sound set for power pickup, moving-vehicle warning, wait/clear transitions, collision, and tower restoration;
- keep meshes low-poly, particle pools bounded, audio optional, and all graphics absent from headless loading.

The gate is a deterministic screenshot sequence at the preparation, conflict, wait, crossing, and success moments, plus a graphical smoke test with no missing assets or shader warnings. Each visible or audible event must name the state that drives it.

The vertical slice now keeps its encounter landmarks physically present while their state changes. The power
module disappears from its cradle after `goals_reached` advances; the conflict overlay reads the live autonomy
phase; and the signal tower remains dark until `MissionStatus::Succeeded`,
then emits a cyan pulse and expanding signal rings. The forklift beacon reads actor velocity, detection, and
`waiting_for_clearance`; RO-1 lamps read that same wait state; tire marks and bounded dust, spray, and skid smoke
read replayed motion, surface, and traction state. The graphical-only audio director consumes `MissionEvent` and
`DecisionEvidence` transitions for pickup, traffic warning, wait, clear, collision, and restoration cues. `--mute`,
demo, screenshots, and every headless target avoid audio initialization.

The local gate passed with byte-identical 1440×900 images from two runs at 2.0, 23.7, 25.0, 26.5, and terminal
simulation time. The generated WAV set also passes byte-for-byte `--check` reproduction.

### Alpha Stage 4 — Add a minimum reproducible run artifact

Status: **complete**

Write one versioned artifact for a completed run. It records source revision, build identity, platform/backend, content hash, mission and condition, seed, hardware selection, algorithm IDs, configuration identity, result metrics, structured decisions, and causal events. Large sensor traces remain external and are referenced by identity when they exist.

Add a small comparison command that rejects incompatible comparisons and reports declared metric differences for like-for-like runs. Interactive UI replay, LSRI observation replay, and the run artifact remain separate formats with separate purposes.

The gate is reproduction of the recommended `lost_signal` result from its manifest plus a clear diff against one controlled alternative.

Run Artifact v3 now records source/build/platform identity, a SHA-256 JSON-content manifest, authored condition,
seed, runtime, physics engine and version, hardware, algorithm IDs, a canonical configuration digest, metrics, events,
the shared decision chain, and constructed-world plan/contact/traversal evidence. `--reproduce` rejects incompatible content and restores the declared run.
`tools/compare_runs.py` verifies both digests, forbids invariant changes, and requires each controlled variable to
be named. A manifest reproduction matched every reported metric. Changing only
`hardware.components.computer` remains the declared controlled variable for the comparison artifact. Exact metrics
are regenerated at release time because corrected autonomy contracts intentionally invalidate old baselines; the
comparison must report the current time, energy, and clearance values rather than preserve obsolete score claims.

### Alpha Stage 5 — Release closure

Status: **complete**

- finish English game copy and Simplified Chinese game localization;
- keep source, tests, headless output, and developer documentation in English;
- update learning, architecture, validation, asset-provenance, and sim-to-real documents to match shipped behavior;
- remove temporary screenshots, generated diagnostics, stale references, and unneeded workspace files;
- verify licenses and provenance for every shipped asset;
- run content and locale validation, the full headless suite, graphical smoke tests, and deterministic screenshots;
- run sanitizer checks for milestone and release-candidate tags;
- publish only after Linux, macOS, Windows, sanitizer, and graphical CI jobs pass.

The release claim remains deliberately narrow: Lost Signal demonstrates deterministic simulation, inspectable autonomy, reusable runtime semantics, and a path toward hardware integration. Real-world transfer requires calibrated hardware trials and recorded evidence.

The current Linux x86_64 Release build passes 90/90 tests, content and locale validation, canonical content
roundtrip, the five-chassis MuJoCo proving matrix, and a byte-identical Bridge screenshot repeat under Xvfb. The
most recent sanitizer baseline predates this Bridge increment; ASan/UBSan remains the final frozen-milestone gate as
planned. Portable Linux, macOS, and Windows CI remain publishing gates; local Linux evidence does not replace those
hosted jobs.

## RO-1 post-baseline optimization backlog

RO-1 now closes the reference control chain from body intent through deployable wheel PID and LSRI normalized-duty
commands to the simulated motor, articulated wheels, tire contact, rigid body, and measured joint feedback. The
items below strengthen timing, fault evidence, and physical fidelity; they do not reopen that ownership boundary.

### Release-candidate gates

1. **Sensor delivery time and fusion alignment.** Keep acquisition time immutable, delay simulated delivery by the
 configured device latency, and align encoder, IMU, pose, and LiDAR updates by sample time. Tests must cover delayed,
 jittered, stale, dropped, and out-of-order packets without using arrival time as the measurement timestamp. The
 current LSRI header carries latency, but shipped component profiles use zero delivery delay and therefore do not yet
 exercise this path end to end.
2. **Integrated command-loss safety.** Exercise the complete RO-1 loop with skipped and expired actuator commands,
 verify LSRI emits both channels' declared safe values, and measure stopping distance and residual wheel effort. The
 existing channel-level watchdog test remains necessary but is not the full plant-level proof.
3. **Long-run numerical stability.** Add bounded energy, pose, contact penetration, wheel-speed, and temperature-free
 motor-state drift checks over longer stationary and constant-speed runs. Compare at least two internal contact step
 sizes with tolerance envelopes rather than requiring bit identity between different solvers or platforms.
4. **Final release execution.** Run ASan/UBSan after the functional work is frozen, then graphical smoke,
 deterministic screenshot, packaging, and Linux/macOS/Windows CI gates. Record the exact commit, compiler, asset
 hashes, scenario, seed, configuration, and result artifacts.

### Optional higher-fidelity profiles

5. **Electrical and drive effects.** Add named profiles for duty deadband, supply-voltage sag, current/torque limits,
 regenerative or resistive braking, gearbox drag/backlash, command delay, and thermal derating. Each effect remains
 independently switchable and requires a measurable maneuver; it must not become an unexplained default constant.
6. **LiDAR fidelity families.** Extend the existing planar first-return profile with optional beam divergence,
 weather attenuation, multi-return behavior, packet timing, and later multi-line spinning and solid-state patterns.
 All variants publish the same LSRI time/frame/source contract and keep device-specific fields explicit.
7. **Payload mechanics.** Add payload collision geometry, attachment compliance, and load placement instead of
 changing only mass, centre height, and yaw inertia. Re-run static loads, braking, tip, slope, and rough-ground gates.
8. **Identified or HIL evidence.** A contributed physical target must record calibration sources and pass held-out
 wheel-speed, yaw, pose, stopping-distance, scan timing, range, and intensity error envelopes. Until then the project
 may state that its boundary supports transfer experiments, but must not claim proven sim-to-real performance.

RO-2 through RO-5 now have independent geometry, actuator, contact, and maneuver acceptance gates. Their typical
parameters remain engineering assumptions, and their completion does not weaken or silently generalize RO-1
evidence. Sensor evidence remains attached to the selected sensor profile rather than duplicated per chassis.

### Selected implementation sequence: 16-line LiDAR, then RO-2

Both expansions are approved, with this fixed order:

1. **16-line spinning LiDAR on RO-1.** First make configured sensor latency affect deterministic delivery while
 preserving acquisition timestamps. Then add data-authored vertical beam angles, ring and azimuth timing, 3D MuJoCo
 raycasts, LSRI point-cloud transport, ground/non-ground projection into the existing local obstacle layer, and a
 height/ring/intensity AI visualization. Analytic fixtures must cover flat ground, a vertical wall, a slope, a low
 obstacle, an overhanging obstacle, occlusion, intensity, ring identity, and motion-distorted per-point timing.
2. **RO-2 articulated Ackermann reference.** Replace the ideal single-body proxy with authored chassis, steering
 knuckles, four rolling wheels, explicit drive and steering actuator channels, physical joint feedback, tire/contact
 response, and deployable controllers across LSRI. The acceptance matrix must separately cover static wheel loads,
 straight/reverse motion, steering step/rate, common instantaneous turn centre, minimum-radius circles, braking,
 low-friction understeer/slip, cross-slope stability, command expiry, and long-run rest stability.

The 16-line profile must not be implemented as sixteen independent planar scans, and RO-2 must not be promoted by
retaining an analytic yaw-torque shortcut. Both features stay data-authored and use the same ROBOT/LSRI/SIM boundary
as RO-1. After their focused gates pass, existing missions are re-run; mission success itself is not the physical
oracle.

The 16-line implementation is functionally complete: schema 3 point clouds, per-beam authored calibration,
three-dimensional firing-time raycasts, estimated-pose static reference, deterministic delayed delivery,
ground/non-ground projection, bounded 3D presentation, shared replay storage, focused geometry tests, and a repeatable
headless performance benchmark are implemented. A dedicated authored learning encounter and broader campaign
playtest remain content work.

The RO-2 articulated physical gate is also complete at the documented `typical` level. The data-authored assembly,
bounded front knuckles, four rolling wheels, rear drive, four-channel LSRI duty path, deployable Ackermann controller,
physical feedback, and MuJoCo contact run without an analytic yaw-torque shortcut. Independent tests cover the matrix
above plus motor-limited gradeability, zero-command rollback, public-step convergence, and 25 mm rough terrain.
Mechanical tie rods, identified parameters, HIL, and real-car evidence remain optional later fidelity work rather
than implied results of this gate.

