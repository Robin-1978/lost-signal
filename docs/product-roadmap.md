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

## Next execution milestones

The numbered stages later in this document describe the completed alpha baseline. All new work follows the milestone
sequence here so that completed release work and future work cannot be confused.

The proving ground, service brake, 2.5D editor output, 16-line LiDAR, and five articulated platform gates are complete.
The active sequence is now **MuJoCo atomic migration → mobile-manipulator foundation → manipulation laboratory → grasp/contact validation → focused visual/audio revision →
experiment/fault framework**. Its world commit, playable proof run, first visual pass, and deterministic evidence are
implemented. [The acceptance contract](reference-platform.md) remains the
source of truth for the limits of the platform claims. Orchard Scramble remains useful engineering evidence, but its
static multi-collection loop is not accepted as the first polished Playground. Responsive-information work continues
only when a concrete usability problem blocks play. Platform fidelity and broader sensors follow after the product
milestones, except for correctness fixes needed by the chosen content.

The active implementation order is:

Current status: stages 1--4 are complete for the published, unloaded RO-1 `typical` envelope. The 16-line LiDAR and
RO-2 articulated `typical` gates are also complete. The independent Vehicle Proving Ground is implemented as an
always-available engineering space with open loadout selection, direct driving, and click-to-drive autonomy. It does
not satisfy the polished-playground gate because it remains an engineering instrument rather than a playful loop.
Its live/debrief evidence, eight data-authored repeatable maneuvers, and headless `--maneuver` selection are complete.
RO-2 now has an independent LSRI service-brake path, and editor saves atomically publish an LSRM package containing
occupancy, exact authored 2.5D elevation (including deterministic rough relief), surface classes, and semantic places.
The first construction foundation is also complete: data-authored pieces and sockets, order-independent canonical
plan hashing, transactional validation, and a unified static-world entity contract. A `world_object` now owns the
prefab transform, collision proxies, contact material, LiDAR reflectivity, MuJoCo bodies, oriented planning
rasterization, and continuous collision geometry. Remaining scene instances are explicitly `visual_only`.
The completion claims exclude the open split-friction, identified combined-slip, dynamic-rollover, systematic
payload-sweep, and hardware-identification envelopes listed in `reference-platform.md`.

1. Complete the unloaded RO-1 reference assembly: articulated drive wheels and caster, physical encoder feedback,
 valid mass/inertia data, motor and reflected joint inertia, inertial and collision origins, and stable contact.
2. Validate its devices and closed loops independently: PWM-to-torque drive, braking/coast/backdrive, encoder,
 IMU frame and specific force, and the reference single-line LiDAR.
3. Treat flat floor, slope, cross-slope, low-friction and loose-ground scenes as fixed engineering fixtures. Compare
 forces, loads, slip, pose, scan returns and failure boundaries with analytic or separately derived expectations.
 Do not tune mission time, spawn, score, route or expected success to make an incomplete model pass.
4. After those gates pass, re-run the existing RO-1 missions and correct the reusable planning, collision, control,
 and goal-region contracts they expose. Only then author or tune player-facing outcomes.
5. Define a composable Scenario/Mechanic data contract and editor-facing algorithm catalogs. Implement exact bounded
 multi-goal A* and make actor/object motion selection fully data-authored.
6. Design and graybox a mechanically distinct Playground with a visible physical event. Test the choice, consequence,
 autonomous response, and same-seed retry before approving its permanent art direction.
7. Refine that approved loop with authored 3D models, materials, animation, particles, lighting, cameras, sound, and a
 concise explainable-autonomy presentation.
8. Generalize the experiment, regression, and fault-injection framework from the finished content.
9. Add further sensors or autonomy only for a named experience
 or measured platform target. Run the sanitizer suite for milestone and release-candidate gates.

The next concrete work packages are:

1. **Implemented:** turn the Vehicle Proving Ground into measurable evidence. Data-authored repeatable maneuvers label live
 and debrief measurements for speed, stopping distance and time, commanded and measured steering, inner/outer
 steering-angle difference, measured turn radius, grade, slip, and active LiDAR profile. Keep simulator truth and
 onboard-observable estimates visibly distinct.
2. **Implemented:** add an optional service-brake actuator for the high-speed RO-2 profile through the existing ROBOT/LSRI/SIM
 boundary. Verify apply/release response, torque sign and saturation, tire-limited stopping, split grip, and grade
 hold independently before using it in a lesson. Parking brake and ABS remain separate capabilities.
3. **Implemented:** extend the map editor to author and export the same LSRM-compatible 2.5D surfaces, height patches, obstacles,
 semantic markers, and test routes consumed by simulation. Validate lossless reload and world/grid transforms.
4. **Implemented:** promote RO-3 tracked, RO-4 swerve, and RO-5 mecanum to independent physical reference
 platforms. Each has its own articulated/support model and acceptance evidence for rest stability, signed commanded
 motion, low grip, grade, cross-slope, rough ground, motor braking, timestep convergence, the LSRI actuator path, and
 chassis-specific invariants. Generic runtime expiry remains covered at the typed LSRI boundary; optional parking
 brakes, ABS, compliant full tracks, and identified tire/roller laws require separate future capabilities. No platform
 inherits another platform's evidence.
5. **Frozen for redesign:** **Bridge the Washout** currently proves atomic world/map/physics construction, but its
   menu-driven repair is not accepted as robot gameplay. Keep it as a regression fixture; do not expand its player
   flow until a real robot manipulation capability exists.
6. **Implemented:** MuJoCo 3.16.0 is the single production engine, with a native compiled model/data lifecycle, 1 ms
 contact substeps, articulated trees, free-joint actor proxies, height fields, and aligned wheel friction. The cutover removed the
 retired engine, backend selector, and planar production world after the five-chassis acceptance matrix passed.
7. **Active:** add a compact mobile manipulator as a general ROBOT/SIM capability. The first release must include a
   data-authored serial chain, explicit joint limits and actuator units, FK/IK, time-parameterized joint targets,
 feedback control through LSRI, MuJoCo articulated dynamics, link transforms, telemetry, and deterministic
   reach/limit/stability acceptance tests. A dedicated Manipulation Lab demonstrates the capability independently
   of any story mission.
8. **Next:** add grasp contacts and object attachment only after the arm can reach and hold poses under gravity.
   Grasp success must come from finger/object contact and force/closure criteria; mission code may not teleport an
   object to the tool frame. After that gate, manipulation can support repair, sorting, recovery, inspection, and
   construction scenarios through data-authored tasks.
9. **Retained regression:** **Bridge the Washout** is a bounded construction puzzle
 followed by an autonomous physical proof run. A Ready-state transaction publishes the validated plan to LSRM,
 MuJoCo support/contact, LiDAR, planning, replay, and rendering. Two valid bridge surfaces create a grip/drag choice;
 an overlong part is rejected without mutating the world. Headless and Run Artifact v3 prove map revision 2,
 physical bridge contact, full traversal, success, and zero collision. The dark storm scene, real guardrails and
 rubble, warning light, material contrast, 16-line point cloud, and bounded plan UI form the first presentation pass.
 A fresh-player test still decides the next content revision; the Vehicle Proving Ground remains an engineering
 sandbox and Orchard remains a systems prototype.
6. Polish the approved loop, then generalize its experiment and fault-injection needs. Full SLAM and VLA remain
 deferred until a named experience provides acceptance criteria.

### M0 — Maintain responsive UI quality (deferred, non-blocking)

The shared legend vocabulary, compact labels, and hover hints remain part of the product contract. Broad screenshot
matrix work is paused while the higher-value gameplay milestones proceed. Fix clipping or overlap when it blocks a
real flow, keep temporary captures out of the repository, and retain the sanitizer suite as a milestone/release
gate because it is intentionally expensive.

**Exit:** no clipped or overlapping localized text, compact labels remain understandable without hover, tooltips
show the full explanation, the normal release checks pass, and no temporary capture is tracked or documented.

### M1 — Create the content direction and build its first graybox

Status: **implementation gate complete — external fresh-player evidence remains**

Start from player experience rather than the current feature list. Define a small content bible for a family of
autonomy playgrounds: what fantasy the player enters, what high-level choices they make, what the robot owns, what
physical event creates surprise, what causal evidence explains it, and what one-variable retry teaches. Develop the
collection, search, herding, construction, rescue, and comparative formats above far enough to distinguish their
loops and required engine capabilities. They may demand new mission structure, interactive objects, task allocation,
terrain semantics, sensors, or algorithms; existing runtime limits do not veto a strong concept.

Keep **Orchard Scramble** as the completed systems prototype and regression case. Build **Bridge the Washout** as the
first candidate for permanent polish, then test whether it is actually playful before committing art. Its detailed
loop, data/runtime boundaries, and acceptance gate are defined in [content direction](content-direction.md).
Implement data-authored construction pieces and sockets, canonical plan hashing, transactional placement, one shared
source for MuJoCo and LSRM geometry, a bounded placement UI, autonomous proof runs, and causal failure evidence.
Build three crossing corridors with at least two viable solution families and meaningful chassis differences. The
player commits a repair plan and high-level restore intent; the robot owns planning, braking, crossing, refusal, and
recovery. Runtime work remains generic, while flood, bridge, repeater, inventory, and scoring semantics stay in data
or simulation-only game systems.

The existing chapters remain the Guided Campaign. All systemic formats live in an optional Autonomy
Playgrounds collection with no campaign prerequisite; the player chooses between them rather than having the new
mode inserted into the current story sequence. The detailed content and runtime contract is in
[content direction](content-direction.md).

The Orchard engineering loop is playable from that separate collection. It includes five autonomous collection
tasks, four small registered task-ordering algorithms selected through data, terrain-aware ordering, dynamic
rigid-body payload updates, cargo integrity, deterministic return reserve, and a four-stage storm timeline. Weather
keyframes interpolate wetness, visibility, daylight, and per-material grip on simulation time. The same state changes
wheel contact, route cost, 3D presentation, replay, and headless evidence; crossing a stage boundary triggers an
autonomous replan. RO-1 uses two driven wheels and one rear caster, all official scenes render at physical scale
1.0, and wheel roll comes from actuator state rather than wall-clock animation.

Its deterministic results remain regression evidence, not gameplay approval. The loop still reads as several
static pickups followed by a return, its physical surprise is weak, and its current art does not establish the
quality bar. M1 therefore remains open until a distinct mechanic and a fresh-player playtest justify the content.

**Exit:** the content bible shows several mechanically distinct experiences and the engine capability each one
would exercise. A fresh player can complete and understand the washout loop; two construction plans or chassis
choices produce visibly different, explainable results on the same seed; construction changes physics and map state
from one authoritative plan; the run is deterministic and testable headlessly. Playtest evidence says the loop is
worth polishing. No permanent art quality is required yet.

### M2 — Give the approved Playground a visual and audio identity (blocked by M1)

Replace graybox presentation with distinctive terrain, landmarks, vehicle details, materials, weather,
bounded particles, pickup and damage feedback, and state-driven sound. Visual and audio events read simulation state
and never manufacture gameplay outcomes. Keep headless execution independent of graphical assets.

The Orchard prototype has a first presentation pass through scene data: varied fruit-bearing tree prefabs, deterministic
scale and yaw variation, a detailed packing shed, storm beacons, state-specific collection/packing prefabs, an
attached cargo crate with healthy/damaged item variants, wet-ground glints, pickup particles, and cargo warnings.
The audio cue catalog maps generic event and decision codes to files and cooldowns; the graphical client no longer
contains orchard-specific sound branches. This remains reusable evidence, but permanent hero assets begin only after
M1 approves a loop and identifies the views and physical events that matter most.

**Exit:** the preparation, route-choice, pickup, risk, and debrief beats are readable without opening AI View; a
deterministic screenshot sequence and graphical smoke test show no missing assets or warnings; every dynamic effect
names the state that drives it.

### M3 — Validate the architecture and generalize an experiment framework

Use the finished orchard slice as evidence to audit planner, route, chassis, actuator, sensor, payload, mission,
scenario, runtime, timing, and data-ownership contracts. Correct boundaries that made the slice difficult, then
generalize only the demonstrated needs into Scenario, Experiment, Baseline, Metric, and Diff concepts. Add
deterministic fault profiles and compare complete trajectories and declared metrics rather than a success boolean.
Keep the in-process fixed-step runtime as the default while preserving provider and message boundaries suitable for
future threads, processes, and native hardware. External middleware compatibility is not a product milestone.

**Exit:** the orchard experiment can reproduce and compare controlled variants without UI-specific logic; contract
tests cover units, frames, timestamps, command semantics, route metadata, dynamic payload, and fault boundaries; a
new planner or chassis can enter through documented contracts without editing mission orchestration.

### M4 — Complete one sourced typical platform-fidelity slice

Use RO-1 and one planar LiDAR to validate sourced sensor profiles, timestamps, intensity and
dropout semantics, actuator modes, command expiry, and provider boundaries described in
[platform fidelity](platform-fidelity.md). Every default records its public specification or engineering assumption,
units, range, and effect. Do not claim calibrated sim-to-real accuracy from simulation tests alone.

**Exit:** the typical profile, limitations, analytic fixtures, deterministic evidence, and parameter provenance are
published. An optional future `identified` profile supplies calibration inputs and held-out hardware errors without
changing the runtime contracts.

### M5 — Expand sensors and autonomy only through authored questions

Add multi-line or solid-state LiDAR, SLAM, VLA, multi-process transport, HIL, or native drivers when
a specific learning or deployment experience requires it. Each addition must first state what new observation,
decision, or validation question it makes possible.

**Exit:** each capability ships with a scenario, measurable acceptance evidence, and a smaller interface than the
external system it may bridge to.

Each milestone passes its exit gate before the next begins. Sensor count and algorithm count are not substitutes for
a refined, explainable experience.

The [verification strategy](verification-strategy.md) is a continuous gate across all milestones. Contract
correctness, mathematical invariants, independent oracles, deterministic system evidence, fault boundaries, and
physical validation are accumulated in that order. An algorithm or interface correction may change an unreleased
scenario outcome; the project keeps the correct model and then updates the lesson or content expectation.

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

## Next implementation order

The RO-1 rigid-body, device, and autonomy reference baseline remains the foundation. The next work does not add
more sensor families, SLAM, VLA, ROS integration, or additional chassis fidelity until the current behavior and
runtime contracts are closed.

1. **Executable motion contract and runtime decomposition.** Make the local planner return the timestamped
 trajectory that control will track, add validation and tracking-envelope feedback, then continue after the
 completed `DynamicTrafficSystem` extraction with sensor scheduling, decision evidence, mission progress, and
 snapshot assembly from `ScenarioSession`. Split deployable operator intent and task lifecycle into a small
 middleware-neutral Task Executive; rename the remaining world, scoring and story owner as the Scenario/Game
 Session so `mission` no longer hides two incompatible responsibilities.
 The first Task Executive slice and the LSRM v1 map-provider/package boundary are implemented. The simulation
 composition root is now physically isolated as `src/sim/game/scenario/ScenarioSession` and `lost_signal_sim_game`; the
 remaining deployable work is a native application/API adapter.

The source and build boundary is now executable rather than descriptive: all deployable code is under
`src/robot`, all virtual devices, plants, worlds, game systems, and presentation are under `src/sim`, and every
dependency points from SIM to ROBOT. `LOST_SIGNAL_BUILD_SIM=OFF` builds and tests the ROBOT stack without configuring
MuJoCo or the client. LSRI now registers provider ownership per device endpoint and carries either body-motion
commands or typed actuator commands with explicit duty, effort, velocity, or position semantics. This permits mixed
native/simulated sensor composition and lets controller placement be selected by platform capability.

The shipped RO-1 MuJoCo scenario now runs the deployable wheel controller through LSRI actuator mode. Joint
position/velocity feedback crosses LSRI measurement schema 2, and normalized left/right duty returns through the
typed command path. SIM retains motor response, joint dynamics, tire contact, and rigid-body integration. A future
real smart drive may declare body-motion authority and own its embedded loops.
2. **Behavior Graph control path.** Add the small data-driven graph schema and reusable navigate, observe, wait,
 cross, collect, recover, and complete nodes. Promote a conflict zone from `shadow` to `control` only after its
 exact executable trajectory passes the gates in [Behavior architecture](behavior-architecture.md).
3. **One polished, playful scenario.** Use the same generic systems to author a chassis/terrain challenge with
 multiple targets, moving world actors, meaningful parameter choices, readable consequences, and a deterministic
 debrief. Existing chapters remain available.
4. **Visual and audio finish for that scenario.** Improve models, materials, wheel animation, lighting, particles,
 camera staging, hints, and legends from snapshot state after the scenario mechanics are stable.
5. **Experiment framework and fault injection.** Compare declared algorithm/configuration variants with versioned
 results, then add delay, loss, stale data, actuator lag, slip, and sensor faults through provider contracts.

Each item must leave deterministic focused tests and the complete mission suite green before the next item starts.

The first slice of item 1 is implemented: all three local-planner families return one typed, timestamped
`ExecutableTrajectory`; mission arbitration publishes its command only after exact static, terrain, prediction-
horizon, and swept dynamic validation; and the extracted `TrajectoryTrackingSystem` reports tracking-envelope
evidence on the next decision tick. The `lost_signal` conflict zone is the first controlled vertical slice: a
filtered dynamic track, conservative crossing assessment, sustained release, latched exit transit, and independent
physical-collision veto now pass the scenario's collision and clearance contract. Other conflict zones remain in
`shadow`. The remaining item-1 work is minimum-risk handling for tracking violations and extraction of sensor
scheduling, decision evidence, mission progress, and snapshot assembly.

