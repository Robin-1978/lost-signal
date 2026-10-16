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

The first prototype after the release foundation should be **Orchard Scramble**. It reuses existing terrain, chassis, tires, payload, planning, deterministic seeds, and scoring, so it tests the flexibility of the framework without using a new sensor or a large subsystem as a substitute for game design.

Its core loop is:

1. Inspect a seeded orchard with apples distributed across paved rows, mud, side slopes, and a narrow ridge.
2. Choose a chassis/build and a collection priority such as safest fruit, fastest basket, or lowest energy.
3. Dispatch the autonomous robot; it selects the visit order and feasible local motion.
4. Watch the carried mass and center of gravity change after each pickup.
5. Debrief fruit integrity, apples collected, route time, energy, slip, and rollover margin.
6. Retry the same seed with one changed variable or compare another chassis through the run-artifact tooling.

Apples are physical mission items with deterministic pickup radii and payload effects. Their score responds to measured impact, roll, and acceleration; decorative animation does not alter the result. The mode earns inclusion only if at least two builds produce visibly different, explainable strategies on the same seed.

## Stage 1 — Design and implement the flagship encounter

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
| The blind crossing | Power pickup summons depot traffic; RO-1 reaches the inspection pocket | The depot wakes up, a forklift emerges from behind an occluding rack, and RO-1 observes, predicts, brakes outside the conflict area, and waits when needed. | Turn an apparent hesitation into a readable safety decision. |
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

Acceptance evidence on the Bullet backend is deterministic for seed 47. The recommended performance
computer finishes in 89.18 s with zero collisions and 0.984 m dynamic clearance. The otherwise identical
light-computer build finishes in 86.18 s with zero collisions but only 0.464 m clearance, below the 0.6 m
safe-score threshold. This gives the player a controlled comparison in which one relevant variable changes
the safety margin without turning the lesson into an arbitrary failure.

## Stage 2 — Make decision evidence a shared runtime contract

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

## Stage 3 — Build the state-driven audiovisual vertical slice

Status: **complete**

Improve the research-base scene after the encounter and evidence timing are stable:

- create distinct silhouettes for the power cradle, inspection pocket, occluding cargo rack, and signal tower;
- improve RO-1 and forklift materials, readable fronts/rears, wheels, mast, work lights, and warning beacon;
- use worn metal, safety paint, damp concrete, screen emission, and controlled color contrast to separate navigable space from industrial clutter;
- bind tower power-up, forklift beacon, RO-1 waiting lights, tire marks, dust, and water effects to real snapshot state;
- add a small original sound set for power pickup, moving-vehicle warning, wait/clear transitions, collision, and tower restoration;
- keep meshes low-poly, particle pools bounded, audio optional, and all graphics absent from headless loading.

The gate is a deterministic screenshot sequence at the preparation, conflict, wait, crossing, and success moments, plus a graphical smoke test with no missing assets or shader warnings. Each visible or audible event must name the state that drives it.

The vertical slice now keeps its encounter landmarks physically present while their state changes. The power
module disappears from its cradle after `goals_reached` advances; the observation pocket changes from active to
complete from that same ordered-goal state; and the signal tower remains dark until `MissionStatus::Succeeded`,
then emits a cyan pulse and expanding signal rings. The forklift beacon reads actor velocity, detection, and
`waiting_for_clearance`; RO-1 lamps read that same wait state; tire marks and bounded dust, spray, and skid smoke
read replayed motion, surface, and traction state. The graphical-only audio director consumes `MissionEvent` and
`DecisionEvidence` transitions for pickup, traffic warning, wait, clear, collision, and restoration cues. `--mute`,
demo, screenshots, and every headless target avoid audio initialization.

The local gate passed with byte-identical 1440×900 images from two runs at 2.0, 23.7, 25.0, 26.5, and terminal
simulation time. The generated WAV set also passes byte-for-byte `--check` reproduction.

## Stage 4 — Add a minimum reproducible run artifact

Status: **pending Stage 3**

Write one versioned artifact for a completed run. It records source revision, build identity, platform/backend, content hash, mission and condition, seed, hardware selection, algorithm IDs, configuration identity, result metrics, structured decisions, and causal events. Large sensor traces remain external and are referenced by identity when they exist.

Add a small comparison command that rejects incompatible comparisons and reports declared metric differences for like-for-like runs. Interactive UI replay, LSRI observation replay, and the run artifact remain separate formats with separate purposes.

The gate is reproduction of the recommended `lost_signal` result from its manifest plus a clear diff against one controlled alternative.

## Stage 5 — Release closure

Status: **pending Stage 4**

- finish English game copy and Simplified Chinese game localization;
- keep source, tests, headless output, and developer documentation in English;
- update learning, architecture, validation, asset-provenance, and sim-to-real documents to match shipped behavior;
- remove temporary screenshots, generated diagnostics, stale references, and unneeded workspace files;
- verify licenses and provenance for every shipped asset;
- run content and locale validation, the full headless suite, sanitizer checks, graphical smoke tests, and deterministic screenshots;
- publish only after Linux, macOS, Windows, sanitizer, and graphical CI jobs pass.

The release claim remains deliberately narrow: Lost Signal demonstrates deterministic simulation, inspectable autonomy, reusable runtime semantics, and a path toward hardware integration. Real-world transfer requires calibrated hardware trials and recorded evidence.

