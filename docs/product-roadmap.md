# Product roadmap

This roadmap turns the current simulation into a polished, open-source autonomy learning experience. Work proceeds in the order below. A later stage starts only after the preceding stage has a reviewable result and passes its acceptance gate.

## Product outcome

The first public release should let a new player give RO-1 a high-level intention, watch it handle a dynamic encounter autonomously, understand why it waited or moved, change one relevant variable, and reproduce the result. The experience must remain useful at three depths:

- **Explore:** the field scene communicates the immediate situation through motion, landmarks, light, and sound.
- **Learn:** AI View connects observations, predictions, constraints, and actions.
- **Verify:** headless output and a run artifact make the same encounter reproducible and comparable.

The campaign remains deterministic and fully usable without a language model. New sensors, general scenario overlays, full SLAM, VLA, multi-process transport, HIL, and native robot drivers remain outside this release path until a concrete learning or deployment case requires them.

## Stage 1 — Design and implement the flagship encounter

Status: **in progress**

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

## Stage 2 — Make decision evidence a shared runtime contract

Status: **pending Stage 1**

Introduce a compact, structured decision record produced by autonomy and mission systems rather than inferred by UI code. A record contains:

- simulation and observation timestamps;
- observation and tracked-object references;
- prediction horizon and measured/predicted separation;
- active constraint and stable reason code;
- selected action and relevant target values;
- outcome or superseding decision.

The field cue, AI View, debrief, headless JSON, and tests consume this same record. Localized prose maps from stable reason codes; algorithms do not emit presentation sentences. Replay seeking must reproduce the record visible at that simulation time.

The gate is a deterministic `lost_signal` run whose observation → prediction → constraint → action → outcome chain is identical in the graphical client, headless result, and regression assertions.

## Stage 3 — Build the state-driven audiovisual vertical slice

Status: **pending Stage 2**

Improve the research-base scene after the encounter and evidence timing are stable:

- create distinct silhouettes for the power cradle, inspection pocket, occluding cargo rack, and signal tower;
- improve RO-1 and forklift materials, readable fronts/rears, wheels, mast, work lights, and warning beacon;
- use worn metal, safety paint, damp concrete, screen emission, and controlled color contrast to separate navigable space from industrial clutter;
- bind tower power-up, forklift beacon, RO-1 waiting lights, tire marks, dust, and water effects to real snapshot state;
- add a small original sound set for power pickup, moving-vehicle warning, wait/clear transitions, collision, and tower restoration;
- keep meshes low-poly, particle pools bounded, audio optional, and all graphics absent from headless loading.

The gate is a deterministic screenshot sequence at the preparation, conflict, wait, crossing, and success moments, plus a graphical smoke test with no missing assets or shader warnings. Each visible or audible event must name the state that drives it.

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

