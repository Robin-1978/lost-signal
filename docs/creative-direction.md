# Creative direction

## Product promise

**Give a robot an intention, watch a real autonomy stack make decisions, and discover why the result changed.**

Lost Signal should feel like an adventure first and a laboratory second. The player is a field engineer in Lan Valley, where infrastructure has failed and a weak transmission hints that someone or something is still online. RO-1 is a companion with personality, physical limitations, and its own driving system. The relationship works because the player decides *what matters* while the robot decides *how to move*.

The one-sentence public description is:

> An open-source robotics learning adventure that turns real autonomous-driving algorithms into observable, testable, reproducible experiments.

## Audience

The primary audience is curious teenagers and adults who enjoy systems, robots, engineering, or automation but do not yet know the robotics vocabulary. Secondary audiences include robotics clubs, introductory courses, museums, makers, and developers who want a compact reference simulation.

The game must explain itself through consequences. A player should be able to enjoy a mission without reading a textbook, then open the AI view when curiosity appears.

## The fantasy

Lan Valley once depended on autonomous machines. A communications failure left roads, depots, relays, and service robots in isolated pockets. The player and RO-1 travel outward from a repair bay, recover capabilities, respond to local work, and follow fragments of the lost signal.

The fantasy has three emotional beats:

1. **Companionship:** RO-1 observes, hesitates, commits, and reports what it believes.
2. **Discovery:** every recovered route reveals a place, a person, a machine, or a new piece of the signal.
3. **Mastery:** confusing behavior becomes understandable when the player learns to read evidence.

## Product pillars

### RO-1 drives itself

After the opening calibration, direct steering is unavailable. The player moves the engineer and issues high-level commands. This keeps the central promise honest: every path, stop, detour, and recovery comes from autonomy code and simulation state.

### Decisions are visible without becoming a dashboard wall

The field view tells the immediate story through motion, sound, lighting, dust, spray, body roll, brake distance, and RO-1's short callouts. The AI view answers deeper questions with layers for sensor returns, estimated pose, global path, local candidates, risk, and actuator output. Information appears when it helps a decision.

### Every mission is a compact experiment

A mission introduces one dominant variable, asks for a prediction, creates a visible consequence, and ends with a debrief. The player can retry with one changed component or preference. The best learning moment is often a failed prediction that the evidence can explain.

### The valley feels alive

Forklifts, pedestrians, weather, lighting, service calls, and changing routes give the autonomy stack reasons to react. These elements must participate in sensing, planning, or physics. Decoration supports mood but should never pretend to influence the robot when it does not.

### Engineering remains truthful

The game may simplify scale and pacing, but it must label simplifications and preserve cause and effect. Hidden truth cannot leak into perception algorithms. Sim-to-real claims require logs and measurements from hardware.

## Core experience

The player repeats a four-part loop:

1. **Prepare:** inspect the place, choose a chassis/build, and predict the risk.
2. **Dispatch:** give RO-1 an intention and travel beside it as the field engineer.
3. **Observe:** notice physical behavior; open the AI view when the outcome is surprising.
4. **Debrief:** connect observation, judgment, action, and result, then retry or continue.

A main mission should last 4–10 minutes. A short contract should last 2–6 minutes. Expedition mode has no forced ending and creates optional events around the player and RO-1.

## Content structure

- **Eight main missions** form the authored learning and story arc.
- **Town contracts** recombine known ideas under different safety, time, and energy goals.
- **Engineering experiments** isolate chassis, tire, motor, sensing-rate, terrain, and policy effects.
- **Lan Valley Expedition** lets the player walk with RO-1 and issue high-level intentions in a persistent world.

The campaign unlocks concepts gradually. The garage should present a meaningful choice, not a catalog of unexplained parts. A new component appears when a mission can demonstrate its consequence.

## Tone and presentation

The world is worn, hopeful, and readable. Warm service lights, cool valley haze, painted safety markings, repair patches, and compact industrial forms create a place worth restoring. RO-1 should look capable and approachable, with motion and light that communicate state before text does.

Writing is concise and specific. RO-1 speaks about evidence and uncertainty in ordinary language: “The wheels say I moved farther than the wall markings do.” Technical names appear after the player has seen the effect.

The interface defaults to English and supports Simplified Chinese. Layouts must tolerate translated text and should avoid baking words into textures or models.

## Inspiration and research

The direction borrows proven principles rather than copying a surface style:

- [Kerbal Space Program](https://www.kerbalspaceprogram.com/) makes physical systems memorable through prediction, failure, and retry.
- [Human Resource Machine](https://tomorrowcorporation.com/humanresourcemachine) turns program execution into visible, stepwise action.
- [Autonauts](https://www.denki.co.uk/autonauts/) makes high-level automation feel like companionship and delegation.
- [CARLA](https://carla.org/) demonstrates the value of an open simulator with configurable sensors and reproducible autonomy experiments.
- [Webots](https://cyberbotics.com/) shows how one environment can serve education, research, and robot-controller development.

Lost Signal's distinct contribution is the combination of an authored companion adventure, inspectable autonomy, chassis-level physical consequences, and a deterministic open-source experiment path.

## Release priorities

For the first public alpha, quality comes from a coherent vertical slice rather than feature count:

1. Make the first 20 minutes attractive, legible, and free of control confusion.
2. Polish English player text and preserve complete Simplified Chinese support.
3. Keep all eight main missions deterministic and explainable.
4. Make headless build and validation work on Linux, macOS, and Windows.
5. Publish honest architecture and sim-to-real boundaries.
6. Invite learning feedback with a focused issue template.

More characters, a broader soundtrack, broader accessibility, controller support, packaged binaries, and real-hardware adapters can follow without weakening the first release.

## After the public alpha

The next milestone should deepen one complete learning encounter rather than broaden the technology list.

### 1. An explainable flagship encounter

Polish `lost_signal` as a short three-beat experience: approach an occluded work area, observe and yield to moving traffic, then restore the signal tower. Give the depot its own readable landmarks, environmental motion, state lighting, sound cues, and payoff. Every cue must come from simulation state.

The field view should make RO-1's situation legible before the player opens a panel. The AI view and debrief should then show one structured evidence chain from observation, through prediction and constraint, to action and outcome. The same reason codes and measurements must feed player text, headless output, and regression tests; the UI should not infer a different explanation from a few final-state flags.

### 2. A small experiment record

Before adding a general experiment framework, define one versioned run artifact containing the source revision, content hash, backend and platform, mission and condition IDs, seed, selected hardware, algorithm IDs, configuration identity, metrics, and causal events. Keep the interactive timeline, LSRI observation replay, and a reproducible run artifact as separate concepts.

A small comparison tool may then evaluate like-for-like alternatives with declared tolerances. Global planners compare with global planners, local planners with local planners, and different chassis retain their own feasible-motion constraints.

### 3. Reusable conditions and deterministic faults

When at least two lessons need the same objective under different conditions, introduce a lightweight scenario overlay for actors, surfaces, visibility, and fault profiles. Do not build a general scenario language first. Fault injection belongs at the platform/runtime boundary so delay, loss, reordering, stale timestamps, actuator lag, and saturation pass through the same LSRI validation and remain seeded and reproducible.

### 4. Admit new technology only through a learning question

- Add a sensor only when it creates a distinct observation problem that the player can predict, see, and explain.
- Use the existing observed-grid work for a map-building lesson before attempting full SLAM. Add SLAM only when an unknown-map mission needs pose uncertainty, map uncertainty, relocalization, and loop closure.
- Treat a local language model as an optional evidence-grounded tutor or high-level policy experiment. The deterministic campaign must run without it. Model output must never bypass the existing policy safety shield, planner, controller, or command watchdog. A text model endpoint is not a VLA stack; a future VLA experiment also needs calibrated, timestamped visual observations, a bounded high-level action vocabulary, recorded evaluation cases, and explicit latency and failure handling.
- Add multi-process transport, HIL, or a native robot driver when a concrete deployment requires it; LSRI semantics remain the stable boundary.
