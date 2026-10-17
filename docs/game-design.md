# Game design

## Player role and control boundary

The player is a field engineer, not a remote-control driver. The opening chassis-calibration mission briefly maps keys to the robot so acceleration, steering, and braking can be felt. After that lesson, RO-1 is autonomous.

In authored missions the player selects hardware, chooses a driving preference, makes a prediction, and dispatches the robot. In expedition mode the player walks around the site and asks RO-1 to follow, go to a point, search, wait, or return. The autonomy stack owns the route, collision avoidance, speed, steering, and actuator output.

Vehicle Proving Ground is an optional engineering workspace outside campaign progression. It exposes every authored chassis and hardware component on a 48 × 32 m data-driven map with wet, sand, mud, gravel, steel, ramp, cross-slope, and rough-road zones. `operator_drive` starts under direct human authority; clicking the world plans through the normal autonomy stack, and any non-neutral driving input takes authority back. Switching control never bypasses LSRI, platform dynamics, perception, planning, or actuator feedback.

## Three nested loops

### Moment to moment: read the situation

The player watches RO-1 approach terrain and moving actors, hears a short callout, and decides whether to let the plan continue or issue a new intention. A surprising stop or detour invites the AI view.

### Mission: predict, run, debrief

A mission begins in the garage with a constrained decision. The prediction screen asks a concrete question. During the run, the world shows the physical consequence. The debrief awards safety, speed, and efficiency separately and presents a causal sequence.

### Valley: restore capability

Main missions recover routes, tools, and knowledge. Contracts reuse the systems under new constraints. Expedition events make restored areas useful. Story fragments pull the player toward the lost signal.

## The eight main missions

| Mission | Player-visible problem | Robotics idea |
|---|---|---|
| Wake Up | RO-1 coasts after the key is released | acceleration, inertia, braking |
| Change Shoes | two tires stop differently | friction and traction |
| First Sight | RO-1 must stop before an unseen obstacle | range sensing and stopping distance |
| Ramp Assembly | a loaded robot must start on a slope | grade force, traction, and motor authority |
| Where Am I? | wheel motion and world motion disagree | encoder drift and sensor fusion |
| The Blocked Gate | the known route is no longer usable | maps and global replanning |
| Cross-Slope Delivery | a high payload makes a side slope dangerous | center of mass and rollover margin |
| Send the Signal | forklifts cross the final route | dynamic perception, prediction, and safe entry |

The mission title translations are player-facing and can evolve, while mission IDs remain stable for saves and tools.

## Failure design

A failure should answer three questions:

1. What was the earliest relevant observation?
2. What did the robot infer and choose?
3. Which physical or logical constraint produced the result?

Collision, timeout, localization error, traction loss, and rollover risk need distinct explanations. “Try again” is only useful when the player knows what variable to change.

The debrief uses a causal timeline built from runtime telemetry. It does not invent an explanation from the final score.

## Chassis as gameplay

Each chassis changes feasible motion and failure modes:

| Chassis | Strength | Cost the player must manage |
|---|---|---|
| Differential | simple, compact, predictable | cannot move sideways; turning needs clearance |
| Tracked | strong terrain contact | scrub during turns distorts odometry and consumes energy |
| Ackermann | efficient forward travel | minimum turning radius and steering-rate limits |
| Swerve | independent wheel steering and holonomic motion | more complex steering alignment and packaging |
| Mecanum | immediate planar motion | reduced efficiency and greater sensitivity to low traction |

The planner and local controller must respect these constraints. A chassis is not an animation skin.

## Information layers

The field view communicates state through the world: pose, body motion, wheel motion, dust or spray, lights, sound, and short dialogue. The AI view contains four optional terrain/navigation layers plus LiDAR, pose estimates, paths, trajectory candidates, predicted actors, and control telemetry.

The default screen should remain readable to a new player. Advanced evidence appears on demand and uses the same color and naming consistently across missions and documentation.

## Scoring

Safety, time, and energy are independent badges. A safe run can be slow; a fast run can waste energy. Thresholds are content data and are pinned by deterministic recommended-build runs. They should reward a comprehensible tradeoff rather than an accidental seed.

Main-story completion should not require perfect badges. Experiments and contracts can set narrower targets after the underlying concept has been taught.

## Content acceptance criteria

A mission is ready when it has:

- one clear learning question;
- a visible contrast produced by real simulation state;
- an achievable default and recommended configuration;
- deterministic seeds and bounded completion time;
- a prediction prompt with plausible choices;
- runtime evidence for success and major failures;
- English and Simplified Chinese player text;
- schema, content, headless, and relevant simulation tests.
