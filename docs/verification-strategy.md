# Verification strategy

## Purpose

Lost Signal treats framework correctness as a chain of evidence. A mission result is useful only when the data
contract, mathematical model, implementation, deterministic system behavior, and claimed physical scope are all
clear. No single test, scenario, or successful run proves the whole stack correct.

This strategy applies to every new sensor, chassis, planner, controller, runtime provider, and authored scenario.
The project is under active development, so an incorrect contract is replaced directly and all in-repository
callers are updated together. Unreleased interfaces do not receive compatibility shims.

## Evidence layers

| Layer | Required evidence | Current examples | Gate for new work |
|---|---|---|---|
| Semantic contract | one unit, frame, timestamp, ownership rule, and field meaning | LSRI command spaces; steering angle remains an angle through the Ackermann stack; canonical `RoutePoint` | reject invalid configuration or data instead of reinterpreting it |
| Mathematical invariants | analytic or geometric properties that must always hold | inverse/forward chassis consistency; common Ackermann turn center; signed bicycle yaw; route gear at cusps | add invariant tests before scenario tuning |
| Independent oracle | a reference that does not copy the production implementation | closed-form geometry, finite differences, dimensional bounds, or an independently written reference model | explain why the oracle is independent in the test |
| Deterministic system evidence | fixed scenario, seed, build, configuration, and input trace | mission regressions, replay checks, Run Artifact comparison, full result metrics | compare the relevant trajectory and evidence fields, not only success |
| Fault and boundary evidence | delayed, missing, stale, saturated, reversed, or limiting inputs | LSRI expiry and sequence rejection; sensor delay/dropout; low grip; steering and speed limits | cover the failure fallback and recovery contract |
| Physical evidence | identified parameters and held-out hardware measurements | currently none | required before any calibrated sim-to-real accuracy claim |

Passing a higher layer never excuses a lower-layer failure. Hardware agreement cannot make an ambiguous command
contract acceptable, and deterministic mission success cannot validate the physics outside the tested envelope.

## Architecture invariants

The following rules are checked in code review and tests:

- A command field has one physical quantity and SI unit. Conversion occurs only at a named adapter.
- Observation sample time is distinct from delivery time; network arrival time never replaces acquisition time.
- Physics is the single writer of world state. Consumers read immutable, timestamped snapshots.
- Global planning produces one canonical sequence of `RoutePoint` values. Each value carries position, nominal body
 heading, and the direction used to arrive at it. The first direction is unspecified; every later direction is
 explicitly forward or reverse.
- Grid A* and Hybrid A* remain different algorithms behind the same output contract. Grid A* emits forward route
 segments and derives geometric tangent headings; its last point inherits the incoming segment heading. Hybrid A*
 preserves the heading and gear from its searched state, including on the exact terminal anchor.
- Lookahead cannot cross a forward/reverse cusp. Ackermann steering obeys
 `yaw_rate = signed_speed * tan(steering_angle) / wheelbase`.
- Unsupported capabilities, malformed routes, stale commands, and invalid units fail explicitly.

## Test design rules

Tests should target the smallest layer that can establish a claim:

1. Use unit and property tests for parsing, units, bounds, state transitions, and mathematical invariants.
2. Use a different derivation for the expected result. Repeating production code inside a test only checks that
 both copies make the same mistake.
3. Use component integration tests for timestamp, frame, queue, lifecycle, controller, and provider boundaries.
4. Use deterministic missions for coupled behavior such as perception-to-braking or terrain-to-control effects.
5. Record scenario, seed, source identity, algorithm identity, configuration identity, and input trace in evidence
 intended for comparison.
6. Keep stochastic effects seeded in deterministic runs and report tolerance-based cross-platform limits where
 bit identity is not a valid claim.

When a corrected model changes an authored outcome, keep the correct model and reassess the lesson. Update the
scenario or its expected outcome only when the new result still demonstrates the stated learning question.

## Planned gates

The verification work proceeds with the product roadmap rather than as a separate late phase:

1. **Current architecture gate:** enforce the typed route contract, Ackermann steering-angle semantics, signed
 reverse behavior, and full headless mission regression.
2. **Orchard prototype gate:** compare at least two chassis or builds on the same seed; validate multi-goal ordering,
 payload changes, terrain choice, and result-artifact identity.
3. **Fault-injection gate:** make delay, drop, stale/out-of-order observations, actuator lag/saturation, and low
 friction declarative and deterministic; assert the intended fallback.
4. **Identified planar-platform gate:** fit one robot and one planar LiDAR from calibration data, then publish held-out
 pose, steering, stopping, timing, range, return-rate, and energy errors.
5. **HIL gate:** compare simulation, replay, and hardware providers through the same LSRI contracts before adding a
 broader sensor family or making a transfer claim.

Multi-line LiDAR, solid-state LiDAR, SLAM, VLA, and multi-process transport enter this sequence only when a concrete
learning or deployment question supplies their acceptance metric.

## Claim boundary

The current suite supports deterministic software behavior within its represented scenarios. It does not establish
real-robot safety, calibrated sensor optics, high-speed vehicle dynamics, or general sim-to-real transfer. Those
claims require the identified and physical gates above, with the target platform and validation envelope named.
