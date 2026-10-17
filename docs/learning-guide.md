# Learning autonomous robotics with Lost Signal

Lost Signal teaches one connected idea: an autonomous robot acts through a chain of imperfect observations and constrained decisions.

```text
world → sensors → estimation → map and prediction → planning → control → actuators → world
```

A surprising result can originate anywhere in that chain. The learning goal is to locate the earliest meaningful cause rather than blame “the AI.”

## The eight-mission path

### 1. Wake Up — motion and stopping

Drive RO-1 in the calibration bay and release the key before the target. Observe that velocity cannot disappear instantly.

Ask: If the same stop command is sent at twice the speed, how should stopping distance change? Which evidence distinguishes input release from physical deceleration?

### 2. Change Shoes — grip

Run the same route with different tire compounds. Watch braking distance and wheel/body disagreement.

Ask: Does more motor authority always help? Which result changed even though the route did not?

### 3. First Sight — LiDAR and stopping distance

RO-1 drives autonomously toward an obstacle that only appears after LiDAR observes it. In the AI view, connect returns to the local map and braking response.

Ask: How do sensor range, scan rate, vehicle speed, latency, and braking distance combine into a safe detection margin?

### 4. Ramp Assembly — slope launch

A payload changes the force and traction required to start on a ramp. Compare motor, tire, and driving preference.

Ask: Is a failed launch caused by insufficient torque, insufficient traction, or an unsafe controller limit? What measurement separates them?

### 5. Where Am I? — odometry drift

Encoders integrate wheel rotation. Slip and quantization make the estimate drift from world motion. Sensor fusion can correct some error but introduces its own timing and noise.

Ask: Why can the wheels report motion that the robot body did not achieve? When should a correction be trusted?

### 6. The Blocked Gate — global replanning

A known route becomes unavailable. Inspect the occupancy/cost map, the global path, and the local controller.

Ask: Which layer should decide that the old route is impossible? Why should a local obstacle avoider not search the entire map?

### 7. Cross-Slope Delivery — stability

A high payload moves the combined center of mass. Body roll and the projected gravity line reduce the rollover margin.

Ask: Which helps more: slowing down, lowering the payload, or choosing a gentler path? Which factor changes static stability and which changes dynamic load transfer?

### 8. Send the Signal — dynamic interaction

Forklifts cross the route. RO-1 must first observe them, estimate their motion, predict conflict, and wait outside the conflict area before entering.

Ask: Why is distance alone insufficient? What makes waiting a planned maneuver rather than failure to move?

## A five-step experiment method

1. **Question:** write one specific question, such as “Will the 5 Hz LiDAR detect the forklift early enough?”
2. **Prediction:** state the expected result and reason before running.
3. **Control variables:** change one component, surface, policy, or condition; keep mission and seed fixed.
4. **Evidence:** compare event time, pose estimate, clearance, wheel speed, energy, and debrief causes.
5. **Explanation:** describe the observation, decision, action, and outcome in that order.

A class or club can assign pairs: one learner runs the experiment while the other explains the evidence without seeing the chosen configuration.

## Reading the AI view

- **LiDAR returns** are observations, not a complete world map.
- **Estimated pose** is what planning believes; the body pose is simulation truth used for evaluation.
- **Global path** answers which corridor to use.
- **Local candidates** answer which immediately feasible velocity or motion to choose.
- **Predicted actor paths** express short-term estimates and uncertainty.
- **Wheel target and measured RPM** reveal control error.
- **PWM** is controller output, not achieved motion.
- **Terrain layers** separate geometry, slope, vehicle-specific risk, and final planning cost.

## Useful vocabulary

| Term | Meaning in the project |
|---|---|
| Observation | timestamped data delivered by a sensor model |
| Ground truth | hidden physical state used by simulation and evaluation |
| Odometry | locally continuous pose estimated from motion, allowed to drift |
| Localization | correction of pose against an external reference or map |
| Global planner | selects a route across the map |
| Local planner | selects a near-term collision-free, feasible motion |
| Prediction | estimates where a moving actor may be soon |
| Kinematics | maps body motion to wheel and steering motion |
| Control | drives measured actuator state toward a target |
| Clearance | minimum separation from relevant obstacles |
| Traction | available contact force before slip |
| Rollover margin | stability against the gravity/load line leaving the support area |

## From the screen to the source

Start with `assets/levels/` for the mission question and thresholds, then read `assets/presentation/` for its player-facing structure. `src/sim/game/scenario/scenario_session.cpp` composes the loop; extracted systems live in `src/sim/game/mission/systems/`. Global and local planning, localization, mapping, perception, behavior, and control have separate directories under `src/robot/autonomy/`. Deployable platform contracts live under `src/robot/platform/` and `src/robot/runtime/`; virtual chassis, sensors, and physics live under `src/sim/hardware/vehicle/`, `src/sim/hardware/sensors/`, and `src/sim/hardware/physics/`.

Run a deterministic mission without a window when comparing code changes:

```bash
./tools/run.sh --headless --level lost_signal --recommended --json
```

Use the same revision, mission, build, seed, physics backend, and duration when comparing results.
