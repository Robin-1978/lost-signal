# Braking architecture

## Decision

Lost Signal separates electric-drive braking, service braking, parking braking, tire contact, and ABS. A stop target
is not a physical brake, and a configured deceleration is not a force applied directly to the body.

```text
ROBOT motion control
 -> drive command + typed brake request
 -> LSRI capability and watchdog
 -> electric drive / service brake / parking brake plants
 -> wheel-joint torques or a bounded lock constraint
 -> tire contact, normal load, slip, and surface friction
 -> rigid-body response
```

The same brake plant attaches to wheel joints by data. Ackermann, differential, tracked, and holonomic chassis do not
receive separate copies of the braking equations.

## Industry basis

- [Project Chrono `ChBrakeSimple`](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_brake_simple.html)
 applies a speed-dependent torque opposing wheel rotation and explicitly states that this simplified ODE model
 cannot represent a sticking brake.
- [Project Chrono `ChBrakeShafts`](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_brake_shafts.html)
 represents a brake as a clutch between shafts, accepts a normalized input, exposes maximum torque, and can lock.
- [CARLA's vehicle API](https://carla.readthedocs.io/en/latest/python_api/) separates `brake` from `hand_brake` and
 gives each wheel independent maximum brake and handbrake torque.
- [PhysX Vehicles](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html) maps normalized brake
 commands to a maximum torque and per-wheel response multipliers. Wheel inertia controls how quickly a wheel locks;
 tire force and slip determine the vehicle response. It uses a separate low-speed sticky state because continuous
 tire-force models can oscillate near rest.

These are semantic references, not validation evidence for Lost Signal parameters.

## Model levels

### Electric drive

The motor plant owns motoring, back-EMF, current limits, and regenerative or dynamic braking. The present compact
four-quadrant teaching model reduces motoring authority toward no-load speed, retains bounded opposing electrical
torque, and treats zero duty as coast. It does not claim battery regeneration, bus-voltage, current, or thermal
accuracy.

### Service brake

The first service-brake provider uses a normalized request, apply/release response, maximum torque, and data-authored
wheel distribution. Away from zero speed, each wheel torque opposes its angular velocity:

```text
requested_torque_i = brake_state * max_torque * wheel_fraction_i
```

The brake plant limits actuator torque only. It never clips torque against `mu * normal_load`; Bullet or the selected
tire/contact provider owns available ground force and wheel slip. An analytic chassis backend must apply that limit
once in its contact model.

Near zero speed, a sign function is insufficient because it reverses torque and chatters. A complete provider changes
to a bounded wheel-lock or stick constraint. The constraint releases on throttle/release intent and cannot prevent
the whole locked tire from sliding when surface friction is insufficient.

### Parking brake

Parking brake is a latched capability with its own maximum torque or lock constraint. It must resist a sustained
grade or external load inside its actuator and tire-contact envelope. Directly setting body velocity to zero and
relying on an unlimited static-friction anchor are invalid implementations.

### ABS and blending

ABS is optional. It estimates longitudinal slip from wheel and vehicle motion, reduces per-wheel service-brake
request above a declared slip target, and restores it after recovery. Brake blending allocates requested deceleration
between electrical and friction braking while maintaining the same wheel/contact limit. Placement is a declared
capability: a smart drive may own ABS and blending, while a passive research plant may expose per-wheel commands to
a deployable ROBOT controller.

## Data and interface contract

The eventual service-brake data contains maximum torque, wheel or axle distribution, apply/release time constants,
low-speed transition parameters, and optional thermal state. LSRI exposes either a normalized service/parking request
with a declared plant mapping or explicit per-wheel effort commands; it never reinterprets a velocity target as brake
torque.

`tires.brake_decel_mps2` is legacy planning configuration and must be split. Brake hardware owns maximum torque;
tire and surface data own friction/contact; the runtime capability derives a conservative available-deceleration
estimate for planning from current mass, grade, load, actuator state, and contact evidence.

## Acceptance sequence

1. Verify electric braking and coast independently at several initial speeds.
2. Add an optional service-brake provider and test torque sign, saturation, response, release, per-wheel distribution,
 stopping distance, load transfer, and high/low-grip slip.
3. Add bounded parking lock and test hold/release below and above brake-torque and tire-adhesion grade limits.
4. Add ABS only to profiles that declare it; compare locked-wheel and controlled-slip runs with identical seeds,
 surfaces, loads, and initial speeds.
5. Add temperature, fade, wear, hydraulic pressure, or an identified tire model only when a lesson or measured target
 requires them.

Every run records initial speed, stop time and distance, peak deceleration, wheel/body speed difference, slip duration,
requested and delivered torque, saturation, and grade-hold displacement. Mission completion is not a brake-model
oracle.
