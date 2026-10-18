# Physics engine decision and migration

## Decision

Lost Signal uses MuJoCo 3.16.0 as its single production physics engine. The simulator has no runtime engine selector,
second production implementation, or compatibility adapter for the retired engine. The atomic cutover removed the
old implementation after the platform acceptance matrix passed.

This is an engine decision inside SIM. ROBOT, LSRI, autonomy, maps, authored robot semantics, and immutable snapshots
remain independent of the simulator library. They describe units, frames, joints, actuators, observations, and
commands; they do not expose `btRigidBody`, `mjModel`, solver parameters, or engine-specific contact flags.

## Why MuJoCo fits the product

[MuJoCo's overview](https://mujoco.readthedocs.io/en/stable/overview.html) describes an engine built around generalized
coordinates, articulated structures, optimization-based contact dynamics, joint-space inertia, and a general
actuation model. Those are a closer fit for Lost Signal's mobile manipulator and possible future legged mechanisms
than extending a game-oriented collection of independently spawned rigid bodies and constraints.

The required product capabilities have native MuJoCo representations:

| Lost Signal need | MuJoCo representation | Decision boundary |
|---|---|---|
| Mobile bases and serial arms | one compiled kinematic tree with free, hinge, and fixed relationships | compile each authored assembly as a native MuJoCo tree |
| Motor and joint dynamics | joint armature, damping, friction loss, limits, and force/position/velocity actuators | ROBOT still produces typed commands; SIM selects the declared plant model |
| Grasping and contact | convex contact dynamics, friction cones, torsional and rolling friction, contact-force queries | grasp success must use measured contact/closure evidence |
| Rubber-like effective contact | per-contact `solref`/`solimp` and solver settings | validate observable deflection, rebound, load, and slip from declared physical parameters |
| 2.5D terrain | native height fields | compile regular elevation data as a height field, with separate semantic and material layers |
| LiDAR | application-level rays using MuJoCo ray queries against physical geoms | Lost Signal retains scan timing, noise, intensity, dropout, and material-return semantics |
| IMU and encoders | body/site kinematics and joint state | sensor devices retain timestamps, bias, noise, latency, and LSRI publication |
| Dynamic actors | free-joint proxy bodies plus explicit state updates, or a deliberate model revision | topology never changes implicitly during a physics step |
| Replay and experiments | immutable `mjModel`, explicit `mjData` state, fixed step | exact state includes integration and warm-start state and records MuJoCo version and architecture |

MuJoCo is not selected because of a blanket claim that it is always faster. Its runtime avoids allocation after model
initialization and is designed for articulated robotics, but Lost Signal must publish its own fixed-model step time,
real-time factor, memory, and LiDAR throughput. Performance is an acceptance measurement, not an assumption.

The migration pins MuJoCo 3.16.0 by source archive and SHA-256
`45fe830707c80cf2fa54b37811ea13d7cc1774400704f69d85d2ecc1a151a236`, disables its examples and tests in the product
build, and records its version in run identity. A local source build established that this release does not build with
the workspace's legacy GCC 9 toolchain and does build with GCC 13. The cutover therefore raises the supported GNU
compiler floor to at least GCC 11 and verifies the actual release build with GCC 13. MuJoCo is Apache-2.0 licensed;
the third-party notices record the pinned dependency.

## Native MuJoCo architecture

MuJoCo separates the mostly immutable compiled model from mutable simulation data. Lost Signal follows that ownership
model:

```text
authored world + LSRM elevation/materials + robot assembly + actor declarations
 |
 v
 SimulationModelCompiler
 |
 v
 complete MuJoCo mjSpec / mjModel
 |
 +--------------+--------------+
 |                             |
 mjData instance               immutable bindings
 state, controls, forces       body/joint/geom/site/channel IDs
 |
 v
 fixed-step MuJoCo simulation authority
 |
 v
 LSRI observations + immutable world snapshot
```

The compiler owns topology, parent-child relationships, collision geoms, inertials, height fields, sites, actuators,
and stable IDs. The simulation instance owns time-varying state, commands, externally applied forces, contact results,
and snapshots. Rendering reads link transforms after a step and never updates physics.

Frequent gameplay changes use state, controls, or a bounded pool of predeclared objects. Dynamic traffic uses named
free-joint proxies whose state is updated before a physics step. Adding or removing a proxy, or publishing a newly
constructed bridge, is an explicit model-revision boundary that recompiles while preserving mapped free-body and
joint state. It is never disguised as a cheap mutation during `mj_step`.

## Contact and terrain rules

MuJoCo uses regularized soft constraints. [Its modeling guide](https://mujoco.readthedocs.io/en/stable/modeling.html)
recommends the Newton solver for most models and recommends elliptic friction cones, increased `impratio`, strict
tolerance, and, if needed, the NoSlip solver when slow slip matters. Lost Signal will begin with Newton plus elliptic
cones and validate each platform before enabling NoSlip. NoSlip is a numerical tool, not evidence for unlimited tire
grip or a parking brake.

Wheel-ground semantics stay data driven: tire and surface profiles define friction intent, rolling/torsional
resistance, contact time constant, and damping ratio. The model compiler maps those values to native geom/contact
settings using positive-format `solref`; `solimp` controls how impedance varies with constraint violation. These are
solver-response parameters, not physical `N/m` and `N s/m` spring constants. Native geom friction provides one
sliding coefficient, one torsional moment arm, and one rolling moment arm shared by both rolling axes. The provider
does not edit `mjContact` after `mj_step1`, because constraint construction has already occurred. Any future
anisotropic tire law must be designed before constraint assembly or implemented as an explicit validated tire model.
The mapping is accepted only when independent fixtures reproduce normal load, deflection, rebound, slope hold or
roll, stopping distance, and slip trends.

RO-1 uses a centred oblate ellipsoid as the collision proxy for its full-width crowned caster tread. This follows
MuJoCo's recommendation to prefer a smooth, accurately centred wheel proxy over a slightly asymmetric collision mesh.
The visual wheel remains cylindrical; the ellipsoid represents the rounded transverse tread rather than the visible
sidewalls. A flat full-width cylinder and a narrow flat cylinder are different physical assumptions and are not used
as substitutes for the crown.

MuJoCo renders arbitrary triangle meshes, but its normal mesh collision path uses a convex hull. Therefore Lost
Signal's regular 2.5D terrain becomes a native height field. Discrete boxes, cylinders, spheres, and validated convex
collision meshes represent obstacles and robot links. Detailed visual meshes remain in the renderer and never become
collision geometry by accident.

MuJoCo's `nativeccd` is a convex collision detector; it is not evidence that every high-speed thin-obstacle case is
safe from tunnelling. RO-2 keeps an explicit high-speed barrier test across the supported timestep and speed envelope.
If that test cannot pass with valid geometry and timestep, the product reduces the supported envelope or introduces a
separately validated continuous-collision strategy before cutover.

MuJoCo does not provide a complete Pacejka/Chrono-class road tire simply by choosing the engine. The first release
keeps the current typical, low-to-moderate-speed contact claims. A higher-fidelity automobile tire remains an optional
plant model with its own parameter source and validation evidence.

## Completed atomic cutover gate

The cutover required the following evidence with MuJoCo authority:

1. Model compilation rejects invalid trees, duplicate parents, invalid mass/inertia, inconsistent joint frames,
 missing actuator bindings, and unsupported terrain without silently changing semantics.
2. RO-1 passes rest/load balance, straight motion, in-place turn, slope, cross-slope, rough strip, low grip, braking,
 encoder, IMU, single-line LiDAR, and deterministic repeatability gates.
3. RO-2 passes Ackermann geometry, steering/drive/brake feedback, high-speed cornering, stopping, and thin-obstacle
 anti-tunnelling gates.
4. RO-3, RO-4, and RO-5 pass their existing chassis-specific contact and actuation gates.
5. RO-6 passes single-joint step, four-joint trajectory, limits, gravity hold, base reaction, contact, and grasp gates
 without teleporting an object or hiding a controller in mission code.
6. Single-line and 16-line LiDAR preserve ray geometry, occlusion, masks, per-ray timing, intensity semantics, and
 deterministic noise; representative scans meet a recorded throughput budget.
7. Fixed-seed repeated runs match the declared state and artifact fields. Replay identity records MuJoCo version,
 build identity, model digest, solver settings, timestep, and architecture.
8. The normal test suite, robot-only build, headless scenario matrix, warnings build, and final sanitizer gate pass.
9. A release benchmark records fixed-step cost, worst-case contact scene cost, 16-line LiDAR cost, memory, and real-time
 factor on the reference machine.

The migration did not preserve the unreleased physics API. Correct model/data ownership took precedence, as required
by the repository development compatibility policy.

## Sources and limits

- [MuJoCo overview and key features](https://mujoco.readthedocs.io/en/stable/overview.html)
- [MuJoCo modeling and solver guidance](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo simulation programming](https://mujoco.readthedocs.io/en/stable/programming/simulation.html)
- [MuJoCo API reference](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)

These sources establish engine semantics and capabilities. They do not prove Lost Signal's model fidelity. That claim
comes only from the cutover fixtures and declared parameter evidence.
