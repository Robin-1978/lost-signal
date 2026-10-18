# Parameter profiles and sources

## Scope

Lost Signal is an educational simulation and experiment platform. RO-1 is a configurable reference robot, not a
digital twin of a physical unit. Its shipped defaults must be physically plausible, dimensionally correct, bounded,
and traceable to either a public specification or a stated engineering assumption. Publishing the game does not
require building or measuring a real RO-1.

Every component family declares one of three fidelity levels:

- `ideal`: a deliberate algorithm or scheduling abstraction with no device-error claim;
- `typical`: a default assembled from public specifications and stated engineering ranges;
- `identified`: parameters fitted to a named unit and accompanied by calibration and held-out validation data.

Only `typical` is required for the reference-platform release. `identified` remains an optional extension and is
required only if a future result claims calibrated accuracy or sim-to-real performance for a named robot.

## Single-plane LiDAR

The stock `sensor.standard_lidar` is based on the public operating envelope of a small rotating triangulation
scanner. The [SLAMTEC RPLIDAR A2 specification](https://www.slamtec.com/en/Lidar/A2Spec) lists, depending on model,
a 0.2 m minimum range, 12--16 m maximum range, 8--16 kHz sample rate, 5--15 Hz rotation rate, 360-degree field of
view, and 2.25--3 W power. The shipped default therefore uses 0.2--12 m, 10 Hz, 800 rays per rotation, and 3 W. It is
a class-level typical profile and does not claim to reproduce an A2's firmware or optical error distribution.

`rays` is the number of acquisitions in one rotation, not a renderer line count. At 800 rays and 10 Hz it corresponds
to an 8 kHz sampling cadence. A hit below `min_range_m` is published as invalid/no-return; the simulator does not
clamp it to the minimum and invent a valid obstacle. The scan timestamp marks acquisition end and each ray retains
its offset within the rotation.

The current constant range standard deviation and quantization are documented simplifications. The reflectivities
and the normalized intensity law are engineering assumptions used for deterministic teaching comparisons:

```text
intensity = reflectivity
          * (incidence_floor + (1 - incidence_floor) * cos_incidence)
          / (1 + distance_attenuation_per_m2 * range^2)
```

The default profile uses terrain `0.30`, static obstacle `0.68`, moving actor `0.52`, incidence floor `0.15`, and
quadratic attenuation `0.018 1/m^2`. These are dimensionless normalized returns, not calibrated reflectance or raw
device counts. Authored ground materials override the terrain fallback with their own normalized
`lidar_reflectivity`; paved, gravel, sand, water, mud, and steel are therefore independent from their friction and
contact-compliance values. The shipped material numbers are documented visual/teaching assumptions rather than
calibrated optical measurements. The other shipped LiDARs
are documented teaching variants in the same class; their shorter range, scan rate, noise, and power trade-offs are
not named commercial devices.

## 16-line spinning LiDAR

`sensor.sixteen_line_lidar` is a typical teaching profile informed by the public architecture of compact 16-channel
rotating scanners. It deliberately avoids a product name because range, power, noise, latency, beam calibration,
intensity, and packet behavior have not been identified together against one device. The authored vertical sequence
is `[-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]` degrees. Keeping this interleaved order in data
tests the distinction between ring identity, geometric elevation, and firing time.

The profile uses 360 azimuth samples, 16 rings, 10 Hz, 0.3--12 m range, 5 mm quantization, 3 cm constant range noise,
8 W teaching power, and 40 ms deterministic delivery latency. The ground projection seeds below `-0.25 m` in
`lidar_link`, accepts connected surfaces up to 30 degrees, and keeps non-ground returns between `-0.2 m` and `1.5 m`.
Those projection numbers are algorithm configuration rather than optical specifications. They must be retuned or
replaced by a 3D mapping/perception method for a differently mounted or taller robot.

The profile shares the documented synthetic reflectivity law with the planar sensor. None of these values claim the
range accuracy, channel calibration, return strength, networking, thermal behavior, or reliability of a physical
scanner. An `identified` profile must store per-beam calibration and compare timestamp, range, return rate, intensity,
and motion distortion against held-out recorded clouds.

## IMU

The basic profile uses the public noise-density scale of the
[Bosch BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/): 0.014 degrees/s/sqrt(Hz) for
the gyroscope and 175 micro-g/sqrt(Hz) for the accelerometer. In SI units these are
`0.0002443461 rad/s/sqrt(Hz)` and `0.00171616375 m/s^2/sqrt(Hz)`. Bosch also lists a typical lifetime zero offset of
plus or minus 1 degree/s for the gyro. Lost Signal uses a smaller `0.005 rad/s` one-sigma startup bias for the
class-level default; it must not be read as the distribution of a particular BMI088 lot.

Noise density is not a per-sample standard deviation. The component declares an effective noise bandwidth and the
runtime derives:

```text
sample_sigma = noise_density * sqrt(noise_bandwidth_hz)
```

The stock bandwidth is 20 Hz. The precise IMU is a lower-noise teaching profile, not an identified product. Bias
random walk remains independently configurable in `rad/s/sqrt(s)`. Physics evaluates ideal specific force and
angular rate at `imu_link`; the device model applies startup bias, random walk, white noise, timestamps, and sequence
numbers once.

## Drive motor and controller

The shipped motors are typical geared-DC teaching profiles. `stall_force_N` is wheel-rim force after gearing,
`free_speed_mps` is the no-load wheel surface speed, and the linear torque-speed approximation is:

```text
wheel_force = pwm * stall_force_N * max(0, 1 - abs(wheel_speed) / free_speed_mps)
```

The values are engineering assumptions for a 22 kg indoor service robot. They are not copied from a named motor and
do not include a thermal model. The wheel controller closes the speed loop on quantized joint encoder feedback, and
the articulated backend consumes shaft torque. Tire friction and rolling resistance are solved at contact and are
not applied a second time in the motor model.

Until a thermal profile exists, `stall_force_N` is a short-duration saturation value. It must not be interpreted as
continuous torque. The release gate checks signs, units, saturation, reverse operation, torque-limited grades, and
deterministic response; it does not claim continuous-duty sizing for hardware.

The stock speed-loop design is also configuration: `6 rad/s` bandwidth, `5 s` integral time, and `5 ms` derivative
time. Coulomb smoothing width (`0.05 m/s`) and viscous drag (`0.6 N/(m/s)`) are named drivetrain assumptions. Motor
components may replace all five values; the fallback motor inside each robot declares the same fields. None remain
as hidden constants in the differential-drive implementation.

`motor.speed` represents a higher-speed, lower-reduction drive package for controlled teaching runs. Its assumed
wheel-surface capability is 6.0 m/s and its free speed is 7.0 m/s, with 32 N short-duration rim stall force, compared
with 2.4/3.0 m/s and 54 N for the standard package. Motor capability and validated chassis rating are separate data:
RO-1 is rated at 1.0 m/s and RO-2 at 2.4 m/s. Autonomy never treats no-load motor speed as a safe chassis speed.

RO-2 uses the same class of documented motor assumptions with two rear drive channels and two front steering
channels. Its 26 kg mass, 0.62 m wheelbase, 0.58 m track, 0.10 m wheel radius, 54 N per-wheel peak rim force, 8 N m
steering stall torque, and 0.05 kg m^2 steering reflected inertia are engineering defaults chosen for the executable
teaching envelope. They are neither measurements nor a named vehicle specification. The relatively large reflected
steering inertia represents the complete servo/transmission/knuckle load seen at the joint; a future identified
profile must replace it with measured step and coast response rather than retuning it to a mission outcome.
The optional `motor.speed` package retains 6.0 m/s of motor headroom; the current RO-2 straight-line fixture validates
the 2.4 m/s chassis rating and stops in about 0.96 m. The articulated drive plant treats opposing duty as current-limited electrical braking and zero
duty as coast. RO-2 separately applies its implemented rear-wheel service-brake plant, whose torque and response time
also enter the independent stopping envelope. Neither mechanism is ABS, a parking brake, or a thermal model. Those
require separate capabilities and sourced parameters before any automotive-speed claim.

RO-3 separates its `0.55 m` physical left/right contact spacing from the `2.4 m`
`effective_track_width_m` used by skid-steer kinematics. Tracked vehicles turn around terrain-dependent
instantaneous centres of rotation, so geometric spacing alone overestimates body yaw from tread speed. The shipped
effective width is a typical engineering assumption selected for the deterministic rigid-contact envelope. A real
profile must identify it from held-out tread-speed and body-yaw data on each relevant surface; mission completion is
not a calibration method.

## Wheel-ground contact

There is no universal rubber stiffness, and the shipped typical profiles do not expose an invented linear spring.
They declare MuJoCo's positive-format contact time constant and damping ratio directly. The tire and surface values
are combined conservatively by the contact adapter, then installed as `solref = (time_constant, damping_ratio)`;
`solimp` remains an engine-owned impedance curve. A shorter time constant produces a harder numerical contact, but it
is not an independently measurable `N/m` value. The `0.006 s, 1.0` standard-tire assumption and each surface value
must therefore be validated through observable loaded height, penetration, rebound and contact load rather than
being described as identified rubber constants.

Friction coefficients, rolling resistance, and contact solver response remain separate configuration values.
MuJoCo's rolling field is a moment arm; Lost Signal derives it from dimensionless `Crr * rolling_radius` rather than
copying a universal length. The steel-wheel profile uses a shorter time constant and lower rolling loss, while still
remaining finite. Solver chatter is a numerical failure and is never presented as physical vibration.

RO-1's stock caster has no suspension travel. It is a massive fork with a vertical swivel, a separate rolling wheel,
35 mm trail, and a crowned rubber tread. The collision proxy is a centred oblate ellipsoid with 55 mm rolling radius
and 64 mm full width. This is a typical-profile geometry assumption, chosen to retain a smooth circular rolling plane
and a narrow central contact band; it is not an identified crown radius or contact patch. A measured profile should
record wheel diameter, tread width, transverse crown coordinates, hardness, static loaded radius, and swivel/rolling
breakaway torque. Contact parameters then fit load/deflection and rebound evidence instead of inferring rubber
properties from the visible mesh.

A sprung caster is a different assembly profile and requires an authored prismatic joint, travel, preload,
stiffness, damping, and stops. It is not a numerical stabilization option. MuJoCo maintainer guidance for wheeled
models likewise separates three issues: use a precisely centred smooth collision shape, soften contact to represent
rubber-like behavior, and add suspension only when the real mechanism or coplanar support requires it. See
[MuJoCo issue 57](https://github.com/google-deepmind/mujoco/issues/57). The
[MuJoCo Menagerie Stretch model](https://github.com/google-deepmind/mujoco_menagerie/blob/main/hello_robot_stretch/stretch.xml)
uses a spherical caster proxy when only support is needed; RO-1 keeps explicit swivel and rolling joints because its
caster kinematics and wheel rotation are part of the learning evidence.

## Compute and algorithm cadence

Compute profiles are controlled experiment settings. Decision rate, prediction horizon, track age, and nominal
power create visible autonomy trade-offs, but they do not model a named processor, scheduler, cache, or operating
system. Their fidelity is `ideal` by design.

The research-base crossing uses a `0.65 m` participant envelope: the authored `0.45 m` forklift footprint plus a
`0.10 m` bounded prediction envelope and `0.10 m` site reserve for clustering, map alignment, and stop tracking.
This is a conservative map assumption, not an identified vehicle parameter. It is intentionally separate from the
live track radius and from the player's safety-clearance policy, and the deterministic mission gate verifies the
resulting physical minimum clearance independently.

## Validation and extension rule

A new typical profile must state its units, source or assumption, valid range, and the behavior it is expected to
change. Loaders reject non-finite values and invalid relationships such as LiDAR minimum range greater than maximum
range. Tests use geometry, dimensional relationships, independent analytic results, and deterministic replay rather
than copying the implementation formula into every oracle.

An identified profile may be added later without changing these contracts. It must name the device or robot,
calibration conditions, dataset, fit method, and held-out error envelope. Those requirements apply to the optional
claim, not to publishing Lost Signal as a simulation and knowledge-learning project.

## Mobile manipulator

RO-6 is a typical, educational compact-arm profile rather than an identified hardware model. Its four-axis chain uses SI link dimensions, bounded revolute joints, rate-limited position references, effort-limited feedback control, and model-based gravity feedforward. The three gravity moments are derived from the authored link masses, link lengths and link-centre locations (`4.8069`, `1.88352`, and `0.23544 N m`); they are explicit deployable controller parameters rather than simulator truth. Geometry follows common small mobile-manipulator proportions; controller values are deterministic engineering defaults and require identification before any physical transfer claim. The implementation follows the separation used by MoveIt, ros2_control, and Gazebo: kinematics and planning consume joint states and limits, controller interfaces carry typed position commands, and the simulator alone resolves rigid-body forces and contacts.

References:

- MoveIt motion planning and Planning Scene concepts: <https://moveit.picknik.ai/main/doc/concepts/move_group.html>
- MoveIt hybrid planning: <https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html>
- ros2_control joint command/state interfaces: <https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html>
- Gazebo joint and joint-trajectory controllers: <https://gazebosim.org/api/sim/9/jointcontrollers.html>
