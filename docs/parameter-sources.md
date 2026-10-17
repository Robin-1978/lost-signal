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

## Wheel-ground contact

There is no universal rubber stiffness. The standard tire default is derived from a declared operating point:
100 N divided by 2.5 mm gives `40,000 N/m`. For effective supported mass `m`, stiffness `k`, and damping ratio `zeta`,
the linearized pair damping is:

```text
damping = 2 * zeta * sqrt(k * m)
```

The shipped values are effective wheel-ground pair parameters around normal RO-1 load. They combine tire, mount,
floor, bearing, and structural compliance. They do not claim a globally linear rubber law. Two linear normal
compliances in series combine as `1/k_pair = 1/k_wheel + 1/k_ground`; damping is a pair assumption and is not blindly
summed as a physical identity.

Friction coefficients, rolling resistance, and normal compliance remain separate configuration values. The steel
wheel profile is deliberately much stiffer and lower-loss, while still finite. Solver chatter is a numerical
failure and is never presented as physical vibration.

RO-1's stock caster has no suspension travel. It is a massive fork with a vertical swivel, a separate rolling wheel,
and finite caster trail. Contact compliance represents the small deformation of tire, mount, floor, bearings, and
structure around the operating load. A sprung caster is a different assembly profile and requires an authored
prismatic joint, travel, preload, stiffness, damping, and stops. It is not a numerical stabilization option.

## Compute and algorithm cadence

Compute profiles are controlled experiment settings. Decision rate, prediction horizon, track age, and nominal
power create visible autonomy trade-offs, but they do not model a named processor, scheduler, cache, or operating
system. Their fidelity is `ideal` by design.

## Validation and extension rule

A new typical profile must state its units, source or assumption, valid range, and the behavior it is expected to
change. Loaders reject non-finite values and invalid relationships such as LiDAR minimum range greater than maximum
range. Tests use geometry, dimensional relationships, independent analytic results, and deterministic replay rather
than copying the implementation formula into every oracle.

An identified profile may be added later without changing these contracts. It must name the device or robot,
calibration conditions, dataset, fit method, and held-out error envelope. Those requirements apply to the optional
claim, not to publishing Lost Signal as a simulation and knowledge-learning project.
