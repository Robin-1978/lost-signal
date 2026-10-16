# Lost Signal

> An open-source robotics learning adventure that turns real autonomous-driving algorithms into observable, testable, reproducible experiments.

Lost Signal is a third-person robotics adventure and an educational simulation. You enter Lan Valley as a field engineer, restore damaged infrastructure, investigate a mysterious transmission, and work alongside RO-1, an autonomous robot companion.

You choose high-level intent: follow, travel, search, wait, or return. RO-1 decides how to get there. Perception, state estimation, global planning, local collision avoidance, chassis control, and Bullet physics all run during the mission. Except for the opening chassis-calibration lesson, the keyboard never directly drives the robot.

The current release is a `v0.1.0-alpha` source preview. The simulation core, Linux client, deterministic headless runner, and cross-platform tests are usable. Art, audio, accessibility, balance, and packaged releases are still in development.

## Why this project is different

- **Autonomy is the game mechanic.** A*, Hybrid A*, DWA, dynamic-object prediction, kinematic conversion, motor control, and PID feedback determine what RO-1 actually does.
- **The robot's reasoning is visible.** Switch between the field view and the AI view to inspect LiDAR returns, estimated pose, global paths, local trajectory candidates, terrain costs, wheel targets, encoder feedback, and PWM output from the same simulation snapshot.
- **Failure produces evidence.** Slippery surfaces, slopes, occlusion, sensor noise, missing returns, and moving forklifts create causal events that can be replayed and tested by changing one variable at a time.
- **Five chassis families have distinct behavior.** Differential, tracked, Ackermann, swerve, and mecanum drives use their own feasible-motion contracts, controllers, and actuator mappings.
- **Experiments are reproducible.** Fixed timesteps, seeded scenarios, JSON headless output, schemas, and automated tests support classes, algorithm comparisons, and community contributions.
- **Content is data-driven.** Missions, maps, scenes, robots, hardware, policies, story beats, presentation, and UI text load from `assets/`.

Lost Signal uses C++20, Bullet 3.25, raylib 6.0, TGUI 1.13, nlohmann/json, CMake, and glTF 2.0/GLB.

## Play and learn

The campaign follows a simple learning loop:

1. Change one component or driving preference in the garage.
2. Predict what will happen.
3. Give RO-1 a mission and watch it drive autonomously.
4. Open the AI view with `Tab` and connect sensor evidence to planning and control.
5. Use the debrief to find the first meaningful difference, then run the experiment again.

The opening mission is a manual chassis calibration: `WASD` or the arrow keys control RO-1 so that acceleration, steering, and braking are tangible. Every later mission is autonomous.

In Lan Valley Expedition, `WASD` moves the field engineer. Press `F` to ask RO-1 to follow, `G` to go to the cursor, `Q` to search near the cursor, `X` to wait, and `R` to return. RO-1 plans the route, avoids obstacles, selects speed and steering, and drives its actuators. Right-drag rotates the camera, middle-drag pans, the wheel zooms, and `C` resets the camera.

During a mission, `Tab` opens the AI view. `F1`–`F4` select elevation, slope, vehicle risk, and final navigation cost. `Space` pauses, while `1`, `2`, and `4` select simulation speed.

The [learning guide](docs/learning-guide.md) maps the eight main missions to perception, localization, planning, control, interaction, and terrain concepts. The simulation uses executable algorithms and consistent SI units, but its scenarios are deliberately simplified for teaching. It is not a safety-certified system for public roads or physical robots.

## Run it

On Linux or macOS, from the repository root:

```bash
./tools/run.sh
```

Open the final mission directly in the AI view:

```bash
./tools/run.sh --demo --level lost_signal --analysis-view
```

The first run checks CMake, downloads pinned dependencies with CMake `FetchContent`, builds the project, and launches the game. Later runs reuse the build. No account, launcher, or license activation is required.

On Ubuntu or Debian, install the platform development packages once:

```bash
sudo apt-get install build-essential git curl libasound2-dev libx11-dev \
 libxrandr-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev \
 libxcursor-dev libxinerama-dev
```

On macOS, install the command-line tools with `xcode-select --install`.

Player progress is stored in the platform user-data directory. Use `--save FILE` for a specific save file or `--no-save` to disable persistence.

### Language

The game defaults to English and currently supports English and Simplified Chinese:

```bash
./tools/run.sh --language en
./tools/run.sh --language zh-CN
```

Source code, documentation, tests, schemas, and headless output use English. Locale overlays may replace player-facing strings only; the validator rejects overlays that target physics, scoring, paths, IDs, or algorithm parameters.

## Build, test, and run headlessly

```bash
# Graphical client, simulation core, and tests
./tools/build.sh --test

# Portable simulation core and tests without graphical dependencies
./tools/build.sh --headless --test

# Run one mission with its recommended build
./tools/run.sh --headless --level lost_signal --recommended

# Exercise a high-level companion command and emit machine-readable output
./tools/run.sh --headless --level boot --endless --duration 8 \
 --intent go-to --target 6 4 --json

# Remove the selected build directory and rebuild
./tools/build.sh --clean
```

Graphical builds use `build/release/`; headless builds use `build/headless/`. Override their location when needed:

```bash
LOST_SIGNAL_TOOLCHAIN_DIR=/path/to/lost-signal-toolchain \
LOST_SIGNAL_BUILD_DIR=/path/to/lost-signal-build \
./tools/run.sh
```

## What is simulated

The compact [Lost Signal Runtime Interface](docs/runtime-interface.md) connects autonomy to simulation, replay, or future native hardware drivers without depending on ROS 2. Its versioned ports validate capabilities, timestamps, sources, sequences, command expiry, lifecycle, and health. An optional bridge can translate LSRI to ROS 2 or another middleware at the edge.

The runtime publishes a `map → odom → base_footprint → base_link → sensor/wheel link` transform tree. LiDAR, encoder, IMU, and localization observations carry timestamps, sequence numbers, source IDs, frame IDs, and latency metadata. Sensor extrinsics affect ray origins and projection.

Spinning LiDAR models sample individual beams at 5, 10, or 20 Hz and include motion distortion, estimated-motion deskew, first-return intensity, bias, white noise, quantization, and missing returns. The IMU models bias, random walk, and angular-velocity and acceleration noise. Encoders are quantized by counts per revolution. These effects alter observations rather than hidden world truth.

The 3D field view and 2D AI view read the same immutable simulation snapshot. Switching views cannot change the timestep or mission result. Bullet is the authoritative and default rigid-body backend in both graphical and headless runs. A deterministic planar backend remains available for focused regression tests and algorithm comparison.

These interfaces create a useful path toward ROS 2 adapters, calibration, log replay, and hardware-in-the-loop tests. They do not by themselves prove sim-to-real transfer. See the [architecture](docs/architecture.md) and [sim-to-real roadmap](docs/sim2real.md) for the exact boundary.

## Repository map

```text
assets/ Missions, maps, scenes, robots, components, locales, and schemas
src/core/ Dependency-light shared primitives
src/hw/ Chassis, motor/contact, sensor, and hardware-abstraction models
src/autonomy/ Localization, mapping, perception, behavior, planning, and control
src/mission/ Content loading, mission simulation, world actors, progression, and scoring
src/physics/ Bullet adapter, physics ports, transforms, and regression backend
src/runtime/ Versioned robot I/O contracts, provider registry, watchdog, and replay driver
src/app/ Headless application entry point
src/client/ raylib field/AI rendering and TGUI player interface
tests/new/ C++ content, algorithm, physics, and mission regression tests
docs/ Design, learning, architecture, authoring, and validation guides
tools/ Build, run, generation, and content-validation scripts
```

Start with the [creative direction](docs/creative-direction.md), [product roadmap](docs/product-roadmap.md), [game design](docs/game-design.md), [architecture](docs/architecture.md), [autonomy extension guide](docs/extending-autonomy.md), [runtime interface](docs/runtime-interface.md), [Run Artifact](docs/run-artifacts.md), and [validation guide](docs/validation.md). To contribute, read [CONTRIBUTING.md](CONTRIBUTING.md), the [Code of Conduct](CODE_OF_CONDUCT.md), and [SECURITY.md](SECURITY.md). Asset sources and generation methods are recorded in [asset provenance](docs/asset-provenance.md).

## License

Original code, configuration, missions, and documentation are licensed under the [Apache License 2.0](LICENSE). It permits use, modification, distribution, and commercial use while requiring preservation of the license and applicable notices and providing an explicit patent grant.

Bullet, raylib, TGUI, nlohmann/json, and Noto Sans CJK retain their own licenses. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
