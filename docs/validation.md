# Validation guide

Lost Signal treats a release as a reproducible set of source, content, and behavior checks. A green test suite supports the declared simulation scope; it does not certify physical-robot safety or sim-to-real accuracy. The
[verification strategy](verification-strategy.md) defines the contract, invariant, independent-oracle, deterministic,
fault, and physical evidence required for stronger claims.

## Build profiles

### Canonical workspace build

Everyday development and release validation use one directory, `build/`. MuJoCo is a required dependency of the
simulation, so there is no separate "with physics engine" build directory: a simulation build always contains it.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
 -DCMAKE_CXX_COMPILER=/usr/bin/g++-13 -DCMAKE_C_COMPILER=/usr/bin/gcc-13
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The two profiles below remain available for focused work (a headless machine without window-system packages, or a
sanitizer run). They are temporary directories; delete them when the investigation is finished so the workspace keeps
a single build tree. Compiler cross-checks (Clang 18) are built the same way in a scratch directory and removed once
the difference is explained.

### Portable headless

```bash
cmake -S . -B build/headless \
 -DLOST_SIGNAL_BUILD_CLIENT=OFF \
 -DLOST_SIGNAL_BUILD_TESTS=ON \
 -DCMAKE_BUILD_TYPE=Release
cmake --build build/headless --parallel
ctest --test-dir build/headless --output-on-failure
```

This profile builds the simulation core and tests without window-system dependencies. CI runs it on Linux, macOS, and Windows.

### Sanitized core

```bash
cmake -S . -B build/sanitizer \
 -DLOST_SIGNAL_BUILD_CLIENT=OFF \
 -DLOST_SIGNAL_BUILD_TESTS=ON \
 -DLOST_SIGNAL_ENABLE_SANITIZERS=ON \
 -DLOST_SIGNAL_WARNINGS_AS_ERRORS=ON \
 -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sanitizer --parallel
ctest --test-dir build/sanitizer --output-on-failure
```

The manually dispatched and version-tag CI job uses AddressSanitizer and
UndefinedBehaviorSanitizer. Routine branch pushes use the faster Release suite;
run sanitizers for milestone and release-candidate validation.

Some managed Linux sandboxes inject a support library before the executable and cause GCC ASan to stop with
`ASan runtime does not come first`. In that environment, preload the runtime from the same compiler used by the
sanitizer build. Keep leak detection enabled for the C++ tests; disable it only for the Python wrapper processes,
whose interpreter shutdown allocations are outside this project:

```bash
asan_runtime="$(${CXX:-c++} -print-file-name=libasan.so)"
LD_PRELOAD="$asan_runtime" ctest --test-dir build/sanitizer --output-on-failure \
 -E 'playtest_summary|content_roundtrip|compare_runs|deployable_boundary|run_artifact'
LD_PRELOAD="$asan_runtime" ASAN_OPTIONS=detect_leaks=0 \
 ctest --test-dir build/sanitizer --output-on-failure \
 -R 'playtest_summary|content_roundtrip|compare_runs|deployable_boundary|run_artifact'
```

This workaround changes only process startup in the affected sandbox; it is not required by normal CI runners.

### Graphical client

```bash
./tools/build.sh --test
```

This verifies dependency integration and the client build in addition to the shared core. A virtual display smoke run checks that the client can initialize and execute a demo in CI.

## Content checks

```bash
python3 tools/validate_content.py
python3 tools/validate_locales.py
python3 tests/test_content.py
python3 tests/test_content_roundtrip.py
python3 tests/test_contracts.py
python3 tools/generate_levels.py --check
python3 tools/generate_contracts.py --week 1 --check
python3 tools/generate_audio.py --check
```

These checks cover JSON Schema, cross-file references, budgets, unlocks, required presentation coverage, source/default locale coverage, placeholders, string-only locale targets, canonical serialization, and generator drift, including byte-identical generated sound cues.

The graphical executable accepts `--mute`. Demo and screenshot automation disable audio automatically, and the
headless target does not compile or link the client audio director.

## Behavioral evidence

C++ tests cover content loading, component registration, field intent, world activity, physics behavior, terrain, chassis kinematics, high-level policy, mission completion, dynamic perception, waiting behavior, entry governance, estimation, replay, slip observation, control consistency, player content, contracts, and deterministic scenario matrices. Local-planner fixtures require differential, Ackermann, and holonomic commands to remain inside the next decision interval's reachable velocity window; differential commands are checked again in left/right wheel space. Terrain fixtures independently check asymmetric front/rear support and grade-dependent longitudinal stability limits. The orchard matrix exercises every task-ordering policy with changing payload, center of mass, terrain, grip, staged no-trajectory stops, and a physical return to the packing pad.

Mission completion and performance ranking are separate contracts. A run is
ranked only after its objective succeeds with zero collision/rollover failure.
The GAME result then records raw time, clearance, estimated energy, command
variation, wait/recovery duration and action changes, and derives five 0--100
axes plus an overall score. Ideal clearance and target pace/energy saturate at
100; falling below an ideal clearance can reduce quality without converting a
physically valid run into failure. `performance_score` pins failed-run
exclusion, the 95/133 pace ratio, clearance saturation, energy ordering,
smoothness and decisiveness. None of these values enters ROBOT admission.

The ROBOT-only `safe_stop_limiter` fixture independently checks that a moving
chassis cannot receive an instantaneous zero target, each differential wheel
respects the next control interval, downhill grade and a taller combined COM
reduce the available stop step, and a COM projection on the support edge
reports zero stability margin. The local trajectory validator consumes that
same declared deceleration for its emergency-stop tail; neither check reads
MuJoCo contact truth.

Longitudinal planning keeps three quantities distinct. Normal drive
acceleration is bounded by the declared motor/wheel and traction envelope;
normal deceleration is the actuator's reachable braking rate. These two rates
form an asymmetric next-update velocity window, Emergency-stop deceleration is a third,
stability-aware value used only to validate the delayed braking tail and stage
a rejected command toward zero. A configured emergency-stop contract whose
current safe authority is zero rejects moving candidates; zero cannot disable
the check. Focused tests cover asymmetric forward and reverse windows plus the zero-authority rejection.

Cross-compiler evidence separates three questions:

- **Numerical correctness:** floating-point results stay within the declared oracle tolerance. GCC and Clang may
 round expressions differently, and MuJoCo contact/friction active sets may switch at those boundaries.
- **Decision stability:** local-planner scores inside a scale-aware 64-ULP band use an explicit secondary order
 (smaller turn demand, then smaller translation and stable signed keys). A meaningful score difference still wins;
 the tie rule does not enlarge collision or stability limits.
- **Physical safety:** every selected rollout independently satisfies collision, prediction-horizon, terrain,
 support-polygon, dynamic-load and emergency-stop-tail constraints. GCC and Clang may produce different safe
 trajectories and completion times; both must satisfy the same quantitative contract.

The compiler, compiler version, build type and source identity belong in every retained run artifact. Cross-compiler
validation compares safety invariants and explainable decisions rather than requiring bit-identical floating-point
trajectories. Bit identity is only required when a narrower test explicitly declares it.

A deterministic release run should also be recorded:

```bash
./build/headless/lost_signal_headless \
 --level lost_signal --recommended --json
```

Retain the revision, command, seed, backend, platform, and output when using a run as evidence. A single successful run does not establish a probability of success; multi-seed claims require a declared seed set and aggregate results.

For a versioned, self-identifying result and a strict comparison, use:

```bash
./build/headless/lost_signal_headless \
 --level lost_signal --recommended --artifact /tmp/recommended.run.json
./build/headless/lost_signal_headless \
 --reproduce /tmp/recommended.run.json --artifact /tmp/reproduced.run.json
python3 tools/compare_runs.py /tmp/recommended.run.json /tmp/reproduced.run.json
```

See [Run Artifact v3](run-artifacts.md) for controlled-variable comparison. Artifacts produced from a dirty source
tree retain `source_dirty: true`; release evidence uses a clean revision.

## Locale checks

The default `ContentCatalog::load(assets)` must select English. Explicit `zh-CN` loading must select the source Chinese strings. Unsupported locale codes must report an error. Tests compare both catalogs to ensure localization leaves mission IDs, content graph, paths, seeds, physics, scoring, and algorithm configuration unchanged.

## Current release-candidate status

The current source revision is **not yet covered by a complete release record**. Recent simulation and autonomy work
changed estimation, LiDAR projection, terrain composition, local-planner reachable command windows, trajectory
validation, staged safe stopping, crossing release, path smoothing and differential-controller transitions. Focused
fixtures now include an analytic circular-fillet oracle and bounded-shuttle reassociation after a 2.4 s occlusion.
Earlier timing and clearance baselines were invalidated by the corrected score/safety split and must not be used as
current release evidence. These focused fixtures remain necessary, but they are not a replacement for the ordered
release gates above.

The former `town_freight --recommended` completion relied on an authored participant-speed limit to convert visible
approach length into an unseen-arrival bound. That evidence remains withdrawn because an ordinary robot map cannot
know an unknown real participant's physical capability. Complete LiDAR coverage remains useful tracking evidence but
does not grant track-free release. Under the corrected contract, the current seed-47 `lost_signal --recommended`
run completes in 88.78 s with zero collisions and 0.392 m minimum measured dynamic clearance. It succeeds without
the optional 0.6 m clearance badge, proving that the score target no longer acts as a ROBOT permission threshold.
`town_freight --recommended` completes in 60.04 s and `signal_echo --recommended` completes in 72.26 s, both with
zero collisions. The crossing first-clear-exit fixture prevents a raster endpoint beyond the physical refuge from
inflating execution past the prediction horizon. Focused geometry, map-adapter, typed-prediction parity, unsafe-hold,
stop-plane-preserving escape and generic unmapped-conflict tests pass. Continuous scores still expose substantial
command variation, waits, recovery time and action switches, so P1 interaction quality and the complete multi-model
prediction chain remain open even though these reference missions now finish.

Create a new record from a clean revision by running, in order:

1. GCC 13 Release build and the complete CTest suite with parallelism 8.
2. ROBOT-only build/tests and all content, schema, locale and generator checks.
3. Repeated identical-seed run artifacts and strict comparison of results and key trajectory metrics.
4. Graphical input, clean-shutdown, recommended-experience and proving-ground smoke tests.
5. Clang 18 compatibility, followed by ASan/UBSan as the final expensive gate.

Until that record exists, publish the repository only as an Alpha / Developer Preview and retain the claim limits in
[Simulation model audit](simulation-model-audit.md).

## Last completed release record

Historical baseline results for the five-chassis autonomy and platform revision tested on Linux x86_64 on
2026-10-18 follow. They remain regression context and must not be presented as validation of later revisions:

- Content validation: passed; 29 levels, 7 maps, 7 scenes, 24 components, 4 policies, 5 robots, and 29 presentations
- Locale validation: passed; 1,451 entries checked and 1,443 required translations present
- Canonical content round-trip and deployable-boundary tests: passed
- Release build with the graphical client enabled and CTest: passed; 90/90 tests
- ROBOT-only build (`LOST_SIGNAL_BUILD_SIM=OFF`): passed; 15/15 tests
- Vehicle Proving Ground matrix: all 35 autonomous combinations of five chassis and seven physical maneuvers
 succeeded with zero reported collisions; the manual LiDAR station remains an operator fixture
- Two repeated recommended `lost_signal` runs were identical: succeeded in 67.64 simulated seconds, zero collisions,
 and 0.660 m minimum dynamic clearance
- The `motor_trial` regression separates gearing tradeoffs: speed/express completes in 4.76 s while eco/balanced
 completes in 5.20 s; eco uses 0.042 Wh versus 0.068 Wh for speed
- Two repeated recommended `signal_watch` runs were identical: succeeded in 69.06 simulated seconds, zero collisions,
 and 0.034 m minimum dynamic clearance
- Graphical smoke and deterministic screenshot gates passed under Xvfb; two independent captures at 8.0 simulated
 seconds had the same SHA-256 digest
- The Bridge crossing capture at 10.0 simulated seconds is byte-identical across two independent Xvfb runs
- ASan/UBSan: the last frozen baseline passed 82/82 C++ tests with leak detection enabled and 5/5 Python wrapper
 tests with interpreter leak detection disabled. It predates the Bridge increment and is intentionally rerun after
 the next functional milestone is frozen.
- Asset provenance: verified; shipped models, textures, and audio are generated by project-owned scripts, and the bundled font retains its OFL notice
- Physical hardware: not tested

## Limits of the evidence

Automated tests demonstrate contracts and regressions represented in the repository. They do not establish gameplay enjoyment, accessibility, GPU compatibility beyond tested environments, numerical identity across every compiler, physical parameter accuracy, or safety for real hardware. Human learning feedback uses the optional protocol in [playtest-guide.md](playtest-guide.md) and is reported separately from automated gates.

The detailed claim boundary and subsystem evidence matrix are maintained in
[simulation-model-audit.md](simulation-model-audit.md). Release builds use GCC 13 explicitly; Clang 18 is a separate
compatibility job. `tools/build.sh` rejects compilers that cannot compile the C++20 atomic wait/notify API and refuses
to reuse a build directory configured by another compiler.
