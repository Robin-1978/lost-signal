# Validation guide

Lost Signal treats a release as a reproducible set of source, content, and behavior checks. A green test suite supports the declared simulation scope; it does not certify physical-robot safety or sim-to-real accuracy. The
[verification strategy](verification-strategy.md) defines the contract, invariant, independent-oracle, deterministic,
fault, and physical evidence required for stronger claims.

## Build profiles

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

C++ tests cover content loading, component registration, field intent, world activity, physics behavior, terrain, chassis kinematics, high-level policy, mission completion, dynamic perception, waiting behavior, entry governance, estimation, replay, slip observation, control consistency, player content, contracts, and deterministic scenario matrices.

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

## Release record

Results for the current five-chassis autonomy and platform revision, tested on Linux x86_64 on 2026-10-18:

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
