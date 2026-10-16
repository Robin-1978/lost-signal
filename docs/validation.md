# Validation guide

Lost Signal treats a release as a reproducible set of source, content, and behavior checks. A green test suite supports the declared simulation scope; it does not certify physical-robot safety or sim-to-real accuracy.

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

The Linux CI job uses AddressSanitizer and UndefinedBehaviorSanitizer.

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

C++ tests cover content loading, component registration, field intent, world activity, physics backends, terrain, chassis kinematics, high-level policy, mission completion, dynamic perception, waiting behavior, entry governance, estimation, replay, slip observation, control consistency, player content, contracts, and deterministic scenario matrices.

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

See [Run Artifact v1](run-artifacts.md) for controlled-variable comparison. Artifacts produced from a dirty source
tree retain `source_dirty: true`; release evidence uses a clean revision.

## Locale checks

The default `ContentCatalog::load(assets)` must select English. Explicit `zh-CN` loading must select the source Chinese strings. Unsupported locale codes must report an error. Tests compare both catalogs to ensure localization leaves mission IDs, content graph, paths, seeds, physics, scoring, and algorithm configuration unchanged.

## Release record

Results for the initial public `main` release candidate, tested on Linux x86_64 on 2026-10-16:

- Content validation: passed; 26 levels, 4 maps, 4 scenes, 17 components, 3 policies, 5 robots, and 26 presentations
- Locale validation: passed; 1,208 entries checked and 1,203 required English translations present
- Python content, round-trip, contract, and playtest tests: passed
- Level, contract, audio, model, and texture generators: up to date and reproducible
- Headless build and CTest: passed; 59/59 tests
- Graphical build and CTest: passed; 59/59 tests
- Graphical smoke run: passed under Xvfb
- Deterministic screenshots: two byte-identical runs at 2.0, 23.7, 25.0, 26.5, and 89.2 simulated seconds
- Recommended `lost_signal` mission: succeeded with seed 47 on Bullet in 89.18 simulated seconds, with zero collisions and 0.984 m dynamic clearance
- Run Artifact v1: reproduction matched result, events, decisions, and configuration identity; the declared computer-only comparison was accepted and an undeclared change was rejected
- Sanitizers: passed locally; 59/59 tests under AddressSanitizer and UndefinedBehaviorSanitizer
- Asset provenance: verified; shipped models, textures, and audio are generated by project-owned scripts, and the bundled font retains its OFL notice
- Physical hardware: not tested

## Limits of the evidence

Automated tests demonstrate contracts and regressions represented in the repository. They do not establish gameplay enjoyment, accessibility, GPU compatibility beyond tested environments, numerical identity across every compiler, physical parameter accuracy, or safety for real hardware. Human learning feedback uses the optional protocol in [playtest-guide.md](playtest-guide.md) and is reported separately from automated gates.
