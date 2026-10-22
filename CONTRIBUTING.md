# Contributing to Lost Signal

Thank you for helping make robotics easier to understand. Contributions may improve code, missions, teaching text, translations, accessibility, tests, or documentation.

## Project principles

- Keep the robot autonomous after the opening calibration mission. Player commands express intent; planning and control decide the motion.
- Make behavior observable. A feature that changes a decision should expose enough evidence to explain that decision.
- Preserve simulation honesty. Do not read hidden world state from an algorithm that is meant to consume sensor observations.
- Keep experiments reproducible with fixed timesteps, explicit seeds, SI units, and machine-readable headless output.
- Teach one idea at a time. Missions should ask players to predict, observe, explain, and retry.
- Treat sim-to-real claims as hypotheses until hardware logs support them.

## Set up a development build

```bash
git clone https://github.com/Robin-1978/lost-signal.git
cd lost-signal
./tools/build.sh --headless --test
```

For the graphical client, install the platform packages listed in the README and run:

```bash
./tools/build.sh --test
```

## Where changes belong

- `src/robot/runtime/` and `src/robot/platform/`: LSRI messages, providers, capabilities, and deployable chassis contracts
- `src/robot/control/` and `src/robot/autonomy/`: deployable control, localization, mapping, perception, behavior, and planning
- `src/robot/executive/` and `src/robot/map_io/`: deployable task lifecycle and LSRM package I/O
- `src/sim/hardware/`: virtual chassis, actuators, contacts, sensors, and physics backends
- `src/sim/game/mission/` and `src/sim/game/scenario/`: authored world/game systems and fixed-step composition
- `src/sim/game/client/`: rendering, audio, and interactive UI
- `assets/`: data-driven content and player-facing source text
- `assets/locales/en/`: default-English string overlays
- `tests/new/`: C++ behavioral and integration tests
- `tools/`: build, generation, and validation utilities

See [docs/design/diy.md](docs/design/diy.md) before adding a mission, component, robot, or content pack. See [docs/design/extending-autonomy.md](docs/design/extending-autonomy.md) before adding or replacing a planner.

## Language and locale rules

Code, comments, documentation, test messages, schemas, and headless output are English. The game currently supports `en` and `zh-CN`, with English as the default.

Chinese source strings live in the normal content files. English translations are sparse JSON Pointer overlays under `assets/locales/en/`. An overlay may target strings only. Preserve placeholders such as `{0}` and newline structure, and use the established terms RO-1, Lan Valley, LiDAR, mecanum drive, swerve drive, tracked drive, mission, and debrief.

Run `python3 tools/validate_locales.py` after editing either source or translated text.

## Required checks

Use the smallest relevant checks while developing, then run the full headless suite before opening a pull request:

```bash
python3 tools/validate_content.py
python3 tools/validate_locales.py
python3 tests/test_content.py
python3 tests/test_content_roundtrip.py
python3 tests/test_contracts.py
python3 tools/generate_levels.py --check
python3 tools/generate_contracts.py --week 1 --check
python3 tools/generate_audio.py --check
./tools/build.sh --headless --test
git diff --check
```

Changes to experiment identity or comparison behavior should also run `python3 tests/test_compare_runs.py` and
the Run Artifact reproduction example in [docs/operations/run-artifacts.md](docs/operations/run-artifacts.md).

A graphical change should also build the graphical client. Add focused tests for algorithms, physics contracts, serialization, content validation, or regressions where an automated check adds real confidence.

## Pull requests

Keep each pull request focused. Explain the player or engineering problem, the resulting behavior, and the checks you ran. If a change affects dynamics or scoring, include the scenario, seed, before/after metric, and expected physical reason. If it affects educational content, state what a learner should predict or understand.

Do not commit build directories, caches, generated screenshots, private keys, model weights, or personal configuration.

By contributing, you agree that your contribution is licensed under the repository's Apache License 2.0.
