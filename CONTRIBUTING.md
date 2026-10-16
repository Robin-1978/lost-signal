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

- `src/hw/`: chassis, actuator, contact, sensor, and hardware-abstraction behavior
- `src/autonomy/`: localization, mapping, perception, behavior, global/local planning, and control
- `src/mission/`: mission state, content loading, telemetry, replay, and scoring
- `src/physics/`: physics interfaces and backends
- `src/client/`: rendering and interactive UI
- `assets/`: data-driven content and player-facing source text
- `assets/locales/en/`: default-English string overlays
- `tests/new/`: C++ behavioral and integration tests
- `tools/`: build, generation, and validation utilities

See [docs/diy.md](docs/diy.md) before adding a mission, component, robot, or content pack. See [docs/extending-autonomy.md](docs/extending-autonomy.md) before adding or replacing a planner.

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
./tools/build.sh --headless --test
git diff --check
```

A graphical change should also build the graphical client. Add focused tests for algorithms, physics contracts, serialization, content validation, or regressions where an automated check adds real confidence.

## Pull requests

Keep each pull request focused. Explain the player or engineering problem, the resulting behavior, and the checks you ran. If a change affects dynamics or scoring, include the scenario, seed, before/after metric, and expected physical reason. If it affects educational content, state what a learner should predict or understand.

Do not commit build directories, caches, generated screenshots, private keys, model weights, or personal configuration.

By contributing, you agree that your contribution is licensed under the repository's Apache License 2.0.
