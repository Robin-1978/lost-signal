# Content and hardware authoring guide

Lost Signal keeps hardware data and algorithm code separate. Existing component families, robots, missions, maps, and presentations can be extended with JSON. A new motion model or algorithm requires C++ and tests.

## JSON extensions

| Extension | Location | Main checks |
|---|---|---|
| Tire, motor, LiDAR, IMU, computer, or policy | `assets/components/*.json` | required behavior fields, unique ID, cost |
| Robot using a known chassis family | `assets/robots/*.json` | frames, mounts, collision shape, drive parameters |
| Mission | `assets/levels/*.json` and `assets/presentation/*.json` | schema, references, budget, learning contrast |
| Map actors and markers | `assets/maps/*.json` | valid references, canonical round trip |
| Scene | `assets/scenes/*.json` | valid models, materials, lighting, and camera data |
| Ability or reward | `assets/abilities.json` | `ability.*` ID and valid unlock chain |
| Locale | `assets/locales/<locale>/...` | string-only pointers and placeholder parity |

Use a stable, lowercase ID and keep player-facing names translatable. Do not encode behavior in a display string.

## A new mission

A mission needs more than a destination. Define one learning question, a visible simulated contrast, a bounded objective, scoring thresholds, a default and recommended build, prediction choices, runtime callouts, and debrief explanations.

Add or update:

1. the level definition in `assets/levels/`;
2. its map and scene references;
3. one presentation in `assets/presentation/`;
4. chapter/campaign placement if it is part of progression;
5. English locale overlays for every player-facing string;
6. generator inputs when the file is generated;
7. focused tests for new behavior or contracts.

Run the mission with multiple seeds before setting score thresholds. Thresholds should describe a repeatable tradeoff, not a single lucky run.

## A new component

Copy the smallest component in the matching slot, give it a unique ID, and provide every behavior property required by that slot. Use measurements or a documented source for physical values. Ensure its cost fits at least one intended mission build.

A component may modify declared behavior parameters. It may not select an algorithm by smuggling a code name into arbitrary content data.

## A new chassis family

Implement `ChassisModel`, including stepping, telemetry, and `ChassisCaps`. Add a `ChassisBuildParams` creator, register it explicitly in `register_builtin_chassis`, add the source to CMake, and define a robot that uses the new `drive.type`.

The capability contract must describe command space, lateral motion, turning constraints, and limits used by planning. Add inverse-consistency, saturation, telemetry, and mission-level tests. The factory should not require mission-specific branches.

## Locales

`assets/locales.json` declares the default, source, and supported locales. A locale file mirrors the source file's relative path and maps JSON Pointers to translated strings:

```json
{
 "schema_version": 1,
 "translations": {
 "/title": "Example Mission",
 "/teaching/prediction_prompt": "What will RO-1 observe first?"
 }
}
```

Preserve placeholders (`{0}`), intended line breaks, units, and technical terms. The locale validator requires full coverage for the default English locale and rejects pointers to non-string values.

## Validation workflow

```bash
python3 tools/validate_content.py
python3 tools/validate_locales.py
python3 tools/generate_levels.py --check
python3 tools/generate_contracts.py --week 1 --check
python3 tests/test_content_roundtrip.py
./tools/build.sh --headless --test
```

For experimental content, copy `assets/` outside the repository and use it as an overlay or separate pack. Do not edit generated output without updating its generator.

## Integrity rules

- identical content, configuration, seed, and backend should produce the same result;
- sensor-facing algorithms may not read hidden actor truth;
- physical values come from a source or measurement, not from tuning solely to pass a mission;
- new telemetry must be represented in headless or automated checks when practical;
- third-party assets require source and license records in `docs/asset-provenance.md` or `THIRD_PARTY_NOTICES.md`.
