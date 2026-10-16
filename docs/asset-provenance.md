# Asset provenance

This file records the origin and reproducible generation path for assets in the public source package. Update it or `THIRD_PARTY_NOTICES.md` whenever an asset is added.

## Original project assets

The following assets are produced by project source and generators and are released under Apache-2.0:

| Asset | Location | Reproduction command |
|---|---|---|
| Robot models | `assets/models/robots/` | `python3 tools/generate_robot_models.py` |
| Dynamic actor models | `assets/models/actors/` | `python3 tools/generate_actor_models.py` |
| Environment models | `assets/models/environment/` | `python3 tools/generate_environment_models.py` |
| Environment textures | `assets/textures/environment/` | `python3 tools/generate_textures.py` |
| Encounter sound cues | `assets/audio/` | `python3 tools/generate_audio.py` |
| Missions and presentations | `assets/levels/`, `assets/presentation/` | `python3 tools/generate_levels.py` |
| Weekly contracts | `assets/contracts/` | `python3 tools/generate_contracts.py --week 1` |

Where a generator exists, it is the source of truth. Change the generator and run its `--check` mode or related regression test rather than hand-editing only its output.

## Third-party assets

`assets/fonts/NotoSansCJKsc-Regular.otf` is Noto Sans CJK SC under the SIL Open Font License 1.1. Its license is stored at `assets/fonts/OFL.txt`.

Bullet, raylib, TGUI, and nlohmann/json are downloaded build dependencies and retain their own licenses. They are not relicensed under Apache-2.0. See `THIRD_PARTY_NOTICES.md` for versions, sources, attribution, and license details.

## Contribution requirements

Do not submit an asset with unknown provenance, personal-use-only terms, a no-redistribution restriction, or unclear generated-content rights. A pull request that adds a third-party asset must state the original URL, author, exact license, modifications, and required notice. Use project-generated placeholder geometry when the rights cannot be verified.
