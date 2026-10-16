# Changelog

Notable changes to Lost Signal are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project intends to use [Semantic Versioning](https://semver.org/spec/v2.0.0.html) after the first public release.

## [Unreleased]

### Added

- English-default game localization with Simplified Chinese support.
- Sparse, string-only locale overlays and locale validation.
- Autonomous Lan Valley Expedition commands for follow, go-to, search, wait, and return.
- Differential, tracked, Ackermann, swerve, and mecanum chassis families.
- Terrain-aware planning, slope and rollover risk, LiDAR/IMU/encoder observation models, and Bullet-backed vehicle motion.
- AI inspection layers for elevation, slope, vehicle risk, and final navigation cost.
- Deterministic headless execution, JSON output, and cross-platform CI.
- A three-beat `lost_signal` encounter with an autonomous blind-crossing decision, persistent landmarks, state-driven effects, and original generated sound cues.
- Structured decision evidence shared by the field view, AI view, debrief, headless output, replay snapshots, and regression tests.
- Run Artifact v1 with source, content, configuration, algorithm, event, decision, and result identity, plus strict reproduction and controlled comparison tools.
- Lost Signal Runtime Interface v1 for versioned, pluggable simulation, replay, native hardware, and optional middleware bridges.
- Eight main missions, contracts, experiments, story beats, and data-driven presentations.

### Changed

- Reframed the project as an autonomous robotics learning adventure centered on prediction, observation, debrief, and retry.
- Moved player input in expedition mode to the field engineer and high-level intent; RO-1 remains autonomous.
- Refined the generated research-base props and forklift materials, geometry, work lights, and encounter readability.
- Expanded CI to validate content, locales, authored contracts, deterministic generators, sanitizers, portable headless builds, and the Linux graphical client.
- Consolidated public documentation around product direction, learning, architecture, content authoring, validation, and sim-to-real evidence.

### Removed

- Repository screenshots and documentation image references.
- Obsolete internal phase reports and frozen planning notes.

## [0.1.0-alpha] - Unreleased

Initial public source preview.
