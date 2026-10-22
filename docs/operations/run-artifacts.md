# Run Artifact v3

A Run Artifact records enough identity and result evidence to rerun and compare one Lost Signal experiment. It
is a compact JSON manifest, not an interactive replay and not a sensor log.

Create an artifact from the recommended flagship run:

```bash
./build/headless/lost_signal_headless \
 --level lost_signal --recommended \
 --artifact /tmp/recommended.run.json
```

The artifact records:

- schema version, project version, source revision, dirty state, compiler, build type, and platform;
- a SHA-256 manifest of every JSON content file under the selected assets directory;
- mission, authored condition, seed, fixed timestep, physics backend, and LSRI provider;
- actual robot, selected components, style, and requested chassis override;
- global and local planner IDs, high-level policy, autonomous task-ordering policy, localization mode, controller,
 and runtime interface;
- a canonical configuration digest, result metrics, causal events, and structured decision evidence;
- an optional canonical construction plan, plan digest, resulting map revision, committed piece IDs, physical-contact
 and full-traversal evidence;
- external trace references, currently empty because Run Artifact v3 does not embed sensor streams.

Version 3 adds constructed-world identity and proof metrics. Version 2 added task-ordering identity and the task,
payload, and cargo-integrity results used by Autonomy Playgrounds. Earlier artifacts are intentionally rejected
during active development rather than silently reinterpreted.

The content manifest hashes each JSON file as sorted relative path, a NUL byte, its exact bytes, and another
NUL byte. Player models, textures, sounds, fonts, and screenshots do not change a simulation comparison.

## Reproduce a run

Use the manifest itself to restore its mission, seed, backend, build, and style:

```bash
./build/headless/lost_signal_headless \
 --reproduce /tmp/recommended.run.json \
 --artifact /tmp/reproduced.run.json

python3 tools/compare_runs.py \
 /tmp/recommended.run.json /tmp/reproduced.run.json
```

Reproduction rejects a changed content hash, damaged configuration digest, unsupported schema, different fixed
timestep, unavailable provider, or a non-authored mission mode. Source revisions may differ because regression
comparison across commits is a primary use. A dirty producer is recorded and should not be used as a release
baseline without preserving its patch.

## Compare one controlled variable

The comparator rejects undeclared configuration changes. Declare the exact leaf that an experiment intends to
change:

```bash
./build/headless/lost_signal_headless \
 --level lost_signal --default \
 --artifact /tmp/light-computer.run.json

python3 tools/compare_runs.py \
 /tmp/recommended.run.json /tmp/light-computer.run.json \
 --vary hardware.components.computer
```

Mission, condition, mode, seed, physics backend, provider, timestep, duration, and content hash are invariants;
the tool never permits `--vary` to waive them. It verifies each configuration digest before comparing status,
time, path length, energy, clearance, localization error, collisions, slip, planning work, rollover risk, and
score changes. `--json` emits the comparison report for automation.

## Separate evidence formats

- Interactive replay stores snapshots for seeking the player UI.
- LSRI replay feeds recorded observation packets back through runtime validation.
- Run Artifact v3 identifies an experiment and its compact outcome.
- A future trace store may hold large LiDAR, IMU, encoder, and command streams by digest and place references in
 `inputs.trace_refs`.

Keeping these formats separate prevents a small regression result from becoming a proprietary log container or
a second middleware protocol.
