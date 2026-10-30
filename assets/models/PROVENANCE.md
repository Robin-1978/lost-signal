# Model provenance audit

Audit date: 2026-10-31

The RO-6 articulated-link set (`robots/ro6/*.glb`) is authored in Blender by
`tools/modeling/build_arm_assets.py`.  Its four link frames use the same
contract as `MotorParams::ArmParams`: the joint pivot is at x=0 and the link
extends along local +X for the configured link length.  The meshes are
presentation-only and are licensed CC0-1.0 for this repository; MuJoCo body,
joint, contact and encoder read-back remain the sole pose authority.

The 39 entries that previously used `NOASSERTION` are the legacy-main
migration set. Each file is present in the repository history and its first
committing author is the project contributor (`Robin`,
`93695114+Robin-1978@users.noreply.github.com`). The manifest retains the
original `legacy-main:*` source artifact for every entry. No external URL,
third-party package, or imported source asset is recorded for this set.

The repository's Apache-2.0 `LICENSE` and `NOTICE` are therefore the current
license basis for these project-authored files. The audit is deliberately
source-based: it does not infer a license from mesh dimensions, file names, or
the fact that a model renders correctly. If a contributor later identifies an
external source, its exact license and attribution must replace the current
entry before redistribution.
