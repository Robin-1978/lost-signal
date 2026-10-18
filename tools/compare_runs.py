#!/usr/bin/env python3
"""Compare compatible Lost Signal Run Artifact v3 files."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

IMMUTABLE_PATHS = {
    "content_sha256",
    "experiment.mission_id",
    "experiment.condition_id",
    "experiment.mode",
    "experiment.seed",
    "experiment.physics_backend",
    "experiment.runtime_provider",
    "experiment.fixed_step_s",
    "experiment.requested_duration_s",
}

METRICS = (
    "status",
    "failure_reason",
    "time_s",
    "path_length_m",
    "energy_wh",
    "scored_clearance_m",
    "min_dynamic_clearance_m",
    "max_odometry_error_m",
    "collisions",
    "slip_events",
    "local_decisions",
    "terrain.max_rollover_risk",
 "environment.peak_wetness",
 "environment.min_visibility",
 "tasks.completed_count",
 "tasks.completed_goal_ids",
 "tasks.abandoned_goal_ids",
 "payload.collected_items",
 "payload.collected_value",
 "payload.mass_kg",
 "payload.cog_height_m",
 "payload.cargo_integrity",
 "construction.contact_observed",
 "construction.surface_traversed",
    "scores.safe",
    "scores.fast",
    "scores.efficient",
)


class ArtifactError(ValueError):
    pass


def canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def load_artifact(path: Path) -> dict[str, Any]:
    try:
        artifact = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ArtifactError(f"cannot read {path}: {error}") from error
    if artifact.get("schema") != {"name": "lost-signal.run", "version": 3}:
        raise ArtifactError(f"{path}: expected lost-signal.run schema version 3")
    try:
        configuration = artifact["configuration"]["comparable"]
        recorded = artifact["configuration"]["digest"]
    except (KeyError, TypeError) as error:
        raise ArtifactError(f"{path}: missing configuration identity") from error
    computed = hashlib.sha256(canonical(configuration).encode()).hexdigest()
    if recorded != computed:
        raise ArtifactError(
            f"{path}: configuration digest does not match its declared inputs"
        )
    return artifact


def flatten(value: Any, prefix: str = "") -> dict[str, Any]:
    if isinstance(value, dict):
        leaves: dict[str, Any] = {}
        for key in sorted(value):
            child = f"{prefix}.{key}" if prefix else key
            leaves.update(flatten(value[key], child))
        return leaves
    return {prefix: value}


def nested(value: Any, path: str) -> Any:
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def compare(
    baseline: dict[str, Any], candidate: dict[str, Any], varied: set[str]
) -> dict[str, Any]:
    forbidden = sorted(varied & IMMUTABLE_PATHS)
    if forbidden:
        raise ArtifactError(
            "cannot declare invariant input as variable: " + ", ".join(forbidden)
        )
    left = flatten(baseline["configuration"]["comparable"])
    right = flatten(candidate["configuration"]["comparable"])
    paths = sorted(set(left) | set(right))
    differences = {
        path: {"baseline": left.get(path), "candidate": right.get(path)}
        for path in paths
        if left.get(path) != right.get(path)
    }
    undeclared = sorted(set(differences) - varied)
    unused = sorted(varied - set(differences))
    if undeclared:
        raise ArtifactError(
            "incompatible run inputs differ without --vary: " + ", ".join(undeclared)
        )
    if unused:
        raise ArtifactError("declared variable did not differ: " + ", ".join(unused))

    metric_changes = []
    for path in METRICS:
        before = nested(baseline["result"], path)
        after = nested(candidate["result"], path)
        change: dict[str, Any] = {
            "metric": path,
            "baseline": before,
            "candidate": after,
        }
        if (
            isinstance(before, (int, float))
            and not isinstance(before, bool)
            and isinstance(after, (int, float))
            and not isinstance(after, bool)
        ):
            change["delta"] = after - before
        metric_changes.append(change)
    return {
        "compatible": True,
        "baseline_revision": baseline["producer"].get("source_revision", "unknown"),
        "candidate_revision": candidate["producer"].get("source_revision", "unknown"),
        "declared_variations": differences,
        "metrics": metric_changes,
    }


def format_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return json.dumps(value, ensure_ascii=False)


def print_report(report: dict[str, Any]) -> None:
    print("compatible: yes")
    print(f"baseline revision: {report['baseline_revision']}")
    print(f"candidate revision: {report['candidate_revision']}")
    if report["declared_variations"]:
        print("declared variations:")
        for path, values in report["declared_variations"].items():
            print(
                f" {path}: {format_value(values['baseline'])} -> "
                f"{format_value(values['candidate'])}"
            )
    print("metric changes:")
    for metric in report["metrics"]:
        line = (
            f" {metric['metric']}: {format_value(metric['baseline'])} -> "
            f"{format_value(metric['candidate'])}"
        )
        if "delta" in metric:
            line += f" (delta {metric['delta']:+.6g})"
        print(line)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument(
        "--vary",
        action="append",
        default=[],
        metavar="PATH",
        help="declare one configuration leaf as the controlled variable",
    )
    parser.add_argument(
        "--json", action="store_true", help="emit the comparison report as JSON"
    )
    args = parser.parse_args()
    try:
        report = compare(
            load_artifact(args.baseline), load_artifact(args.candidate), set(args.vary)
        )
    except ArtifactError as error:
        print(f"incompatible: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
