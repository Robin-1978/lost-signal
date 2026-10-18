#!/usr/bin/env python3
"""Run every official build and print a product-balance report.

Examples:
 python3 tools/level_report.py --headless build-refactor/lost_signal_headless
 python3 tools/level_report.py --seeds 10 --output /tmp/level-report.md
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def default_headless() -> Path:
    candidates = (
        ROOT / "build" / "headless" / "lost_signal_headless",
        ROOT / "build-refactor" / "lost_signal_headless",
        ROOT / "build" / "release" / "lost_signal_headless",
    )
    existing = [path for path in candidates if path.is_file()]
    return (
        max(existing, key=lambda path: path.stat().st_mtime)
        if existing
        else candidates[0]
    )


def run(
    headless: Path,
    assets: Path,
    level: str,
    config: str,
    seed: int | None,
) -> dict:
    command = [
        str(headless),
        "--assets",
        str(assets),
        "--level",
        level,
        f"--{config}",
        "--json",
    ]
    if seed is not None:
        command += ["--seed", str(seed)]
    process = subprocess.run(command, capture_output=True, text=True, check=False)
    try:
        result = json.loads(process.stdout)
    except json.JSONDecodeError as error:
        detail = process.stderr.strip() or process.stdout.strip() or "no output"
        raise RuntimeError(f"{level}/{config}: {detail}") from error
    if process.returncode not in (0, 2):
        detail = process.stderr.strip() or f"exit {process.returncode}"
        raise RuntimeError(f"{level}/{config}: {detail}")
    return result


def score_code(result: dict) -> str:
    scores = result["scores"]
    return "/".join(
        "✓" if scores[key] else "·" for key in ("safe", "fast", "efficient")
    )


def outcome(result: dict) -> str:
    if result["status"] == "succeeded":
        return "Succeeded"
    return f"Failed: {result.get('failure_reason', 'unknown')}"


def single_row(level: str, config: str, result: dict) -> str:
    terrain = result.get("terrain", {})
    saturated = "✓" if terrain.get("traction_saturation_occurred", False) else "·"
    clearance = result.get("scored_clearance_m", result["min_clearance_m"])
    return (
        f"| {level} | {config} | {outcome(result)} | {result['time_s']:.2f} | "
        f"{result['energy_wh']:.3f} | {clearance:.3f} | "
        f"{result.get('max_odometry_error_m', result['odometry_error_m']):.3f} | "
        f"{terrain.get('max_grade', 0.0):.3f} | "
        f"{terrain.get('max_rollover_risk', 0.0):.3f} | "
        f"{result.get('slip_events', 0)}/{saturated} | {score_code(result)} |"
    )


def single_seed_report(
    headless: Path, assets: Path, levels: list[str]
) -> tuple[list[str], int]:
    lines = [
        "| Mission | Build | Result | Time/s | Energy/Wh | Scored clearance/m | Max odometry error/m | Max grade | "
        "Max rollover risk | Slip/traction saturation | Safe/Fast/Efficient |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    failures = 0
    for level in levels:
        for config in ("default", "recommended"):
            try:
                result = run(headless, assets, level, config, None)
                lines.append(single_row(level, config, result))
            except Exception as error:  # noqa: BLE001
                lines.append(
                    f"| {level} | {config} | Runtime error: {error} | - | - | - | - | - | - | - | - |"
                )
                failures += 1
    return lines, failures


def multi_seed_report(
    headless: Path, assets: Path, levels: list[str], seeds: int
) -> tuple[list[str], int]:
    lines = [
        f"Each build runs {seeds} seeds (level.seed + 0..{seeds - 1}); the first seed is repeated for determinism.",
        "",
        "| Mission | Build | Success | Mean time[min-max]/s | Mean energy/Wh | Max odometry error/m | "
        "Max rollover risk | Safe | Deterministic |",
        "|---|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    failures = 0
    for level in levels:
        base_seed = json.loads((assets / "levels" / f"{level}.json").read_text())[
            "seed"
        ]
        for config in ("default", "recommended"):
            try:
                trials = [
                    run(headless, assets, level, config, base_seed + index)
                    for index in range(seeds)
                ]
                repeat = run(headless, assets, level, config, base_seed)
            except Exception as error:  # noqa: BLE001
                lines.append(
                    f"| {level} | {config} | Runtime error: {error} | - | - | - | - | - | - |"
                )
                failures += 1
                continue
            deterministic = json.dumps(trials[0], sort_keys=True) == json.dumps(
                repeat, sort_keys=True
            )
            succeeded = sum(result["status"] == "succeeded" for result in trials)
            times = [result["time_s"] for result in trials]
            energies = [result["energy_wh"] for result in trials]
            odometry = max(result["max_odometry_error_m"] for result in trials)
            rollover = max(
                result.get("terrain", {}).get("max_rollover_risk", 0.0)
                for result in trials
            )
            safe = sum(result["scores"]["safe"] for result in trials)
            lines.append(
                f"| {level} | {config} | {succeeded}/{seeds} | "
                f"{statistics.fmean(times):.2f}[{min(times):.2f}-{max(times):.2f}] | "
                f"{statistics.fmean(energies):.3f} | {odometry:.3f} | {rollover:.3f} | "
                f"{safe}/{seeds} | {'✓' if deterministic else '✗'} |"
            )
            if not deterministic:
                failures += 1
    return lines, failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, default=Path("assets"))
    parser.add_argument("--headless", type=Path, default=default_headless())
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error("--seeds must be at least 1")

    assets = args.assets if args.assets.is_absolute() else ROOT / args.assets
    headless = args.headless if args.headless.is_absolute() else ROOT / args.headless
    if not headless.is_file():
        parser.error(f"headless executable not found: {headless}")
    campaign = json.loads((assets / "campaign.json").read_text())
    if args.seeds == 1:
        lines, failures = single_seed_report(
            headless, assets, campaign["levels"]
        )
    else:
        lines, failures = multi_seed_report(
            headless, assets, campaign["levels"], args.seeds
        )
    report = "\n".join(lines) + "\n"
    if args.output:
        args.output.write_text(report, encoding="utf-8")
    else:
        print(report, end="")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
