#!/usr/bin/env python3
"""Measure parameter sensitivity under the same command and scenario.

Use this before hardware transfer to learn which parameters most need calibration.
Usage:
  python3 tools/sensitivity.py --level lost_signal --config recommended [--delta 0.1]

The tool copies assets into a temporary directory, perturbs one parameter by
plus or minus delta, and reruns the level's fixed seed. It records status,
time, clearance, odometry error, and energy without changing simulation code.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# (JSON path, key path, label). Perturb values without changing structure.
TARGETS: list[tuple[str, list[str], str]] = [
    ("components/tires.json", ["friction_mu"], "tire friction"),
    ("components/tires.json", ["slip_scale"], "tire slip scale"),
    ("components/tires.json", ["brake_decel_mps2"], "tire braking"),
    ("components/tires.json", ["rolling_resistance"], "rolling resistance"),
    ("components/motors.json", ["stall_force_N"], "motor stall force"),
    ("components/motors.json", ["free_speed_mps"], "motor free speed"),
    ("components/imus.json", ["gyro_noise_radps"], "gyroscope noise"),
    ("components/imus.json", ["gyro_bias_radps"], "gyroscope bias"),
    ("@robot", ["mass_kg"], "vehicle mass"),
    ("@robot", ["yaw_inertia_kgm2"], "yaw inertia"),
]


def run(headless: Path, assets: Path, level: str, config: str, seed: int) -> dict:
    proc = subprocess.run(
        [str(headless), "--assets", str(assets), "--level", level, f"--{config}",
         "--seed", str(seed), "--json"],
        capture_output=True, text=True, check=False)
    return json.loads(proc.stdout)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--level", default="lost_signal")
    parser.add_argument("--config", default="recommended", choices=("default", "recommended"))
    parser.add_argument("--delta", type=float, default=0.1)
    args = parser.parse_args()

    assets = ROOT / "assets"
    headless = ROOT / "build" / "headless" / "lost_signal_headless"
    campaign = json.loads((assets / "campaign.json").read_text())
    if args.level not in campaign["levels"]:
        print(f"unknown level {args.level}", file=sys.stderr)
        return 2
    presentation = None
    for candidate in sorted((assets / "presentation").glob("*.json")):
        doc = json.loads(candidate.read_text())
        if doc.get("level") == args.level:
            presentation = doc
            break
    if presentation is None:
        print(f"no presentation for level {args.level}", file=sys.stderr)
        return 2
    build = presentation["garage"][f"{args.config}_build"]
    level_seed = json.loads((assets / "levels" / f"{args.level}.json").read_text())["seed"]

    base = run(headless, assets, args.level, args.config, level_seed)
    base_row = (base["status"], base["time_s"], base["min_clearance_m"],
                base["odometry_error_m"], base["energy_wh"])
    print(f"baseline {args.level}/{args.config} seed={level_seed}: " + str(base_row))
    print("| Parameter | Direction | Status | Time Δ% | Clearance Δ | Odometry Δ | Energy Δ% | Notes |")
    print("|---|---|---|---|---|---|---|---|")

    # slot -> equipped component id for this build (robots handled separately).
    equipped = {"tires": build.get("tires"), "motors": build.get("motor"), "imus": build.get("imu")}
    robot_id = presentation.get("robot") or campaign["companion"]["robot"]
    robot_rel = "robots/" + robot_id.split(".")[-1] + ".json"
    for relpath, keys, label in TARGETS:
        if relpath == "@robot":
            relpath = robot_rel
        is_robot = relpath.startswith("robots/")
        if is_robot:
            wanted = None
        else:
            slot = {"components/tires.json": "tires", "components/motors.json": "motors",
                    "components/imus.json": "imus"}[relpath]
            wanted = equipped.get(slot)
        for direction in (+1, -1):
            with tempfile.TemporaryDirectory() as tmp:
                scratch = Path(tmp) / "assets"
                shutil.copytree(assets, scratch, symlinks=True)
                doc_path = scratch / relpath
                doc = json.loads(doc_path.read_text())
                if relpath.startswith("robots/"):
                    target = doc["drive"]
                    for key in keys[:-1]:
                        target = target[key]
                    target[keys[-1]] = target[keys[-1]] * (1.0 + direction * args.delta)
                else:
                    hit = False
                    for component in doc.get("components", []):
                        if component["id"] != wanted:
                            continue
                        behavior = component.get("behavior", {})
                        if keys[-1] in behavior:
                            behavior[keys[-1]] = behavior[keys[-1]] * (1.0 + direction * args.delta)
                            hit = True
                    if not hit:
                        print(f"| {label} | {'+' if direction > 0 else '-'} | not equipped | - | - | - | - | |")
                        continue
                doc_path.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n")
                try:
                    trial = run(headless, scratch, args.level, args.config, level_seed)
                except Exception as error:  # noqa: BLE001
                    print(f"| {label} | {'+' if direction > 0 else '-'} | run failed | - | - | - | - | {error} |")
                    continue
                dt = (trial["time_s"] - base["time_s"]) / max(1e-9, abs(base["time_s"])) * 100.0
                dc = trial["min_clearance_m"] - base["min_clearance_m"]
                do = trial["odometry_error_m"] - base["odometry_error_m"]
                de = ((trial["energy_wh"] - base["energy_wh"]) / max(1e-9, abs(base["energy_wh"]))
                      * 100.0)
                flag = " status changed" if trial["status"] != base["status"] else ""
                print(f"| {label} | {'+' if direction > 0 else '-'} | {trial['status']}{flag} "
                      f"| {dt:+.1f} | {dc:+.3f} | {do:+.4f} | {de:+.1f} | |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
