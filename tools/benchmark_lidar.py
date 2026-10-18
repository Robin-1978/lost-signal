#!/usr/bin/env python3
"""Compare planar and 16-line LiDAR throughput in the same headless run."""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import time
from pathlib import Path


SENSORS = ("sensor.standard_lidar", "sensor.sixteen_line_lidar")


def run_once(command: list[str]) -> tuple[float, dict[str, object]]:
    started = time.perf_counter()
    completed = subprocess.run(
        command, check=True, capture_output=True, text=True
    )
    elapsed = time.perf_counter() - started
    return elapsed, json.loads(completed.stdout)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark LiDAR providers with a fixed deterministic scenario."
    )
    parser.add_argument(
        "--executable", default="build/release/lost_signal_headless"
    )
    parser.add_argument("--assets", default="assets")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--duration", type=float, default=20.0)
    arguments = parser.parse_args()
    if arguments.runs <= 0 or arguments.duration <= 0.0:
        parser.error("--runs and --duration must be positive")

    executable = Path(arguments.executable)
    if not executable.is_file():
        parser.error(f"headless executable does not exist: {executable}")

    results: dict[str, object] = {
        "scenario": "first_sight/endless",
        "duration_s": arguments.duration,
        "runs": arguments.runs,
        "profiles": {},
    }
    base = [
        str(executable),
        "--assets",
        arguments.assets,
        "--level",
        "first_sight",
        "--recommended",
        "--component",
        "tires=tires.standard",
        "--endless",
        "--duration",
        str(arguments.duration),
        "--json",
    ]
    for sensor in SENSORS:
        samples = []
        last_run: dict[str, object] = {}
        for _ in range(arguments.runs):
            elapsed, last_run = run_once(
                base + ["--component", f"sensor={sensor}"]
            )
            samples.append(elapsed)
        median_s = statistics.median(samples)
        results["profiles"][sensor] = {
            "median_wall_s": median_s,
            "minimum_wall_s": min(samples),
            "maximum_wall_s": max(samples),
            "realtime_factor": arguments.duration / median_s,
            "samples_s": samples,
            "simulated_time_s": last_run.get("time_s"),
            "status": last_run.get("status"),
        }

    planar = results["profiles"][SENSORS[0]]["median_wall_s"]
    multibeam = results["profiles"][SENSORS[1]]["median_wall_s"]
    results["sixteen_to_planar_wall_ratio"] = multibeam / planar
    print(json.dumps(results, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
