#!/usr/bin/env python3
"""批量平衡报告：全关卡 × 官方配置，一次跑完输出 Markdown 表格。

调平衡看表调参，不进游戏。用法：
  python3 tools/level_report.py [--assets assets] [--seeds K]

--seeds 1（默认）：今天的行为，关卡 JSON 种子单跑，表格逐行输出。
--seeds K>1：种子 level.seed+0..K-1 各跑一次，输出汇总行（成功率、
均值/极值），并把首种子重跑一遍做确定性自检（同一种子必须逐字节一致）。
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def run(headless: Path, assets: Path, level: str, config: str, seed: int | None) -> dict:
    cmd = [str(headless), "--assets", str(assets), "--level", level, f"--{config}", "--json"]
    if seed is not None:
        cmd += ["--seed", str(seed)]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return json.loads(proc.stdout)


def badge_of(d: dict) -> str:
    scores = d["scores"]
    if d["status"] == "succeeded" and all(scores.values()):
        return "✓"
    if d["status"] == "succeeded":
        return "○"
    return "✗"


def single_row(level: str, config: str, d: dict) -> str:
    scores = d["scores"]
    return (
        f"| {level} | {config} | {badge_of(d)} {d['status']} {d.get('failure_reason', '')} "
        f"| {d['time_s']:.2f} | {d['min_clearance_m']:.3f} | "
        f"{d['odometry_error_m']:.4f} | "
        f"{int(scores['safe'])}/{int(scores['fast'])}/{int(scores['efficient'])} |")


def multi_seed_summary(headless: Path, assets: Path, levels: list[str], seeds: int) -> int:
    print(f"多种子模式：每组 {seeds} 种子（level.seed+0..{seeds - 1}），先做确定性自检。")
    print("| 关卡 | 配置 | 成功率 | 用时均值[最小-最大] | 净空最小 | 里程计最大 | 安全分 | 确定性 |")
    print("|---|---|---|---|---|---|---|---|")
    failures = 0
    for level in levels:
        base_seed = json.loads((assets / "levels" / f"{level}.json").read_text())["seed"]
        for config in ("default", "recommended"):
            try:
                trials = [run(headless, assets, level, config, base_seed + i) for i in range(seeds)]
                repeat = run(headless, assets, level, config, base_seed)
            except Exception as error:  # noqa: BLE001
                print(f"| {level} | {config} | 运行失败 | - | - | - | - | - | {error} |")
                failures += 1
                continue
            deterministic = json.dumps(trials[0], sort_keys=True) == json.dumps(repeat, sort_keys=True)
            ok = sum(1 for d in trials if d["status"] == "succeeded")
            times = [d["time_s"] for d in trials]
            clearance = min(d["min_clearance_m"] for d in trials)
            odom = max(d["odometry_error_m"] for d in trials)
            safe = sum(1 for d in trials if d["scores"]["safe"])
            print(f"| {level} | {config} | {ok}/{seeds} "
                  f"| {statistics.fmean(times):.2f}[{min(times):.2f}-{max(times):.2f}] "
                  f"| {clearance:.3f} | {odom:.4f} | {safe}/{seeds} "
                  f"| {'✓' if deterministic else '✗ 非确定！'} |")
            if not deterministic:
                failures += 1
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", default="assets")
    parser.add_argument("--seeds", type=int, default=1)
    args = parser.parse_args()
    if args.seeds < 1:
        print("--seeds 至少为 1", file=sys.stderr)
        return 2
    assets = (ROOT / args.assets).resolve()
    headless = ROOT / "build" / "headless" / "lost_signal_headless"
    campaign = json.loads((assets / "campaign.json").read_text())

    if args.seeds > 1:
        return multi_seed_summary(headless, assets, campaign["levels"], args.seeds)

    print("| 关卡 | 配置 | 结果 | 用时 | 净空 | 里程计误差 | 安全/迅速/节能 |")
    print("|---|---|---|---|---|---|---|")
    for level in campaign["levels"]:
        for config in ("default", "recommended"):
            try:
                d = run(headless, assets, level, config, None)
            except Exception as error:  # noqa: BLE001
                print(f"| {level} | {config} | 运行失败 | - | - | - | {error} |")
                continue
            print(single_row(level, config, d))
    return 0


if __name__ == "__main__":
    sys.exit(main())
