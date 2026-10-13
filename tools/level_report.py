#!/usr/bin/env python3
"""批量平衡报告：全关卡 × 官方配置，一次跑完输出 Markdown 表格。

调平衡看表调参，不进游戏。用法：
  python3 tools/level_report.py [--assets assets]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def run(headless: Path, assets: Path, level: str, config: str) -> dict:
    proc = subprocess.run(
        [str(headless), "--assets", str(assets), "--level", level, f"--{config}", "--json"],
        capture_output=True, text=True, check=False)
    return json.loads(proc.stdout)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", default="assets")
    args = parser.parse_args()
    assets = (ROOT / args.assets).resolve()
    headless = ROOT / "build" / "headless" / "lost_signal_headless"
    campaign = json.loads((assets / "campaign.json").read_text())

    print("| 关卡 | 配置 | 结果 | 用时 | 净空 | 里程计误差 | 安全/迅速/节能 |")
    print("|---|---|---|---|---|---|---|")
    for level in campaign["levels"]:
        for config in ("default", "recommended"):
            try:
                d = run(headless, assets, level, config)
            except Exception as error:  # noqa: BLE001
                print(f"| {level} | {config} | 运行失败 | - | - | - | {error} |")
                continue
            scores = d["scores"]
            badge = (
                "✓" if d["status"] == "succeeded" and all(scores.values())
                else "○" if d["status"] == "succeeded" else "✗")
            print(f"| {level} | {config} | {badge} {d['status']} {d.get('failure_reason', '')} "
                  f"| {d['time_s']:.2f} | {d['min_clearance_m']:.3f} | "
                  f"{d['odometry_error_m']:.4f} | "
                  f"{int(scores['safe'])}/{int(scores['fast'])}/{int(scores['efficient'])} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
