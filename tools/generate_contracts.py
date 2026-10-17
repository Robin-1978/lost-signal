#!/usr/bin/env python3
"""Generate a deterministic weekly contract board from a seed.

This content-only tool rotates existing missions by cargo and scoring axis.
Reputation rewards scale with mission order. ``--check`` compares generated
bytes with the committed files so CI can enforce reproducibility.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
CONTRACTS = ASSETS / "contracts"

CARGO = ["grain", "mail", "parts", "medkit", "water"]
CARGO_NAME = {
    "grain": " grain（种子粮）",
    "mail": " mail（镇民邮件）",
    "parts": " parts（机器零件）",
    "medkit": " medkit（医疗包）",
    "water": " water（净水）",
}
CARGO_PHYSICS = {
 "grain": {"mass_kg": 14.0, "cog_height_m": 0.48, "fragility": 0.35},
 "mail": {"mass_kg": 4.0, "cog_height_m": 0.38, "fragility": 0.60},
 "parts": {"mass_kg": 22.0, "cog_height_m": 0.52, "fragility": 0.20},
 "medkit": {"mass_kg": 8.0, "cog_height_m": 0.72, "fragility": 0.90},
 "water": {"mass_kg": 18.0, "cog_height_m": 0.62, "fragility": 0.75},
}
AXES = ["fast", "efficient", "safe"]
AXIS_BLURB = {
    "fast": "镇上等着用，越快越好——但别撞，撞了这单 reputation 照扣。",
    "efficient": "电要省着花，镇发电站的老王只认能耗账。",
    "safe": "货怕晃，净空就是一切，慢就是快。",
}
JOBS_PER_WEEK = 5


def job_pool() -> list[dict]:
    """Return Guided Campaign contract candidates, excluding the tutorial."""
    campaign = json.loads((ASSETS / "campaign.json").read_text(encoding="utf-8"))
    guided_levels = set(campaign["levels"])
    pool = []
    for path in sorted((ASSETS / "levels").glob("*.json")):
        level = json.loads(path.read_text(encoding="utf-8"))
        if level["id"] in guided_levels and level.get("order", 0) > 2:
            pool.append({"id": level["id"], "order": level["order"]})
    return pool


def rep_for_order(order: int) -> int:
    """Scale reputation by mission order, capped by the current content set."""
    return 1 + (order - 1) // 6


def generate_board(week: int, seed: int) -> dict:
    rng = random.Random(seed ^ (week * 0x9E3779B1))
    pool = job_pool()
    order = list(range(len(pool)))
    rng.shuffle(order)
    jobs = []
    for slot in range(JOBS_PER_WEEK):
        level = pool[order[slot % len(order)]]
        axis = AXES[(slot + week) % len(AXES)]
        cargo = CARGO[(slot * 2 + week) % len(CARGO)]
        jobs.append(
            {
                "id": f"w{week:02d}-{slot + 1}",
                "level": level["id"],
                "cargo": cargo,
                "axis": axis,
                "rep": rep_for_order(level["order"]),
 **CARGO_PHYSICS[cargo],
                "blurb": f"把{CARGO_NAME[cargo]}送过{level['id']}。{AXIS_BLURB[axis]}",
            }
        )
    return {"schema_version": 1, "week": week, "seed": seed, "jobs": jobs}


def board_path(week: int) -> Path:
    return CONTRACTS / f"weekly_{week:02d}.json"


def render(board: dict) -> str:
    return json.dumps(board, ensure_ascii=False, indent=2) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--week", type=int, default=1)
    parser.add_argument("--seed", type=int, default=20261015)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    board = generate_board(args.week, args.seed)
    text = render(board)
    if args.check:
        current = board_path(args.week).read_text(encoding="utf-8")
        if current != text:
            print(f"contracts out of date for week {args.week}: regenerate", file=sys.stderr)
            return 1
        print(f"contracts week {args.week:02d} match")
        return 0
    CONTRACTS.mkdir(exist_ok=True)
    board_path(args.week).write_text(text, encoding="utf-8")
    print(f"wrote {board_path(args.week)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
