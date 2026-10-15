#!/usr/bin/env python3
"""悬赏板独立覆盖：生成器确定性＋关系型 pin（引用必解、无绝对数硬编码）。"""

from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from generate_contracts import AXES, JOBS_PER_WEEK, generate_board, render  # noqa: E402
from validate_content import load_and_validate  # noqa: E402


class ContractBoardTests(unittest.TestCase):
    def test_regeneration_is_byte_identical(self) -> None:
        first = render(generate_board(week=1, seed=20261015))
        second = render(generate_board(week=1, seed=20261015))
        self.assertEqual(first, second)
        checked_in = (ROOT / "assets" / "contracts" / "weekly_01.json").read_text(encoding="utf-8")
        self.assertEqual(first, checked_in)

    def test_check_mode_passes(self) -> None:
        proc = subprocess.run(
            [sys.executable, str(ROOT / "tools" / "generate_contracts.py"),
             "--week", "1", "--check"],
            capture_output=True, text=True, check=False,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_board_pins_are_relational(self) -> None:
        catalog = load_and_validate(ROOT / "assets")
        campaign_levels = set(catalog["campaign"]["levels"])
        boards = {tuple(sorted(j["level"] for j in b["jobs"])): b for b in catalog["contracts"]}
        self.assertTrue(boards)
        for board in catalog["contracts"]:
            jobs = board["jobs"]
            self.assertEqual(len(jobs), JOBS_PER_WEEK)
            # Every job rides an existing campaign level, no duplicates.
            levels = [job["level"] for job in jobs]
            self.assertEqual(len(set(levels)), len(levels))
            for job in jobs:
                self.assertIn(job["level"], campaign_levels)
                self.assertGreaterEqual(job["rep"], 1)
                self.assertTrue(job["blurb"])
            # One board serves every score axis (cargo x score coverage).
            self.assertEqual({job["axis"] for job in jobs}, set(AXES))
            # Tutorial levels never take jobs (order<=2 excluded from pool).
            for job in jobs:
                self.assertGreater(catalog["levels"][job["level"]]["order"], 2)

    def test_weeks_rotate(self) -> None:
        first = [job["level"] for job in generate_board(week=1, seed=20261015)["jobs"]]
        second = [job["level"] for job in generate_board(week=2, seed=20261015)["jobs"]]
        self.assertNotEqual(first, second)


if __name__ == "__main__":
    unittest.main()
