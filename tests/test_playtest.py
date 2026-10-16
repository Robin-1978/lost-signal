#!/usr/bin/env python3

from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from summarize_playtest import read_records, summarize  # noqa: E402


FIELDS = [
    "participant_id",
    "scope",
    "choice_without_help",
    "causal_explanation",
    "retried",
    "recovered_ability",
    "wants_continue",
    "transfer_correct",
    "completion_minutes",
]


class PlaytestSummaryTests(unittest.TestCase):
    def write(self, rows: list[dict[str, str]]) -> Path:
        temporary = tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", newline="", delete=False
        )
        with temporary:
            writer = csv.DictWriter(temporary, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        self.addCleanup(Path(temporary.name).unlink)
        return Path(temporary.name)

    @staticmethod
    def passing_row(index: int, scope: str = "p0") -> dict[str, str]:
        return {
            "participant_id": f"P{index:02d}",
            "scope": scope,
            "choice_without_help": "yes",
            "causal_explanation": "yes",
            "retried": "1",
            "recovered_ability": "can predict a moving forklift",
            "wants_continue": "true",
            "transfer_correct": "pass",
            "completion_minutes": "12",
        }

    def test_complete_p0_sample_passes(self) -> None:
        rows = [self.passing_row(index) for index in range(1, 9)]
        report, passed = summarize(read_records(self.write(rows)))
        self.assertTrue(passed)
        self.assertIn("Emotional motivation: 8/8", report)
        self.assertIn("P0 gate: pass", report)

    def test_incomplete_sample_cannot_pass(self) -> None:
        rows = [self.passing_row(index) for index in range(1, 8)]
        report, passed = summarize(read_records(self.write(rows)))
        self.assertFalse(passed)
        self.assertIn("Valid P0 sample: 7/8", report)

    def test_chapter_pace_is_reported(self) -> None:
        rows = [self.passing_row(index) for index in range(1, 9)]
        for index in range(9, 17):
            row = self.passing_row(index, "chapter")
            row["completion_minutes"] = str(31 + (index % 4))
            rows.append(row)
        report, passed = summarize(read_records(self.write(rows)))
        self.assertTrue(passed)
        self.assertIn("Chapter pace: median", report)


if __name__ == "__main__":
    unittest.main()
