#!/usr/bin/env python3
"""Summarize the MVP playtest gate from the shared CSV template."""

from __future__ import annotations

import argparse
import csv
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable


YES = {"1", "true", "yes", "y", "pass"}


@dataclass(frozen=True)
class Metric:
    name: str
    threshold: int
    matches: Callable[[dict[str, str]], bool]


METRICS = (
    Metric(
        "Emotional motivation",
        6,
        lambda row: (
            bool(row["recovered_ability"].strip()) and truth(row["wants_continue"])
        ),
    ),
    Metric("Initial understanding", 6, lambda row: truth(row["choice_without_help"])),
    Metric("Causal understanding", 6, lambda row: truth(row["causal_explanation"])),
    Metric("Active iteration", 6, lambda row: truth(row["retried"])),
    Metric("Learning transfer", 5, lambda row: truth(row["transfer_correct"])),
    Metric("Enjoyment signal", 6, lambda row: truth(row["wants_continue"])),
)


def truth(value: str) -> bool:
    return value.strip().lower() in YES


def read_records(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "participant_id",
            "scope",
            "recovered_ability",
            "wants_continue",
            "choice_without_help",
            "causal_explanation",
            "retried",
            "transfer_correct",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("CSV is missing columns: " + ", ".join(sorted(missing)))
        rows = [
            {key: value or "" for key, value in row.items()}
            for row in reader
            if (row.get("participant_id") or "").strip()
        ]
    identifiers = [row["participant_id"].strip() for row in rows]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("participant_id values must be unique")
    for row in rows:
        if row["scope"].strip().lower() not in {"p0", "chapter"}:
            raise ValueError(
                f"{row['participant_id']}: scope must be 'p0' or 'chapter'"
            )
    return rows


def completion_times(rows: Iterable[dict[str, str]]) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get("completion_minutes", "").strip()
        if not value:
            continue
        try:
            minutes = float(value)
        except ValueError as error:
            raise ValueError(
                f"{row['participant_id']}: invalid completion_minutes '{value}'"
            ) from error
        if minutes <= 0:
            raise ValueError(
                f"{row['participant_id']}: completion_minutes must be positive"
            )
        values.append(minutes)
    return values


def summarize(rows: list[dict[str, str]]) -> tuple[str, bool]:
    p0 = [row for row in rows if row["scope"].strip().lower() == "p0"]
    chapter = [row for row in rows if row["scope"].strip().lower() == "chapter"]
    lines = ["# Lost Signal playtest gate", "", f"Valid P0 sample: {len(p0)}/8", ""]
    p0_passed = len(p0) >= 8
    for metric in METRICS:
        passed = sum(1 for row in p0 if metric.matches(row))
        met = len(p0) >= 8 and passed >= metric.threshold
        p0_passed = p0_passed and met
        lines.append(
            f"- {'Pass' if met else 'Fail'} — {metric.name}: {passed}/{len(p0)} (threshold {metric.threshold}/8)"
        )

    p0_times = completion_times(p0)
    if p0_times:
        lines.extend(
            (
                "",
                f"Median P0 completion: {statistics.median(p0_times):.1f} minutes (reported only; not a chapter gate)",
            )
        )

    lines.extend(("", f"Valid chapter sample: {len(chapter)}/8"))
    chapter_times = completion_times(chapter)
    if len(chapter) >= 8 and len(chapter_times) == len(chapter):
        median = statistics.median(chapter_times)
        pace_passed = 30 <= median <= 45
        lines.append(
            f"- {'Pass' if pace_passed else 'Fail'} — Chapter pace: median {median:.1f} minutes (threshold 30–45)"
        )
    else:
        lines.append(
            "- Pending — Chapter pace requires completion times from 8 players"
        )

    lines.extend(("", "P0 gate: " + ("pass" if p0_passed else "not passed")))
    return "\n".join(lines) + "\n", p0_passed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--require-p0-pass",
        action="store_true",
        help="return a failing exit code until all P0 gates pass",
    )
    args = parser.parse_args()
    try:
        rows = read_records(args.csv_path)
        report, passed = summarize(rows)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(report, end="")
    return 0 if passed or not args.require_p0_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
