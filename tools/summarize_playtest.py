#!/usr/bin/env python3
"""Summarize the MVP playtest gate from the shared CSV template."""

from __future__ import annotations

import argparse
import csv
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable


YES = {"1", "true", "yes", "y", "是", "通过"}


@dataclass(frozen=True)
class Metric:
    name: str
    threshold: int
    matches: Callable[[dict[str, str]], bool]


METRICS = (
    Metric(
        "情感动机",
        6,
        lambda row: (
            bool(row["recovered_ability"].strip()) and truth(row["wants_continue"])
        ),
    ),
    Metric("首次理解", 6, lambda row: truth(row["choice_without_help"])),
    Metric("因果理解", 6, lambda row: truth(row["causal_explanation"])),
    Metric("主动迭代", 6, lambda row: truth(row["retried"])),
    Metric("教学迁移", 5, lambda row: truth(row["transfer_correct"])),
    Metric("乐趣信号", 6, lambda row: truth(row["wants_continue"])),
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
    lines = ["# 《失落的信号》试玩门槛", "", f"P0 有效样本：{len(p0)}/8", ""]
    p0_passed = len(p0) >= 8
    for metric in METRICS:
        passed = sum(1 for row in p0 if metric.matches(row))
        met = len(p0) >= 8 and passed >= metric.threshold
        p0_passed = p0_passed and met
        lines.append(
            f"- {'通过' if met else '未通过'}　{metric.name}：{passed}/{len(p0)}（门槛 {metric.threshold}/8）"
        )

    p0_times = completion_times(p0)
    if p0_times:
        lines.extend(
            (
                "",
                f"P0 完成时间中位数：{statistics.median(p0_times):.1f} 分钟（仅记录，不作为第一章节奏门槛）",
            )
        )

    lines.extend(("", f"第一章有效样本：{len(chapter)}/8"))
    chapter_times = completion_times(chapter)
    if len(chapter) >= 8 and len(chapter_times) == len(chapter):
        median = statistics.median(chapter_times)
        pace_passed = 30 <= median <= 45
        lines.append(
            f"- {'通过' if pace_passed else '未通过'}　章节节奏：中位数 {median:.1f} 分钟（门槛 30～45 分钟）"
        )
    else:
        lines.append("- 待验证　章节节奏：完整第一章制作后记录 8 名玩家的完成时间")

    lines.extend(("", "P0 阶段门：" + ("通过" if p0_passed else "尚未通过")))
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
