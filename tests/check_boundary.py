#!/usr/bin/env python3
"""Boundary gate: truth never penetrates ROBOT.

1. src/robot must never include sim/ headers.
2. src/robot must never name SIM truth/eval types (WorldTruth, EvalStep,
   EpisodeRecorder, MissionSnapshot).
3. No lost_signal_robot_* target may link a lost_signal_sim_* library.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ROBOT = ROOT / "src" / "robot"
CMAKELISTS = ROOT / "CMakeLists.txt"

failures = []

include_re = re.compile(r'#\s*include\s*[<"]([^>"]+)[>"]')
truth_tokens = re.compile(
    r"\b(WorldTruth|EvalStep|EpisodeRecorder|EpisodeScore|MissionSnapshot|"
    r"sim_truth|truth_after|truth_before)\b"
)
for path in sorted(ROBOT.rglob("*.hpp")) + sorted(ROBOT.rglob("*.cpp")):
    text = path.read_text(encoding="utf-8", errors="ignore")
    for inc in include_re.findall(text):
        if inc.startswith("sim/") or "/sim/" in inc or inc.startswith("src/sim"):
            failures.append(f"{path.relative_to(ROOT)} includes sim header: {inc}")
    for m in truth_tokens.findall(text):
        failures.append(
            f"{path.relative_to(ROOT)} names SIM truth/eval type: {m}"
        )

text = CMAKELISTS.read_text(encoding="utf-8")
for line in text.splitlines():
    stripped = line.strip()
    if stripped.startswith("target_link_libraries(lost_signal_robot"):
        if "lost_signal_sim_" in stripped:
            failures.append(f"CMake robot target links sim lib: {stripped}")

if failures:
    print("ROBOT BOUNDARY VIOLATIONS (truth isolation):")
    for f in failures:
        print(" -", f)
    sys.exit(1)

print("robot boundary ok: no sim includes, no truth tokens, no sim links")
