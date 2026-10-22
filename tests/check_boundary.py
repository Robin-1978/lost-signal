#!/usr/bin/env python3
"""Boundary gate: src/robot must never depend on src/sim."""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ROBOT = ROOT / "src" / "robot"
CMAKELISTS = ROOT / "CMakeLists.txt"

failures = []

include_re = re.compile(r'#\s*include\s*[<"]([^>"]+)[>"]')
for path in sorted(ROBOT.rglob("*.hpp")) + sorted(ROBOT.rglob("*.cpp")):
    text = path.read_text(encoding="utf-8", errors="ignore")
    for inc in include_re.findall(text):
        if inc.startswith("sim/") or "/sim/" in inc or inc.startswith("src/sim"):
            failures.append(f"{path.relative_to(ROOT)} includes sim header: {inc}")

text = CMAKELISTS.read_text(encoding="utf-8")
# Every robot lib target must not link a sim lib.
for line in text.splitlines():
    stripped = line.strip()
    if stripped.startswith("target_link_libraries(lost_signal_robot"):
        if "lost_signal_sim_" in stripped:
            failures.append(f"CMake robot target links sim lib: {stripped}")

if failures:
    print("ROBOT BOUNDARY VIOLATIONS:")
    for f in failures:
        print(" -", f)
    sys.exit(1)

print("robot boundary ok: no sim includes, no sim links")
