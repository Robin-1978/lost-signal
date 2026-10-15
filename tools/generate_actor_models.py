#!/usr/bin/env python3
"""Generate the Apache-2.0 low-poly actor kit: forklift + road worker.

Procedural boxes used to stand in for both (draw_forklift/draw_pedestrian);
these are real models on the same Part/box/cylinder pipeline as the robots.
Reference geometry: forklift authored for radius 0.45, worker for 0.25 —
the renderer scales linearly from the actor's own radius (content data).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from generate_robot_models import Part, rgba, write_model

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "assets" / "models" / "actors"

ROT_X_90 = (math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0))


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    forklift = [
        Part("chassis", "box", (0.95, 0.52, 0.18), rgba("#2A2E2C"), (0, 0, 0.12)),
        Part("body", "box", (1.12, 0.58, 0.55), rgba("#D38427"), (0, 0, 0.42), metallic=0.30),
        Part("stripe", "box", (0.50, 0.59, 0.06), rgba("#61D6C2"), (0, 0, 0.62), emissive=0.55),
        Part("cab", "box", (0.46, 0.46, 0.55), rgba("#30373B"), (-0.20, 0, 0.85)),
        Part("cab_roof", "box", (0.52, 0.52, 0.06), rgba("#20242A"), (-0.20, 0, 1.15)),
        Part("mast_left", "box", (0.08, 0.08, 0.85), rgba("#3A4046"), (0.52, 0.20, 0.55)),
        Part("mast_right", "box", (0.08, 0.08, 0.85), rgba("#3A4046"), (0.52, -0.20, 0.55)),
        Part("fork_left", "box", (0.75, 0.09, 0.06), rgba("#5C665F"), (0.80, 0.20, 0.08),
             metallic=0.60),
        Part("fork_right", "box", (0.75, 0.09, 0.06), rgba("#5C665F"), (0.80, -0.20, 0.08),
             metallic=0.60),
    ]
    for x in (-0.33, 0.33):
        for y in (-0.30, 0.30):
            forklift.append(
                Part(f"wheel_{x:+.2f}_{y:+.2f}", "cylinder", (0.14, 0.14, 0.10),
                     rgba("#17191B"), (x, y, 0.14), rotation=ROT_X_90, roughness=0.90)
            )
    write_model("forklift", forklift, OUTPUT)

    worker = [
        Part("leg_left", "box", (0.11, 0.11, 0.50), rgba("#242E2C"), (0, 0.07, 0.25)),
        Part("leg_right", "box", (0.11, 0.11, 0.50), rgba("#242E2C"), (0, -0.07, 0.25)),
        Part("torso", "box", (0.30, 0.20, 0.52), rgba("#E87A1F"), (0, 0, 0.72)),
        Part("band", "box", (0.32, 0.22, 0.10), rgba("#F0C85A"), (0, 0, 0.78), emissive=0.35),
        Part("arm_left", "box", (0.09, 0.09, 0.44), rgba("#C86A1E"), (0.02, 0.17, 0.72)),
        Part("arm_right", "box", (0.09, 0.09, 0.44), rgba("#C86A1E"), (0.02, -0.17, 0.72)),
        Part("head", "box", (0.17, 0.16, 0.20), rgba("#EBD4C2"), (0, 0, 1.12)),
        Part("cap", "box", (0.19, 0.18, 0.07), rgba("#17A090"), (0, 0, 1.25)),
    ]
    write_model("worker", worker, OUTPUT)
    print(f"actor kit written to {OUTPUT}")


if __name__ == "__main__":
    main()
