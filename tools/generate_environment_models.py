#!/usr/bin/env python3
"""Generate the Apache-2.0 low-poly environment kit used by the research base."""

from __future__ import annotations

import math
from pathlib import Path

from generate_robot_models import Part, rgba, write_model

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "assets" / "models" / "environment" / "research_base"


def main() -> None:
    turn_x = (math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0))
    tilt_back = (-math.sin(math.radians(8.0)), 0.0, 0.0, math.cos(math.radians(8.0)))
    wall = [
        Part(
            "panel",
            "box",
            (3.45, 0.24, 1.0),
            rgba("#2B302D"),
            (0, 0, 0.50),
            metallic=0.35,
        ),
        Part(
            "inset", "box", (2.72, 0.035, 0.60), rgba("#202724"), (0.18, -0.135, 0.52)
        ),
        Part(
            "left_frame",
            "box",
            (0.16, 0.34, 1.16),
            rgba("#494B43"),
            (-1.61, 0, 0.58),
            metallic=0.55,
        ),
        Part(
            "right_frame",
            "box",
            (0.10, 0.30, 1.05),
            rgba("#3D403A"),
            (1.62, 0, 0.53),
            metallic=0.50,
        ),
        Part(
            "top_frame",
            "box",
            (3.20, 0.30, 0.10),
            rgba("#454940"),
            (0, 0, 1.02),
            metallic=0.50,
        ),
        Part(
            "light_strip",
            "box",
            (1.05, 0.045, 0.035),
            rgba("#61D6C2"),
            (0.60, -0.16, 0.93),
            metallic=0.10,
            emissive=0.72,
        ),
        Part(
            "warning",
            "box",
            (0.30, 0.045, 0.16),
            rgba("#D2943D"),
            (-1.10, -0.16, 0.78),
            metallic=0.12,
        ),
    ]
    for index in range(6):
        wall.append(
            Part(
                f"vent_{index}",
                "box",
                (0.62, 0.040, 0.035),
                rgba("#56605A"),
                (0.72, -0.16, 0.68 - index * 0.075),
                metallic=0.44,
            )
        )

    rubble = [
        Part(
            "slab_a",
            "box",
            (0.54, 0.38, 0.26),
            rgba("#515249"),
            (-0.20, 0.02, 0.13),
            roughness=0.90,
        ),
        Part(
            "slab_b",
            "box",
            (0.38, 0.46, 0.22),
            rgba("#656258"),
            (0.18, -0.08, 0.11),
            roughness=0.92,
        ),
        Part(
            "slab_c",
            "box",
            (0.30, 0.28, 0.16),
            rgba("#454942"),
            (0.06, 0.24, 0.08),
            roughness=0.88,
        ),
        Part(
            "rebar",
            "cylinder",
            (0.018, 0.018, 0.65),
            rgba("#6D5847"),
            (0.0, 0.0, 0.22),
            turn_x,
            metallic=0.70,
        ),
    ]

    plant = [
        Part(
            "stem",
            "cylinder",
            (0.035, 0.035, 0.50),
            rgba("#315D38"),
            (0, 0, 0.25),
            roughness=0.82,
        ),
        Part(
            "leaf_a",
            "box",
            (0.08, 0.36, 0.16),
            rgba("#3D7847"),
            (-0.10, 0.04, 0.43),
            roughness=0.88,
        ),
        Part(
            "leaf_b",
            "box",
            (0.36, 0.08, 0.14),
            rgba("#2F653B"),
            (0.08, -0.02, 0.52),
            roughness=0.88,
        ),
        Part(
            "leaf_c",
            "box",
            (0.07, 0.30, 0.13),
            rgba("#4A8650"),
            (0.06, 0.06, 0.62),
            roughness=0.88,
        ),
        Part(
            "root_light",
            "cylinder",
            (0.09, 0.09, 0.045),
            rgba("#23362D"),
            (0, 0, 0.025),
            metallic=0.25,
        ),
    ]

    service_light = [
        Part(
            "foot",
            "cylinder",
            (0.12, 0.12, 0.06),
            rgba("#3D443F"),
            (0, 0, 0.03),
            metallic=0.50,
        ),
        Part(
            "pole",
            "cylinder",
            (0.035, 0.035, 0.55),
            rgba("#697069"),
            (0, 0, 0.30),
            metallic=0.70,
        ),
        Part(
            "lamp_ring",
            "cylinder",
            (0.105, 0.105, 0.065),
            rgba("#263B37"),
            (0, 0, 0.60),
            metallic=0.40,
        ),
        Part(
            "lamp",
            "cylinder",
            (0.075, 0.075, 0.08),
            rgba("#72E4CF"),
            (0, 0, 0.66),
            metallic=0.05,
            roughness=0.25,
            emissive=0.90,
        ),
    ]

    control_console = [
        Part(
            "plinth",
            "box",
            (0.82, 0.46, 0.10),
            rgba("#252B29"),
            (0, 0.0, 0.05),
            metallic=0.45,
        ),
        Part(
            "cabinet",
            "box",
            (0.72, 0.38, 0.70),
            rgba("#3A403B"),
            (0, 0.02, 0.42),
            metallic=0.42,
            roughness=0.64,
        ),
        Part(
            "front_inset",
            "box",
            (0.56, 0.025, 0.42),
            rgba("#202825"),
            (0, -0.205, 0.43),
            metallic=0.25,
        ),
        Part(
            "screen_frame",
            "box",
            (0.66, 0.12, 0.43),
            rgba("#555C55"),
            (0, -0.08, 0.88),
            tilt_back,
            metallic=0.58,
        ),
        Part(
            "screen",
            "box",
            (0.52, 0.018, 0.29),
            rgba("#173D38"),
            (0, -0.151, 0.91),
            tilt_back,
            metallic=0.10,
            roughness=0.24,
            emissive=0.30,
        ),
        Part(
            "signal_line",
            "box",
            (0.36, 0.010, 0.025),
            rgba("#62E0C7"),
            (0.04, -0.164, 0.94),
            tilt_back,
            emissive=0.88,
        ),
        Part(
            "warning_light",
            "cylinder",
            (0.035, 0.035, 0.022),
            rgba("#E4A13D"),
            (-0.22, -0.215, 0.63),
            turn_x,
            emissive=0.72,
        ),
        Part(
            "dead_light",
            "cylinder",
            (0.035, 0.035, 0.022),
            rgba("#722F2B"),
            (-0.11, -0.215, 0.63),
            turn_x,
            roughness=0.44,
        ),
        Part(
            "broken_cover",
            "box",
            (0.22, 0.025, 0.30),
            rgba("#51574F"),
            (0.32, -0.27, 0.31),
            rotation=(
                0.0,
                math.sin(math.radians(11.0)),
                0.0,
                math.cos(math.radians(11.0)),
            ),
            metallic=0.46,
        ),
    ]

    cable_reel = [
        Part(
            "left_foot",
            "box",
            (0.16, 0.58, 0.08),
            rgba("#323936"),
            (-0.22, 0, 0.04),
            metallic=0.46,
        ),
        Part(
            "right_foot",
            "box",
            (0.16, 0.58, 0.08),
            rgba("#323936"),
            (0.22, 0, 0.04),
            metallic=0.46,
        ),
        Part(
            "left_flange",
            "cylinder",
            (0.34, 0.34, 0.10),
            rgba("#B67A2A"),
            (0, -0.20, 0.38),
            turn_x,
            metallic=0.38,
            roughness=0.63,
        ),
        Part(
            "right_flange",
            "cylinder",
            (0.34, 0.34, 0.10),
            rgba("#B67A2A"),
            (0, 0.20, 0.38),
            turn_x,
            metallic=0.38,
            roughness=0.63,
        ),
        Part(
            "cable_core",
            "cylinder",
            (0.23, 0.23, 0.44),
            rgba("#181D1C"),
            (0, 0, 0.38),
            turn_x,
            roughness=0.94,
        ),
        Part(
            "hub",
            "cylinder",
            (0.085, 0.085, 0.50),
            rgba("#67706A"),
            (0, 0, 0.38),
            turn_x,
            metallic=0.68,
        ),
        Part(
            "loose_cable_a",
            "box",
            (0.48, 0.035, 0.035),
            rgba("#171B1A"),
            (0.34, -0.17, 0.04),
            roughness=0.96,
        ),
        Part(
            "loose_cable_b",
            "box",
            (0.035, 0.42, 0.035),
            rgba("#171B1A"),
            (0.56, 0.02, 0.04),
            roughness=0.96,
        ),
        Part(
            "plug",
            "box",
            (0.12, 0.08, 0.07),
            rgba("#4E5A55"),
            (0.56, 0.25, 0.06),
            metallic=0.30,
        ),
    ]

    maintenance_crates = [
        Part(
            "lower_crate",
            "box",
            (0.92, 0.62, 0.46),
            rgba("#48504A"),
            (0, 0, 0.23),
            metallic=0.35,
            roughness=0.70,
        ),
        Part(
            "lower_band_a",
            "box",
            (0.12, 0.64, 0.48),
            rgba("#767768"),
            (-0.31, 0, 0.24),
            metallic=0.52,
        ),
        Part(
            "lower_band_b",
            "box",
            (0.12, 0.64, 0.48),
            rgba("#767768"),
            (0.31, 0, 0.24),
            metallic=0.52,
        ),
        Part(
            "upper_crate",
            "box",
            (0.64, 0.52, 0.38),
            rgba("#5D5945"),
            (0.10, 0.02, 0.66),
            metallic=0.28,
            roughness=0.74,
        ),
        Part(
            "upper_lid",
            "box",
            (0.68, 0.56, 0.07),
            rgba("#89805D"),
            (0.10, 0.02, 0.87),
            metallic=0.35,
        ),
        Part(
            "status_panel",
            "box",
            (0.22, 0.025, 0.12),
            rgba("#1E2A27"),
            (0.10, -0.291, 0.69),
            metallic=0.22,
        ),
        Part(
            "status_light",
            "box",
            (0.12, 0.012, 0.022),
            rgba("#58D8BE"),
            (0.10, -0.307, 0.70),
            emissive=0.84,
        ),
        Part(
            "loose_case",
            "box",
            (0.38, 0.30, 0.18),
            rgba("#343C38"),
            (-0.52, 0.17, 0.09),
            metallic=0.32,
        ),
    ]

    warning_strip = [
        Part(
            "backing",
            "box",
            (2.70, 0.38, 0.012),
            rgba("#222725"),
            (0, 0, 0.006),
            roughness=0.90,
        )
    ]
    for index in range(9):
        warning_strip.append(
            Part(
                f"amber_{index}",
                "box",
                (0.21, 0.34, 0.010),
                rgba("#C78B31" if index % 3 else "#A8742B"),
                (-1.16 + index * 0.29, 0, 0.017),
                roughness=0.84,
            )
        )
    for index, offset in enumerate((-0.88, 0.26, 0.98)):
        warning_strip.append(
            Part(
                f"wear_{index}",
                "box",
                (0.10, 0.16, 0.006),
                rgba("#4E4938"),
                (offset, 0.04 * (index - 1), 0.024),
                roughness=0.96,
            )
        )

    for name, parts in {
        "wall_panel": wall,
        "rubble_cluster": rubble,
        "plant_cluster": plant,
        "service_light": service_light,
        "control_console": control_console,
        "cable_reel": cable_reel,
        "maintenance_crates": maintenance_crates,
        "warning_strip": warning_strip,
    }.items():
        write_model(name, parts, OUTPUT)


if __name__ == "__main__":
    main()
