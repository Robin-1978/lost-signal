#!/usr/bin/env python3
"""Generate the Apache-2.0 low-poly GLB parts shipped with the P0 robot."""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "assets" / "models" / "robots" / "ro1"
Vector3 = tuple[float, float, float]
Mesh = tuple[list[Vector3], list[Vector3], list[int]]


@dataclass(frozen=True)
class Part:
    name: str
    mesh: str
    size: tuple[float, float, float]
    color: tuple[float, float, float, float]
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float, float] | None = None
    metallic: float = 0.25
    roughness: float = 0.58
    emissive: float = 0.0


def box_mesh(size: tuple[float, float, float]) -> Mesh:
    sx, sy, sz = (value / 2 for value in size)
    # Faces use independent vertices so their hard edges retain flat normals.
    faces: list[tuple[Vector3, tuple[Vector3, Vector3, Vector3, Vector3]]] = [
        (
            (0.0, 0.0, -1.0),
            ((-sx, -sy, -sz), (-sx, sy, -sz), (sx, sy, -sz), (sx, -sy, -sz)),
        ),
        ((0.0, 0.0, 1.0), ((-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz))),
        (
            (0.0, -1.0, 0.0),
            ((-sx, -sy, -sz), (sx, -sy, -sz), (sx, -sy, sz), (-sx, -sy, sz)),
        ),
        ((1.0, 0.0, 0.0), ((sx, -sy, -sz), (sx, sy, -sz), (sx, sy, sz), (sx, -sy, sz))),
        ((0.0, 1.0, 0.0), ((sx, sy, -sz), (-sx, sy, -sz), (-sx, sy, sz), (sx, sy, sz))),
        (
            (-1.0, 0.0, 0.0),
            ((-sx, sy, -sz), (-sx, -sy, -sz), (-sx, -sy, sz), (-sx, sy, sz)),
        ),
    ]
    positions: list[Vector3] = []
    normals: list[Vector3] = []
    indices: list[int] = []
    for normal, vertices in faces:
        base = len(positions)
        positions.extend(vertices)
        normals.extend([normal] * 4)
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3])
    return positions, normals, indices


def cylinder_mesh(size: tuple[float, float, float], sides: int = 24) -> Mesh:
    radius_x, radius_y, height = size
    half = height * 0.5
    positions: list[Vector3] = []
    normals: list[Vector3] = []
    for z in (-half, half):
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            cosine, sine = math.cos(angle), math.sin(angle)
            positions.append((cosine * radius_x, sine * radius_y, z))
            length = math.hypot(cosine / radius_x, sine / radius_y)
            normals.append((cosine / radius_x / length, sine / radius_y / length, 0.0))

    # Cap vertices are duplicated so the side remains smooth while each cap stays flat.
    bottom_ring = len(positions)
    for index in range(sides):
        angle = 2.0 * math.pi * index / sides
        positions.append(
            (math.cos(angle) * radius_x, math.sin(angle) * radius_y, -half)
        )
        normals.append((0.0, 0.0, -1.0))
    top_ring = len(positions)
    for index in range(sides):
        angle = 2.0 * math.pi * index / sides
        positions.append((math.cos(angle) * radius_x, math.sin(angle) * radius_y, half))
        normals.append((0.0, 0.0, 1.0))
    positions.extend([(0.0, 0.0, -half), (0.0, 0.0, half)])
    normals.extend([(0.0, 0.0, -1.0), (0.0, 0.0, 1.0)])
    bottom_center = len(positions) - 2
    top_center = bottom_center + 1
    indices: list[int] = []
    for index in range(sides):
        following = (index + 1) % sides
        indices.extend(
            [
                index,
                following,
                sides + following,
                index,
                sides + following,
                sides + index,
            ]
        )
        indices.extend([bottom_center, bottom_ring + following, bottom_ring + index])
        indices.extend([top_center, top_ring + index, top_ring + following])
    return positions, normals, indices


def write_model(name: str, parts: list[Part], output: Path = OUTPUT) -> None:
    mesh_builders: dict[str, Callable[[tuple[float, float, float]], Mesh]] = {
        "box": box_mesh,
        "cylinder": cylinder_mesh,
    }
    binary = bytearray()
    accessors: list[dict] = []
    buffer_views: list[dict] = []
    meshes: list[dict] = []
    materials: list[dict] = []
    nodes: list[dict] = []

    for index, part in enumerate(parts):
        positions, normals, indices = mesh_builders[part.mesh](part.size)
        while len(binary) % 4:
            binary.append(0)
        position_offset = len(binary)
        binary.extend(b"".join(struct.pack("<3f", *position) for position in positions))
        position_length = len(binary) - position_offset
        while len(binary) % 4:
            binary.append(0)
        normal_offset = len(binary)
        binary.extend(b"".join(struct.pack("<3f", *normal) for normal in normals))
        normal_length = len(binary) - normal_offset
        while len(binary) % 4:
            binary.append(0)
        index_offset = len(binary)
        binary.extend(struct.pack(f"<{len(indices)}H", *indices))
        index_length = len(indices) * 2

        position_view = len(buffer_views)
        buffer_views.append(
            {
                "buffer": 0,
                "byteOffset": position_offset,
                "byteLength": position_length,
                "target": 34962,
            }
        )
        normal_view = len(buffer_views)
        buffer_views.append(
            {
                "buffer": 0,
                "byteOffset": normal_offset,
                "byteLength": normal_length,
                "target": 34962,
            }
        )
        index_view = len(buffer_views)
        buffer_views.append(
            {
                "buffer": 0,
                "byteOffset": index_offset,
                "byteLength": index_length,
                "target": 34963,
            }
        )
        minimum = [min(position[axis] for position in positions) for axis in range(3)]
        maximum = [max(position[axis] for position in positions) for axis in range(3)]
        position_accessor = len(accessors)
        accessors.append(
            {
                "bufferView": position_view,
                "componentType": 5126,
                "count": len(positions),
                "type": "VEC3",
                "min": minimum,
                "max": maximum,
            }
        )
        normal_accessor = len(accessors)
        accessors.append(
            {
                "bufferView": normal_view,
                "componentType": 5126,
                "count": len(normals),
                "type": "VEC3",
            }
        )
        index_accessor = len(accessors)
        accessors.append(
            {
                "bufferView": index_view,
                "componentType": 5123,
                "count": len(indices),
                "type": "SCALAR",
            }
        )
        materials.append(
            {
                "name": f"{part.name}_material",
                "pbrMetallicRoughness": {
                    "baseColorFactor": part.color,
                    "metallicFactor": part.metallic,
                    "roughnessFactor": part.roughness,
                },
                "emissiveFactor": [
                    channel * part.emissive for channel in part.color[:3]
                ],
            }
        )
        meshes.append(
            {
                "name": part.name,
                "primitives": [
                    {
                        "attributes": {
                            "POSITION": position_accessor,
                            "NORMAL": normal_accessor,
                        },
                        "indices": index_accessor,
                        "material": index,
                    }
                ],
            }
        )
        # Generated robot assets use the repository's robotics frame (+x forward,
        # +y left, +z up). The renderer owns the single robotics-to-raylib basis
        # change when it loads a model. Keeping that boundary here avoids baking a
        # second conversion into vertices and turning the assembled chassis over.
        node = {"mesh": index, "name": part.name, "translation": part.position}
        if part.rotation is not None:
            node["rotation"] = part.rotation
        nodes.append(node)

    while len(binary) % 4:
        binary.append(0)
    document = {
        "asset": {
            "version": "2.0",
            "generator": "Lost Signal low-poly model generator",
        },
        "scene": 0,
        "scenes": [{"nodes": list(range(len(nodes)))}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(binary)}],
    }
    encoded = json.dumps(document, ensure_ascii=True, separators=(",", ":")).encode()
    while len(encoded) % 4:
        encoded += b" "
    total_length = 12 + 8 + len(encoded) + 8 + len(binary)
    payload = struct.pack("<III", 0x46546C67, 2, total_length)
    payload += struct.pack("<II", len(encoded), 0x4E4F534A) + encoded
    payload += struct.pack("<II", len(binary), 0x004E4942) + binary
    output.mkdir(parents=True, exist_ok=True)
    (output / f"{name}.glb").write_bytes(payload)


def rgba(hex_value: str) -> tuple[float, float, float, float]:
    value = hex_value.removeprefix("#")
    channels = tuple(int(value[index : index + 2], 16) / 255.0 for index in (0, 2, 4))
    return channels + (1.0,)


def main() -> None:
    turn_x = (math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0))
    white, dark, mint = rgba("#D8E8E3"), rgba("#172321"), rgba("#50E1C5")
    standard_wheel = [
        Part(
            "tire",
            "cylinder",
   (0.10, 0.10, 0.10),
            rgba("#263331"),
            roughness=0.84,
        ),
        Part(
            "rim",
            "cylinder",
            (0.066, 0.066, 0.106),
            rgba("#6F837E"),
            metallic=0.62,
            roughness=0.32,
        ),
        Part(
            "hub",
            "cylinder",
            (0.031, 0.031, 0.112),
            mint,
            metallic=0.46,
            emissive=0.12,
        ),
    ]
    grip_wheel = [
        Part(
            "tire",
            "cylinder",
   (0.09, 0.09, 0.12),
            dark,
            roughness=0.94,
        ),
        Part(
            "rim",
            "cylinder",
            (0.061, 0.061, 0.125),
            rgba("#64736F"),
            metallic=0.54,
        ),
        Part(
            "hub",
            "cylinder",
            (0.036, 0.036, 0.132),
            mint,
            metallic=0.42,
            emissive=0.18,
        ),
    ]
    for tread in range(12):
        angle = 2.0 * math.pi * tread / 12.0
        grip_wheel.append(
            Part(
                f"tread_{tread:02d}",
                "box",
 (0.024, 0.026, 0.135),
                rgba("#101817"),
 (math.cos(angle) * 0.088, -math.sin(angle) * 0.088, 0.0),
 rotation=(0.0, 0.0, math.sin(-angle * 0.5), math.cos(-angle * 0.5)),
                roughness=0.98,
            )
        )
    eco_wheel = [
        Part(
            "tire",
            "cylinder",
   (0.10, 0.10, 0.08),
            rgba("#314442"),
            roughness=0.72,
        ),
        Part(
            "aero_disc",
            "cylinder",
            (0.071, 0.071, 0.086),
            rgba("#A9C7BE"),
            metallic=0.40,
            roughness=0.28,
        ),
        Part(
            "hub",
            "cylinder",
            (0.025, 0.025, 0.091),
            mint,
            metallic=0.40,
            emissive=0.10,
        ),
    ]
    # Wheel assets are authored in their link frame: the joint axis is local +Z.
    # The robot link / mount transform turns +Z onto the vehicle's lateral axis.
    # Keeping that rotation out of the mesh prevents the physical link rotation
    # from being applied twice by the renderer.
    # An off-center sidewall witness makes encoder-backed rotation visible.
    for wheel_parts, face_z in ((standard_wheel, 0.056), (grip_wheel, 0.068), (eco_wheel, 0.046)):
        for side in (-1.0, 1.0):
            wheel_parts.append(
                Part(
                    f"rotation_witness_{'left' if side > 0 else 'right'}",
                    "box",
                    (0.028, 0.040, 0.010),
                    rgba("#F4B74B"),
                    (0.058, 0.0, side * face_z),
                    emissive=0.48,
                    roughness=0.30,
                )
            )
    tracked_hull = [
    Part("armored_belly", "box", (0.58, 0.46, 0.12), dark, (0, 0, -0.015),
    metallic=0.56, roughness=0.46),
    Part("main_hull", "box", (0.52, 0.42, 0.16), rgba("#A9C1BA"), (0.015, 0, 0.075),
    metallic=0.38, roughness=0.36),
    Part("sloped_nose", "box", (0.12, 0.38, 0.13), rgba("#C8DAD4"), (0.285, 0, 0.075),
    metallic=0.32, roughness=0.32),
    Part("front_bumper", "box", (0.045, 0.52, 0.075), rgba("#40504C"), (0.337, 0, -0.025),
    metallic=0.70, roughness=0.38),
    Part("equipment_deck", "box", (0.30, 0.28, 0.055), dark, (-0.035, 0, 0.185),
    metallic=0.62, roughness=0.28),
    Part("sensor_tower", "cylinder", (0.13, 0.13, 0.085), rgba("#768D87"),
    (0.085, 0, 0.245), metallic=0.48, roughness=0.30),
    Part("face_mask", "box", (0.032, 0.25, 0.080), rgba("#111B19"), (0.352, 0, 0.105),
    metallic=0.54, roughness=0.20),
    Part("eye_left", "box", (0.017, 0.060, 0.027), mint, (0.371, 0.072, 0.112), emissive=0.92),
    Part("eye_right", "box", (0.017, 0.060, 0.027), mint, (0.371, -0.072, 0.112), emissive=0.92),
    Part("side_guard_left", "box", (0.46, 0.028, 0.095), rgba("#6C807A"),
    (-0.005, 0.237, 0.055), metallic=0.66, roughness=0.33),
    Part("side_guard_right", "box", (0.46, 0.028, 0.095), rgba("#6C807A"),
    (-0.005, -0.237, 0.055), metallic=0.66, roughness=0.33),
    Part("rear_module", "box", (0.13, 0.31, 0.13), rgba("#829991"), (-0.255, 0, 0.115),
    metallic=0.40, roughness=0.36),
    Part("rear_status", "box", (0.018, 0.18, 0.024), rgba("#F0AA48"), (-0.326, 0, 0.135),
    emissive=0.72, roughness=0.24),
    ]
    tracked_track = [
    Part("belt_center", "box", (0.58, 0.145, 0.19), rgba("#171F1E"), roughness=0.96),
    Part("belt_top", "box", (0.58, 0.158, 0.032), rgba("#252E2C"), (0, 0, 0.105),
    roughness=0.98),
    Part("belt_bottom", "box", (0.58, 0.158, 0.032), rgba("#101615"), (0, 0, -0.105),
    roughness=0.99),
    Part("belt_front", "cylinder", (0.105, 0.105, 0.15), rgba("#202826"), (0.29, 0, 0),
    rotation=turn_x, roughness=0.97),
    Part("belt_rear", "cylinder", (0.105, 0.105, 0.15), rgba("#202826"), (-0.29, 0, 0),
    rotation=turn_x, roughness=0.97),
    ]
    for wheel_index, wheel_x in enumerate((-0.285, -0.145, 0.0, 0.145, 0.285)):
        radius = 0.078 if wheel_index in (0, 4) else 0.066
        tracked_track.extend([
        Part(f"road_wheel_{wheel_index}", "cylinder", (radius, radius, 0.166), rgba("#71837E"),
        (wheel_x, 0, 0), rotation=turn_x, metallic=0.62, roughness=0.34),
        Part(f"road_hub_{wheel_index}", "cylinder", (radius * 0.34, radius * 0.34, 0.174), mint,
        (wheel_x, 0, 0), rotation=turn_x, metallic=0.48, roughness=0.24, emissive=0.08),
        ])
    for pad_index, pad_x in enumerate((-0.27, -0.18, -0.09, 0.0, 0.09, 0.18, 0.27)):
        tracked_track.extend([
        Part(f"top_pad_{pad_index}", "box", (0.074, 0.174, 0.026), rgba("#0D1312"),
        (pad_x, 0, 0.124), roughness=1.0),
        Part(f"bottom_pad_{pad_index}", "box", (0.074, 0.174, 0.026), rgba("#0D1312"),
        (pad_x, 0, -0.124), roughness=1.0),
        ])
    models = {
        "chassis": [
            Part(
                "lower_skirt",
                "cylinder",
                (0.365, 0.265, 0.07),
                dark,
                (0, 0, -0.045),
                metallic=0.45,
                roughness=0.48,
            ),
            Part(
                "lower_hull",
                "cylinder",
                (0.35, 0.25, 0.14),
                white,
                (0, 0, 0.015),
                metallic=0.32,
                roughness=0.38,
            ),
            Part(
                "upper_hull",
                "cylinder",
                (0.275, 0.205, 0.105),
                rgba("#B8CCC6"),
                (0, 0, 0.125),
                metallic=0.28,
                roughness=0.34,
            ),
            Part(
                "top_deck",
                "box",
                (0.32, 0.26, 0.035),
                dark,
                (-0.015, 0, 0.190),
                metallic=0.50,
                roughness=0.28,
            ),
            Part(
                "face_panel",
                "box",
                (0.085, 0.30, 0.105),
                dark,
                (0.325, 0, 0.080),
                metallic=0.45,
                roughness=0.25,
            ),
            Part(
                "front_bumper",
                "box",
                (0.045, 0.42, 0.055),
                rgba("#52605C"),
                (0.362, 0, -0.035),
                metallic=0.62,
                roughness=0.38,
            ),
            Part(
                "front_accent",
                "box",
                (0.018, 0.31, 0.022),
                mint,
                (0.389, 0, 0.010),
                emissive=0.45,
                roughness=0.22,
            ),
            Part(
                "eye_bezel_left",
                "box",
                (0.030, 0.074, 0.052),
                rgba("#0A100F"),
                (0.373, 0.082, 0.094),
                metallic=0.55,
                roughness=0.20,
            ),
            Part(
                "eye_bezel_right",
                "box",
                (0.030, 0.074, 0.052),
                rgba("#0A100F"),
                (0.373, -0.082, 0.094),
                metallic=0.55,
                roughness=0.20,
            ),
            Part(
                "eye_left",
                "box",
                (0.018, 0.050, 0.026),
                mint,
                (0.391, 0.082, 0.094),
                emissive=0.90,
            ),
            Part(
                "eye_right",
                "box",
                (0.018, 0.050, 0.026),
                mint,
                (0.391, -0.082, 0.094),
                emissive=0.90,
            ),
            Part(
                "side_rail_left",
                "box",
                (0.29, 0.032, 0.055),
                rgba("#66807A"),
                (0.01, 0.259, 0.045),
                metallic=0.48,
                roughness=0.34,
            ),
            Part(
                "side_rail_right",
                "box",
                (0.29, 0.032, 0.055),
                rgba("#66807A"),
                (0.01, -0.259, 0.045),
                metallic=0.48,
                roughness=0.34,
            ),
            Part(
                "rear_service",
                "box",
                (0.085, 0.23, 0.085),
                rgba("#879C96"),
                (-0.315, 0, 0.055),
                metallic=0.36,
                roughness=0.36,
            ),
            Part(
                "rear_status",
                "box",
                (0.016, 0.13, 0.018),
                rgba("#F0AA48"),
                (-0.363, 0, 0.072),
                emissive=0.68,
                roughness=0.24,
            ),
 Part(
 "front_chin",
 "box",
 (0.060, 0.24, 0.036),
 rgba("#263A36"),
 (0.382, 0, -0.074),
 metallic=0.58,
 roughness=0.36,
 ),
 Part(
 "shoulder_left",
 "box",
 (0.20, 0.030, 0.095),
 rgba("#A6BDB7"),
 (0.105, 0.274, 0.105),
 metallic=0.34,
 roughness=0.31,
 ),
 Part(
 "shoulder_right",
 "box",
 (0.20, 0.030, 0.095),
 rgba("#A6BDB7"),
 (0.105, -0.274, 0.105),
 metallic=0.34,
 roughness=0.31,
 ),
 Part(
 "service_latch_left",
 "box",
 (0.055, 0.018, 0.030),
 mint,
 (-0.155, 0.282, 0.118),
 emissive=0.22,
 roughness=0.25,
 ),
 Part(
 "service_latch_right",
 "box",
 (0.055, 0.018, 0.030),
 mint,
 (-0.155, -0.282, 0.118),
 emissive=0.22,
 roughness=0.25,
 ),
        ],
 "caster_wheel": [
 Part(
 "caster_tire",
 "cylinder",
 (0.055, 0.055, 0.064),
 rgba("#151D1C"),
 (0, 0, 0),
 roughness=0.92,
 ),
 Part(
 "caster_rim",
 "cylinder",
 (0.031, 0.031, 0.069),
 rgba("#778A85"),
 (0, 0, 0),
 metallic=0.68,
 roughness=0.28,
 ),
 Part(
 "caster_axle",
 "cylinder",
 (0.013, 0.013, 0.096),
 mint,
 (0, 0, 0),
 metallic=0.72,
 roughness=0.22,
 ),
 ],
 "caster_fork": [
 Part("fork_left", "box", (0.070, 0.018, 0.092), rgba("#657772"),
 (-0.0175, 0.047, -0.013), metallic=0.66, roughness=0.30),
 Part("fork_right", "box", (0.070, 0.018, 0.092), rgba("#657772"),
 (-0.0175, -0.047, -0.013), metallic=0.66, roughness=0.30),
 Part("fork_bridge", "box", (0.092, 0.112, 0.026), rgba("#52645F"),
 (0, 0, 0.039), metallic=0.70, roughness=0.27),
 Part("swivel_stem", "cylinder", (0.024, 0.024, 0.075), rgba("#7E918B"),
 (0, 0, 0.082), metallic=0.76, roughness=0.22),
 Part("swivel_bearing", "cylinder", (0.052, 0.052, 0.022), dark,
 (0, 0, 0.127), metallic=0.56, roughness=0.28),
 ],
        "wheel_standard": standard_wheel,
        "wheel_grip": grip_wheel,
        "wheel_eco": eco_wheel,
        "lidar_fast": [
            Part("base", "cylinder", (0.105, 0.105, 0.035), dark),
            Part("scanner", "cylinder", (0.078, 0.078, 0.065), mint, (0, 0, 0.045)),
            Part(
                "cap", "cylinder", (0.052, 0.052, 0.018), rgba("#DDFCF5"), (0, 0, 0.087)
            ),
            Part(
                "scan_bar",
                "box",
                (0.19, 0.025, 0.018),
                mint,
                (0, 0, 0.105),
                emissive=0.82,
            ),
        ],
        "lidar_standard": [
            Part("base", "cylinder", (0.11, 0.11, 0.04), dark),
            Part(
                "scanner",
                "cylinder",
                (0.082, 0.082, 0.07),
                rgba("#4AA7E8"),
                (0, 0, 0.05),
            ),
            Part(
                "scan_bar",
                "box",
                (0.19, 0.025, 0.018),
                rgba("#A9DFFF"),
                (0, 0, 0.096),
                emissive=0.82,
            ),
        ],
        "lidar_long": [
            Part("base", "cylinder", (0.115, 0.115, 0.045), dark),
            Part(
                "scanner",
                "cylinder",
                (0.088, 0.088, 0.08),
                rgba("#E7A43F"),
                (0, 0, 0.055),
            ),
            Part(
                "scan_bar",
                "box",
                (0.20, 0.025, 0.018),
                rgba("#FFD58A"),
                (0, 0, 0.105),
                emissive=0.82,
            ),
            Part("antenna", "box", (0.025, 0.025, 0.11), rgba("#F6D087"), (0, 0, 0.14)),
        ],
        "compute_light": [
            Part("case", "box", (0.22, 0.20, 0.10), rgba("#607783"), metallic=0.48),
            Part(
                "indicator",
                "box",
                (0.025, 0.12, 0.025),
                mint,
                (0.112, 0, 0.02),
                emissive=0.78,
            ),
        ],
        "compute_performance": [
            Part("case", "box", (0.25, 0.22, 0.12), rgba("#754EB0"), metallic=0.42),
            Part(
                "heat_sink",
                "box",
                (0.14, 0.16, 0.035),
                rgba("#9D77D7"),
                (0, 0, 0.075),
                metallic=0.58,
            ),
            Part(
                "indicator",
                "box",
                (0.026, 0.14, 0.028),
                rgba("#F4B74B"),
                (0.127, 0, 0.015),
                emissive=0.78,
            ),
        ],
    }
    for name, parts in models.items():
        write_model(name, parts)
    tracked_output = ROOT / "assets" / "models" / "robots" / "ro3"
    write_model("hull", tracked_hull, tracked_output)
    write_model("track", tracked_track, tracked_output)


if __name__ == "__main__":
    main()
