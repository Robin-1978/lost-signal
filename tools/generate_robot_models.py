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


def cylinder_mesh(size: tuple[float, float, float], sides: int = 16) -> Mesh:
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
    models = {
        "chassis": [
            Part("lower_hull", "cylinder", (0.35, 0.25, 0.15), white, metallic=0.32),
            Part(
                "upper_hull",
                "cylinder",
                (0.27, 0.20, 0.10),
                rgba("#B7CBC5"),
                (0, 0, 0.115),
            ),
            Part(
                "face_panel",
                "box",
                (0.10, 0.29, 0.095),
                dark,
                (0.31, 0, 0.075),
                metallic=0.45,
            ),
            Part(
                "eye_left",
                "box",
                (0.025, 0.055, 0.032),
                mint,
                (0.365, 0.075, 0.085),
                emissive=0.90,
            ),
            Part(
                "eye_right",
                "box",
                (0.025, 0.055, 0.032),
                mint,
                (0.365, -0.075, 0.085),
                emissive=0.90,
            ),
        ],
        "wheel_standard": [
            Part(
                "wheel",
                "cylinder",
                (0.105, 0.105, 0.10),
                rgba("#263331"),
                rotation=turn_x,
                roughness=0.82,
            ),
            Part(
                "hub",
                "cylinder",
                (0.045, 0.045, 0.105),
                rgba("#7B918B"),
                rotation=turn_x,
                metallic=0.58,
            ),
        ],
        "wheel_grip": [
            Part(
                "wheel",
                "cylinder",
                (0.115, 0.115, 0.12),
                dark,
                rotation=turn_x,
                roughness=0.90,
            ),
            Part(
                "hub",
                "cylinder",
                (0.048, 0.048, 0.125),
                mint,
                rotation=turn_x,
                metallic=0.42,
            ),
        ],
        "wheel_eco": [
            Part(
                "wheel",
                "cylinder",
                (0.098, 0.098, 0.08),
                rgba("#314442"),
                rotation=turn_x,
                roughness=0.74,
            ),
            Part(
                "hub",
                "cylinder",
                (0.055, 0.055, 0.085),
                rgba("#8DB8AB"),
                rotation=turn_x,
                metallic=0.38,
            ),
        ],
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


if __name__ == "__main__":
    main()
