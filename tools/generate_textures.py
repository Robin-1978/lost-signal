#!/usr/bin/env python3
"""Generate deterministic, project-owned textures with the Python standard library."""

from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "assets" / "textures" / "environment"
SIZE = 512


def _hash(x: int, y: int, salt: int = 0) -> int:
    value = (x * 0x1F123BB5 + y * 0x5F356495 + salt * 0x6C8E9CF5) & 0xFFFFFFFF
    value ^= value >> 15
    value = (value * 0x2C1B3C6D) & 0xFFFFFFFF
    value ^= value >> 12
    return value & 0xFFFFFFFF


def _smooth(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def _tile_noise(x: int, y: int, cell: int, salt: int) -> float:
    grid = SIZE // cell
    cell_x, cell_y = x // cell, y // cell
    local_x = _smooth((x % cell) / cell)
    local_y = _smooth((y % cell) / cell)

    def sample(offset_x: int, offset_y: int) -> float:
        return (
            _hash((cell_x + offset_x) % grid, (cell_y + offset_y) % grid, salt) & 0xFFFF
        ) / 65535.0

    top = sample(0, 0) * (1.0 - local_x) + sample(1, 0) * local_x
    bottom = sample(0, 1) * (1.0 - local_x) + sample(1, 1) * local_x
    return top * (1.0 - local_y) + bottom * local_y


def _wrapped_distance(value: int, center: int) -> int:
    direct = abs(value - center)
    return min(direct, SIZE - direct)


def concrete_floor() -> bytes:
    stains = [
        (73, 118, 66, 42, 11),
        (390, 84, 92, 54, 8),
        (286, 371, 74, 96, 10),
        (491, 452, 51, 37, 7),
    ]
    crack_segments = [
        ((42, 226), (161, 279)),
        ((161, 279), (209, 366)),
        ((209, 366), (292, 410)),
        ((363, 33), (418, 111)),
        ((418, 111), (488, 153)),
        ((488, 153), (23, 179)),
    ]
    pixels = bytearray()
    for y in range(SIZE):
        for x in range(SIZE):
            broad = _tile_noise(x, y, 64, 3) - 0.5
            medium = _tile_noise(x, y, 16, 7) - 0.5
            grain = ((_hash(x, y, 19) >> 8) & 0xFF) / 255.0 - 0.5
            value = 218.0 + broad * 25.0 + medium * 10.0 + grain * 8.0

            for center_x, center_y, radius_x, radius_y, strength in stains:
                dx = _wrapped_distance(x, center_x) / radius_x
                dy = _wrapped_distance(y, center_y) / radius_y
                distance = dx * dx + dy * dy
                if distance < 1.0:
                    value -= (1.0 - distance) ** 2 * strength

            crack = 0.0
            for (start_x, start_y), (end_x, end_y) in crack_segments:
                segment_x, segment_y = end_x - start_x, end_y - start_y
                length_squared = segment_x * segment_x + segment_y * segment_y
                progress = max(
                    0.0,
                    min(
                        1.0,
                        ((x - start_x) * segment_x + (y - start_y) * segment_y)
                        / length_squared,
                    ),
                )
                nearest_x = start_x + segment_x * progress
                nearest_y = start_y + segment_y * progress
                distance = math.hypot(x - nearest_x, y - nearest_y)
                if distance < 1.6:
                    crack = max(crack, (1.0 - distance / 1.6) * 42.0)
            value -= crack

            if (_hash(x, y, 31) & 0xFFF) < 5:
                value -= 24.0
            level = max(126, min(242, round(value)))
            pixels.extend((level, max(120, level - 1), max(116, level - 5), 255))
    return bytes(pixels)


def mud_floor() -> bytes:
    """Wet orchard soil with tileable ruts, puddles, and granular clods."""
    puddles = [
        (82, 96, 72, 44),
        (356, 174, 108, 55),
        (195, 402, 82, 48),
        (486, 468, 58, 37),
    ]
    pixels = bytearray()
    for y in range(SIZE):
        for x in range(SIZE):
            broad = _tile_noise(x, y, 64, 71) - 0.5
            clods = _tile_noise(x, y, 12, 73) - 0.5
            grain = ((_hash(x, y, 79) >> 9) & 0xFF) / 255.0 - 0.5
            wet = 0.0
            for center_x, center_y, radius_x, radius_y in puddles:
                dx = _wrapped_distance(x, center_x) / radius_x
                dy = _wrapped_distance(y, center_y) / radius_y
                distance = dx * dx + dy * dy
                if distance < 1.0:
                    wet = max(wet, (1.0 - distance) ** 2)

            # Two gently wandering wheel ruts remain seamless across the tile.
            rut_center = 144.0 + 18.0 * math.sin(y * 2.0 * math.pi / SIZE)
            rut_distance = min(_wrapped_distance(x, round(rut_center)),
                _wrapped_distance(x, round(rut_center + 210.0)))
            rut = max(0.0, 1.0 - rut_distance / 15.0)
            wet = max(wet, rut * 0.58)

            red = 91.0 + broad * 25.0 + clods * 10.0 + grain * 5.0 - wet * 33.0
            green = 72.0 + broad * 17.0 + clods * 7.0 + grain * 4.0 - wet * 29.0
            blue = 49.0 + broad * 11.0 + clods * 5.0 + grain * 3.0 - wet * 16.0
            if (_hash(x, y, 83) & 0x7FF) < 4:
                red += 25.0
                green += 18.0
                blue += 8.0
            pixels.extend((
                max(28, min(126, round(red))),
                max(24, min(104, round(green))),
                max(19, min(82, round(blue))),
                255,
                ))
    return bytes(pixels)


def write_png(path: Path, width: int, height: int, pixels: bytes) -> None:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    rows = b"".join(
        b"\0" + pixels[row * width * 4 : (row + 1) * width * 4] for row in range(height)
    )
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(rows, level=9))
    png += chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def main() -> None:
    write_png(OUTPUT / "concrete_floor.png", SIZE, SIZE, concrete_floor())
    write_png(OUTPUT / "mud_floor.png", SIZE, SIZE, mud_floor())


if __name__ == "__main__":
    main()
