#!/usr/bin/env python3
"""Generate Lost Signal's small original UI and encounter sound set."""

from __future__ import annotations

import argparse
import io
import math
import random
import struct
import wave
from pathlib import Path

RATE = 22_050
ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "assets" / "audio"


def envelope(time_s: float, duration_s: float, attack_s: float = 0.012) -> float:
 return min(1.0, time_s / attack_s) * max(0.0, min(1.0, (duration_s - time_s) / 0.09))


def tone(time_s: float, frequency_hz: float, phase: float = 0.0) -> float:
 return math.sin(2.0 * math.pi * frequency_hz * time_s + phase)


def render(duration_s: float, sample) -> bytes:
 frames = bytearray()
 for index in range(round(duration_s * RATE)):
  value = max(-1.0, min(1.0, sample(index / RATE)))
  frames.extend(struct.pack("<h", round(value * 32767)))
 target = io.BytesIO()
 with wave.open(target, "wb") as output:
  output.setnchannels(1)
  output.setsampwidth(2)
  output.setframerate(RATE)
  output.writeframes(bytes(frames))
 return target.getvalue()


def note(time_s: float, start_s: float, duration_s: float, frequency_hz: float) -> float:
 local = time_s - start_s
 if local < 0.0 or local >= duration_s:
  return 0.0
 body = tone(local, frequency_hz) + 0.24 * tone(local, frequency_hz * 2.0, 0.2)
 return body * envelope(local, duration_s) * 0.34


def assets() -> dict[str, bytes]:
 rng = random.Random(47)
 collision_noise = [rng.uniform(-1.0, 1.0) for _ in range(round(0.42 * RATE))]
 harvest_noise = [rng.uniform(-1.0, 1.0) for _ in range(round(0.48 * RATE))]
 return {
 "power_pickup.wav": render(
 0.56,
 lambda t: note(t, 0.00, 0.22, 392.0) + note(t, 0.13, 0.24, 523.25) + note(t, 0.27, 0.27, 783.99),
 ),
 "traffic_warning.wav": render(
 0.66,
 lambda t: (note(t, 0.00, 0.22, 659.25) + note(t, 0.28, 0.22, 659.25)) * 0.82,
 ),
 "decision_wait.wav": render(
 0.34,
 lambda t: note(t, 0.00, 0.30, 293.66) + 0.52 * note(t, 0.05, 0.25, 220.0),
 ),
 "decision_clear.wav": render(
 0.40,
 lambda t: note(t, 0.00, 0.21, 440.0) + note(t, 0.13, 0.25, 659.25),
 ),
 "relay_restored.wav": render(
 1.08,
 lambda t: note(t, 0.00, 0.34, 392.0) + note(t, 0.16, 0.38, 523.25)
 + note(t, 0.34, 0.48, 659.25) + note(t, 0.56, 0.48, 783.99),
 ),
 "collision.wav": render(
 0.42,
 lambda t: (0.62 * collision_noise[min(len(collision_noise) - 1, int(t * RATE))]
 + 0.38 * tone(t, 82.0)) * math.exp(-8.0 * t),
 ),
        "harvest_pickup.wav": render(
            0.58,
            lambda t: 0.10 * harvest_noise[min(len(harvest_noise) - 1, int(t * RATE))]
            * math.exp(-7.0 * t)
            + note(t, 0.05, 0.25, 349.23)
            + note(t, 0.19, 0.31, 523.25)
            + 0.55 * note(t, 0.30, 0.24, 698.46),
        ),
        "task_abandoned.wav": render(
            0.54,
            lambda t: note(t, 0.00, 0.28, 392.0)
            + note(t, 0.18, 0.32, 293.66),
        ),
        "fruit_damage.wav": render(
            0.48,
            lambda t: (0.36 * harvest_noise[min(len(harvest_noise) - 1, int(t * RATE))]
            + 0.24 * tone(t, 118.0) + 0.18 * tone(t, 177.0)) * math.exp(-9.5 * t),
        ),
 }


def main() -> int:
 parser = argparse.ArgumentParser(description=__doc__)
 parser.add_argument("--check", action="store_true", help="fail when generated files differ")
 args = parser.parse_args()
 generated = assets()
 if args.check:
  mismatches = [name for name, data in generated.items() if not (OUTPUT / name).is_file()
  or (OUTPUT / name).read_bytes() != data]
  if mismatches:
   print("audio differs: " + ", ".join(mismatches))
   return 1
  print("audio: up to date")
  return 0
 OUTPUT.mkdir(parents=True, exist_ok=True)
 for name, data in generated.items():
  (OUTPUT / name).write_bytes(data)
 print(f"generated {len(generated)} audio assets in {OUTPUT.relative_to(ROOT)}")
 return 0


if __name__ == "__main__":
 raise SystemExit(main())
