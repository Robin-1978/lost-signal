"""Validate every model against the versioned asset manifest.

Run:
  /disk/blender-5.2.2-linux-x64/blender --background \
    --python tools/modeling/inspect_assets.py

The script never edits an asset. It imports each GLB/GLTF into a clean scene,
evaluates object transforms, validates inventory, source metadata, pivot,
bounds, and wheel reference dimensions, then prints one deterministic record.
"""

import json
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[2]
MODELS = ROOT / "assets" / "models"
MANIFEST = MODELS / "manifest.json"
TOLERANCE_M = 1.0e-5


def clear_scene():
 bpy.ops.wm.read_factory_settings(use_empty=True)


def import_model(path):
 if path.suffix == ".glb" or path.suffix == ".gltf":
  bpy.ops.import_scene.gltf(filepath=str(path))
  return
 raise RuntimeError(f"unsupported model extension: {path}")


def model_bounds(path):
 clear_scene()
 import_model(path)
 lower = [float("inf")] * 3
 upper = [float("-inf")] * 3
 mesh_count = 0
 for obj in bpy.context.scene.objects:
  if obj.type != "MESH":
   continue
  mesh_count += 1
  for corner in obj.bound_box:
   point = obj.matrix_world @ Vector(corner)
   for axis in range(3):
    lower[axis] = min(lower[axis], point[axis])
    upper[axis] = max(upper[axis], point[axis])
 if mesh_count == 0:
  raise RuntimeError(f"model has no mesh: {path}")
 return {
  "path": path.relative_to(MODELS).as_posix(),
  "mesh_count": mesh_count,
  "min": [round(value, 6) for value in lower],
  "max": [round(value, 6) for value in upper],
 }


def close_vec(actual, expected):
    return all(abs(a - b) <= TOLERANCE_M for a, b in zip(actual, expected))


def validate(entry, measured):
 if not entry.get("source"):
  raise RuntimeError(f"missing source metadata: {entry['path']}")
 if entry.get("meters_per_unit") != 1:
  raise RuntimeError(f"asset is not authored in meters: {entry['path']}")
 expected = entry["expected_bounds_m"]
 if not close_vec(measured["min"], expected["min"]):
  raise RuntimeError(
   f"minimum bounds changed for {entry['path']}: "
   f"{measured['min']} != {expected['min']}")
 if not close_vec(measured["max"], expected["max"]):
  raise RuntimeError(
   f"maximum bounds changed for {entry['path']}: "
   f"{measured['max']} != {expected['max']}")
 pivot = entry["pivot_m"]
 for axis in range(3):
  if pivot[axis] < measured["min"][axis] - TOLERANCE_M:
   raise RuntimeError(f"pivot below bounds for {entry['path']}")
  if pivot[axis] > measured["max"][axis] + TOLERANCE_M:
   raise RuntimeError(f"pivot above bounds for {entry['path']}")
 if entry["kind"] != "wheel":
  return
 radius = max(
  measured["max"][0] - measured["min"][0],
  measured["max"][2] - measured["min"][2]) * 0.5
 half_width = (measured["max"][1] - measured["min"][1]) * 0.5
 if abs(radius - entry["reference_radius_m"]) > TOLERANCE_M:
  raise RuntimeError(f"wheel radius changed for {entry['path']}")
 if abs(half_width - entry["reference_half_width_m"]) > TOLERANCE_M:
  raise RuntimeError(f"wheel width changed for {entry['path']}")


def main():
 manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
 if manifest.get("format") != "lost-signal-assets/1":
  raise RuntimeError("unsupported asset manifest format")
 entries = {entry["path"]: entry for entry in manifest["assets"]}
 paths = sorted([*MODELS.rglob("*.glb"), *MODELS.rglob("*.gltf")])
 inventory = {path.relative_to(MODELS).as_posix() for path in paths}
 if set(entries) != inventory:
  missing = sorted(inventory - set(entries))
  stale = sorted(set(entries) - inventory)
  raise RuntimeError(
   f"manifest inventory mismatch missing={missing} stale={stale}")
 for path in paths:
  measured = model_bounds(path)
  validate(entries[measured["path"]], measured)
  print("ASSET_BOUNDS " + json.dumps(measured, sort_keys=True))
 clear_scene()
 print(f"ASSET_MANIFEST_OK assets={len(paths)}")


if __name__ == "__main__":
    main()
