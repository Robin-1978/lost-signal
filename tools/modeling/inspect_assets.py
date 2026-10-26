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
 triangle_count = 0
 material_names = set()
 for obj in bpy.context.scene.objects:
  if obj.type != "MESH":
   continue
  # Blender's glTF importer creates one non-exported Icosphere and assigns
  # it as the display shape for every imported bone. It is editor chrome,
  # absent from the GLB mesh inventory, and must not pollute model bounds or
  # triangle budgets for skinned assets.
  if any(collection.name == "glTF_not_exported"
   for collection in obj.users_collection):
   continue
  mesh_count += 1
  obj.data.calc_loop_triangles()
  triangle_count += len(obj.data.loop_triangles)
  material_names.update(
   slot.material.name for slot in obj.material_slots if slot.material)
  for corner in obj.bound_box:
   # Blender's glTF importer converts the file's Y-up convention into
   # Blender coordinates. The runtime manifest describes the coordinates
   # actually stored in the GLB/GLTF, so undo that importer basis change
   # before validating axis-specific bounds: blender=(x,-z,y).
   blender_point = obj.matrix_world @ Vector(corner)
   point = (blender_point.x, blender_point.z, -blender_point.y)
   for axis in range(3):
    lower[axis] = min(lower[axis], point[axis])
    upper[axis] = max(upper[axis], point[axis])
 if mesh_count == 0:
  raise RuntimeError(f"model has no mesh: {path}")
 return {
  "path": path.relative_to(MODELS).as_posix(),
  "mesh_count": mesh_count,
  "triangle_count": triangle_count,
  "material_count": len(material_names),
  "armature_count": sum(
   obj.type == "ARMATURE" for obj in bpy.context.scene.objects),
  "animation_clips": sorted(action.name for action in bpy.data.actions),
  "min": [round(value, 6) for value in lower],
  "max": [round(value, 6) for value in upper],
 }


def close_vec(actual, expected):
    return all(abs(a - b) <= TOLERANCE_M for a, b in zip(actual, expected))


def content_hash(path):
 value = 14695981039346656037
 for byte in path.read_bytes():
  value ^= byte
  value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
 return f"fnv1a64:{value:016x}"


def validate(entry, measured):
 if not entry.get("source"):
  raise RuntimeError(f"missing source metadata: {entry['path']}")
 for field in ("source_artifact", "license", "content_hash",
   "pivot_semantic", "link_binding"):
  if not entry.get(field):
   raise RuntimeError(f"missing {field}: {entry['path']}")
 if entry["content_hash"] != content_hash(MODELS / entry["path"]):
  raise RuntimeError(f"content hash changed for {entry['path']}")
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
 translation = entry.get("asset_to_link_translation_m")
 if not isinstance(translation, list) or len(translation) != 3:
  raise RuntimeError(f"asset-to-link translation missing for {entry['path']}")
 lods = entry.get("lods")
 if (not isinstance(lods, list) or not lods or lods[0].get("level") != 0 or
   lods[0].get("path") != entry["path"] or
   lods[0].get("maximum_distance_m") != 0):
  raise RuntimeError(f"LOD0 contract invalid for {entry['path']}")
 for level, lod in enumerate(lods):
  if lod.get("level") != level or not (MODELS / lod["path"]).is_file():
   raise RuntimeError(f"LOD inventory invalid for {entry['path']}")
  if level and lod["maximum_distance_m"] <= lods[level - 1]["maximum_distance_m"]:
   raise RuntimeError(f"LOD distance is not increasing for {entry['path']}")
 roles = entry.get("material_roles")
 if not isinstance(roles, list) or not roles or len(set(roles)) != len(roles):
  raise RuntimeError(f"material roles invalid for {entry['path']}")
 if measured["triangle_count"] > entry.get("triangle_budget", 0):
  raise RuntimeError(f"triangle budget exceeded for {entry['path']}")
 if measured["material_count"] > entry.get("material_budget", 0):
  raise RuntimeError(f"material budget exceeded for {entry['path']}")
 clips = entry.get("animation_clips")
 if clips != measured["animation_clips"]:
  raise RuntimeError(f"animation inventory changed for {entry['path']}")
 if measured["armature_count"] and not entry.get("skeleton_root"):
  raise RuntimeError(f"skeleton root missing for {entry['path']}")
 if entry.get("kind") == "sensor":
  optical = entry.get("sensor_optical_center_m")
  if not isinstance(optical, list) or len(optical) != 3:
   raise RuntimeError(f"sensor optical center missing for {entry['path']}")
 grounding = entry.get("grounding_points_m")
 if not isinstance(grounding, list):
  raise RuntimeError(f"grounding point list missing for {entry['path']}")
 up_axis = {"X": 0, "Y": 1, "Z": 2}[entry["up_axis"][-1]]
 support_plane = (measured["min"][up_axis]
  if entry["up_axis"][0] == "+" else measured["max"][up_axis])
 for point in grounding:
  if len(point) != 3 or not close_vec(
    [point[up_axis]], [support_plane]):
   raise RuntimeError(f"grounding point misses support plane: {entry['path']}")
  for axis in range(3):
   if not (measured["min"][axis] - TOLERANCE_M <= point[axis] <=
     measured["max"][axis] + TOLERANCE_M):
    raise RuntimeError(f"grounding point outside bounds: {entry['path']}")
 if (entry.get("kind") == "wheel_module" or
 not entry.get("reference_radius_m") or
 not entry.get("reference_half_width_m") or
 not entry.get("axle_axis")):
  return
 def axis_index(axis):
  return {"X": 0, "Y": 1, "Z": 2}[axis[-1]]
 forward_axis = axis_index(entry["forward_axis"])
 up_axis = axis_index(entry["up_axis"])
 axle_axis = axis_index(entry["axle_axis"])
 radius = max(
 measured["max"][forward_axis] - measured["min"][forward_axis],
 measured["max"][up_axis] - measured["min"][up_axis]) * 0.5
 half_width = (
 measured["max"][axle_axis] - measured["min"][axle_axis]) * 0.5
 if abs(radius - entry["reference_radius_m"]) > TOLERANCE_M:
  raise RuntimeError(f"wheel radius changed for {entry['path']}")
 if abs(half_width - entry["reference_half_width_m"]) > TOLERANCE_M:
  raise RuntimeError(f"wheel width changed for {entry['path']}")


def main():
 manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
 if manifest.get("format") != "lost-signal-assets/2":
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
