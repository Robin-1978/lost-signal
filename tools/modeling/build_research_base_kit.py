"""Build the first authored research-base environment kit.

Run:
  /disk/blender-5.2.2-linux-x64/blender --background \
    --python tools/modeling/build_research_base_kit.py

The generated GLBs are presentation assets. Solid models bind existing world
collision proxies through the scene manifest; the charging pad is marked as a
surface-only visual. Coordinates use metres, +X forward, +Y left, and +Z up.
"""

from pathlib import Path
import math

import bpy


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "assets" / "models" / "environment" / "research_base"


def reset_scene():
 bpy.ops.wm.read_factory_settings(use_empty=True)


def material(name, color, metallic=0.0, roughness=0.45, emission=None,
 alpha=1.0):
 result = bpy.data.materials.new(name)
 result.use_nodes = True
 shader = result.node_tree.nodes.get("Principled BSDF")
 shader.inputs["Base Color"].default_value = (*color, alpha)
 shader.inputs["Metallic"].default_value = metallic
 shader.inputs["Roughness"].default_value = roughness
 shader.inputs["Alpha"].default_value = alpha
 if emission is not None:
  shader.inputs["Emission Color"].default_value = (*emission, 1.0)
  shader.inputs["Emission Strength"].default_value = 2.8
 if alpha < 1.0:
  result.surface_render_method = "DITHERED"
 result.diffuse_color = (*color, alpha)
 return result


def assign(obj, surface):
 obj.data.materials.append(surface)


def box(name, location, dimensions, surface, bevel=0.0):
 bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
 obj = bpy.context.object
 obj.name = name
 obj.dimensions = dimensions
 bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
 assign(obj, surface)
 if bevel > 0.0:
  modifier = obj.modifiers.new("edge_highlight", "BEVEL")
  modifier.width = bevel
  modifier.segments = 2
  modifier.limit_method = "ANGLE"
  modifier.angle_limit = math.radians(25.0)
  bpy.context.view_layer.objects.active = obj
  bpy.ops.object.modifier_apply(modifier=modifier.name)
 return obj


def cylinder(name, location, radius, depth, surface, vertices=24):
 bpy.ops.mesh.primitive_cylinder_add(
  vertices=vertices, radius=radius, depth=depth, location=location)
 obj = bpy.context.object
 obj.name = name
 assign(obj, surface)
 return obj


def export_glb(filename):
 OUTPUT.mkdir(parents=True, exist_ok=True)
 bpy.ops.export_scene.gltf(
  filepath=str(OUTPUT / filename), export_format="GLB",
  use_selection=False, export_apply=False, export_yup=False)


def build_storage_rack():
 reset_scene()
 graphite = material("rack_graphite", (0.055, 0.075, 0.075), 0.65, 0.24)
 steel = material("rack_steel", (0.38, 0.44, 0.43), 0.72, 0.25)
 orange = material("rack_safety_orange", (0.96, 0.39, 0.055), 0.25, 0.27)
 mint = material("rack_identity_mint", (0.08, 0.72, 0.59), 0.18, 0.22,
                 (0.03, 0.25, 0.18))
 crate_blue = material("rack_crate_blue", (0.12, 0.25, 0.31), 0.05, 0.45)
 crate_sand = material("rack_crate_sand", (0.58, 0.46, 0.27), 0.0, 0.55)

 # Six bays fill the same 1.0 m x 11.0 m x 1.2 m physical proxy.
 post_y = [-5.45, -3.27, -1.09, 1.09, 3.27, 5.45]
 for index, y in enumerate(post_y):
  for x in (-0.46, 0.46):
   box(f"rack_post_{index}_{'left' if x > 0 else 'right'}",
       (x, y, 0.60), (0.08, 0.08, 1.20), graphite, 0.008)
  box(f"rack_crossbar_{index}", (0.0, y, 1.14),
      (0.92, 0.07, 0.08), orange, 0.008)
 for level, z in enumerate((0.09, 0.59, 1.10)):
  box(f"rack_shelf_{level}", (0.0, 0.0, z),
      (0.88, 10.90, 0.065), steel, 0.006)
  for bay in range(5):
   y = -4.36 + 2.18 * bay
   box(f"rack_beam_{level}_{bay}", (0.0, y, z + 0.04),
       (0.98, 0.07, 0.10), orange, 0.007)

 # Cargo gives the rack rhythm and scale while staying inside its collision
 # envelope. Gaps preserve visual depth and prevent a single grey wall.
 cargo = [
  (-0.21, -4.35, 0.31, 0.36, 0.74, 0.34, crate_blue),
  (0.19, -3.55, 0.28, 0.40, 0.62, 0.28, crate_sand),
  (-0.18, -1.65, 0.32, 0.42, 0.78, 0.36, crate_sand),
  (0.20, 0.45, 0.30, 0.38, 0.68, 0.32, crate_blue),
  (-0.18, 2.50, 0.31, 0.42, 0.76, 0.34, crate_blue),
  (0.18, 4.55, 0.29, 0.38, 0.66, 0.30, crate_sand),
  (0.18, -4.15, 0.82, 0.38, 0.70, 0.30, crate_sand),
  (-0.18, -0.55, 0.82, 0.40, 0.76, 0.30, crate_blue),
  (0.20, 3.25, 0.81, 0.36, 0.64, 0.28, crate_sand),
 ]
 for index, (x, y, z, dx, dy, dz, surface) in enumerate(cargo):
  box(f"rack_cargo_{index}", (x, y, z), (dx, dy, dz), surface, 0.018)
  box(f"rack_cargo_label_{index}",
      (x + math.copysign(dx * 0.5 + 0.002, x), y, z),
      (0.006, dy * 0.46, dz * 0.28), mint, 0.002)

 # End guards and a glowing asset identity strip are legible at overview
 # distance and visually communicate the protected rack footprint.
 for y in (-5.47, 5.47):
  box(f"rack_end_guard_{y:+.0f}", (0.0, y, 0.12),
      (1.00, 0.055, 0.24), orange, 0.012)
 box("rack_identity_strip", (0.475, 0.0, 1.17),
     (0.018, 10.70, 0.026), mint, 0.004)
 export_glb("storage_rack_11m.glb")


def build_glass_gate():
 reset_scene()
 graphite = material("gate_graphite", (0.045, 0.065, 0.068), 0.70, 0.22)
 mint = material("gate_mint", (0.08, 0.78, 0.66), 0.18, 0.18,
                 (0.04, 0.38, 0.27))
 amber = material("gate_amber", (1.0, 0.52, 0.08), 0.05, 0.22,
                  (0.8, 0.20, 0.02))
 glass = material("gate_glass", (0.10, 0.42, 0.48), 0.10, 0.08, alpha=0.28)

 box("gate_glass", (0.0, 0.0, 0.64), (0.045, 0.82, 0.94), glass, 0.006)
 box("gate_floor_threshold", (0.0, 0.0, 0.03),
 (0.50, 0.96, 0.06), graphite, 0.012)
 for y in (-0.47, 0.47):
  box(f"gate_post_{y:+.0f}", (0.0, y, 0.60),
      (0.16, 0.06, 1.20), graphite, 0.012)
 for z in (0.16, 1.12):
  box(f"gate_rail_{z:.2f}", (0.0, 0.0, z),
      (0.12, 0.94, 0.065), graphite, 0.008)
 box("gate_scan_band", (-0.065, 0.0, 0.68),
     (0.022, 0.88, 0.035), mint, 0.004)
 box("gate_warning_band", (-0.066, 0.0, 0.27),
     (0.022, 0.72, 0.055), amber, 0.004)
 for y in (-0.42, 0.42):
  cylinder(f"gate_status_{y:+.0f}", (-0.09, y, 1.12),
           0.027, 0.026, amber, 24).rotation_euler[1] = math.pi / 2.0
 export_glb("glass_safety_gate.glb")


def build_charging_pad():
 reset_scene()
 graphite = material("pad_graphite", (0.035, 0.055, 0.057), 0.30, 0.40)
 mint = material("pad_mint", (0.08, 0.82, 0.67), 0.12, 0.22,
                 (0.04, 0.48, 0.32))
 amber = material("pad_amber", (1.0, 0.50, 0.07), 0.05, 0.24,
                  (0.65, 0.15, 0.01))
 box("pad_base", (0.0, 0.0, 0.007), (1.20, 0.90, 0.014), graphite, 0.01)
 for side in (-1.0, 1.0):
  box(f"pad_guide_{side:+.0f}", (0.0, side * 0.39, 0.015),
      (0.92, 0.045, 0.025), mint, 0.008)
 for index, x in enumerate((-0.38, -0.18, 0.02, 0.22, 0.42)):
  strip = box(f"pad_chevron_{index}", (x, 0.0, 0.017),
      (0.09, 0.56, 0.012), amber if index % 2 == 0 else mint, 0.006)
  strip.rotation_euler[2] = math.radians(-22.0)
 box("pad_centerline", (0.0, 0.0, 0.019),
     (0.76, 0.025, 0.010), mint, 0.004)
 export_glb("robot_charging_pad.glb")


def build_traffic_signal():
 reset_scene()
 graphite = material("signal_graphite", (0.035, 0.050, 0.055), 0.72, 0.22)
 steel = material("signal_steel", (0.22, 0.30, 0.30), 0.65, 0.26)
 red = material("signal_red", (0.95, 0.035, 0.025), 0.05, 0.20,
  (1.0, 0.015, 0.005))
 amber = material("signal_amber", (0.98, 0.40, 0.025), 0.05, 0.20,
  (1.0, 0.16, 0.005))
 green = material("signal_green", (0.03, 0.86, 0.30), 0.05, 0.20,
  (0.01, 0.32, 0.06))
 # A compact, readable signal on a weighted foot. The visual is
 # surface-only in the scene manifest; no collision proxy is inferred
 # from the mesh and no asset pose is fed back into ROBOT.
 box("signal_foot", (0.0, 0.0, 0.035), (0.42, 0.32, 0.07), steel, 0.025)
 cylinder("signal_pole", (0.0, 0.0, 1.02), 0.035, 1.97, graphite, 20)
 box("signal_head", (0.0, 0.0, 1.82), (0.24, 0.18, 0.72), graphite,
  0.025)
 lenses = ((1.60, red), (1.82, amber), (2.04, green))
 for index, (height, surface) in enumerate(lenses):
  lens = cylinder(f"signal_lens_{index}", (0.0, -0.097, height),
  0.072, 0.018, surface, 24)
  lens.rotation_euler[0] = math.radians(90.0)
 box("signal_cap", (0.0, 0.0, 2.24), (0.28, 0.22, 0.05), steel, 0.015)
 export_glb("traffic_signal.glb")


def build_safety_bollards():
 reset_scene()
 graphite = material("bollard_graphite", (0.045, 0.065, 0.065), 0.60, 0.25)
 yellow = material("bollard_yellow", (0.95, 0.48, 0.035), 0.12, 0.30)
 cyan = material("bollard_cyan", (0.02, 0.58, 0.62), 0.20, 0.24,
  (0.01, 0.12, 0.15))
 for index, x in enumerate((-0.52, 0.0, 0.52)):
  box(f"bollard_foot_{index}", (x, 0.0, 0.035), (0.28, 0.28, 0.07),
  graphite, 0.02)
  cylinder(f"bollard_post_{index}", (x, 0.0, 0.46), 0.055, 0.82,
  yellow if index != 1 else cyan, 20)
  cylinder(f"bollard_cap_{index}", (x, 0.0, 0.89), 0.075, 0.045,
  graphite, 20)
 # Thin visual chain marks a boundary; the collision world remains the
 # only authority and this asset is placed surface-only.
 for index, x in enumerate((-0.26, 0.26)):
  chain = cylinder(f"bollard_chain_{index}", (x, 0.0, 0.56), 0.018,
  0.52, yellow, 12)
  chain.rotation_euler[1] = math.radians(90.0)
 export_glb("safety_bollards.glb")


def build_pallet_stack():
 reset_scene()
 wood = material("pallet_wood", (0.48, 0.24, 0.09), 0.0, 0.62)
 wood_light = material("pallet_edge", (0.72, 0.42, 0.16), 0.0, 0.55)
 crate_blue = material("crate_blue", (0.055, 0.22, 0.34), 0.08, 0.42)
 crate_orange = material("crate_orange", (0.88, 0.24, 0.035), 0.08, 0.40)
 # One reusable pallet with four contrasting crates. All geometry sits
 # on z=0 so the scene manifest can place it without a draw-time
 # grounding correction.
 box("pallet_deck", (0.0, 0.0, 0.075), (1.10, 0.82, 0.15), wood, 0.025)
 for index, x in enumerate((-0.42, 0.42)):
  box(f"pallet_runner_{index}", (x, 0.0, 0.035), (0.12, 0.76, 0.07),
  wood_light, 0.012)
 crates = ((-0.28, -0.20, crate_blue), (0.28, -0.20, crate_orange),
  (-0.28, 0.20, crate_orange), (0.28, 0.20, crate_blue))
 for index, (x, y, surface) in enumerate(crates):
  box(f"pallet_crate_{index}", (x, y, 0.39), (0.46, 0.32, 0.48),
  surface, 0.025)
  box(f"pallet_label_{index}", (x, y - 0.166, 0.40),
  (0.23, 0.008, 0.17), wood_light, 0.003)
 export_glb("pallet_stack.glb")

def main():
 build_storage_rack()
 build_glass_gate()
 build_charging_pad()
 build_traffic_signal()
 build_safety_bollards()
 build_pallet_stack()
 reset_scene()
 print("research-base environment kit generated")


if __name__ == "__main__":
 main()
