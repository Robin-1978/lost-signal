"""Author the RO-1 hero presentation assets for the teaching product.

The meshes are visual-only.  MuJoCo remains the authority for collision,
links, joints and contact.  The canonical frame is metres, +X forward, +Y
left and +Z up; the asset manifest and existing link bindings are unchanged.

Run:
  /disk/blender-5.2.2-linux-x64/blender --background \
    --python tools/modeling/redesign_hero_assets.py
"""

from pathlib import Path
import math

import bpy

ROOT = Path(__file__).resolve().parents[2]
MODELS = ROOT / "assets" / "models"


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def material(name, color, metallic=0.0, roughness=0.4, emission=None):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    shader = result.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1.0)
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    if emission is not None:
        shader.inputs["Emission Color"].default_value = (*emission, 1.0)
        shader.inputs["Emission Strength"].default_value = 2.2
    result.diffuse_color = (*color, 1.0)
    return result


def assign(obj, surface):
    obj.data.materials.append(surface)


def bevel(obj, width):
    modifier = obj.modifiers.new("manufactured_edge", "BEVEL")
    modifier.width = width
    modifier.segments = 1
    modifier.limit_method = "ANGLE"
    modifier.angle_limit = math.radians(25.0)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=modifier.name)


def box(name, location, dimensions, surface, edge=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(obj, surface)
    if edge > 0.0 and name in {
        "base_skirt", "lower_hull", "upper_shell", "front_bumper",
        "rear_service", "sensor_deck",
    }:
        bevel(obj, edge)
    return obj


def cylinder(name, location, radius, depth, surface, vertices=16, axis="Z"):
    rotation = {
        "X": (0.0, math.pi / 2.0, 0.0),
        "Y": (math.pi / 2.0, 0.0, 0.0),
        "Z": (0.0, 0.0, 0.0),
    }[axis]
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices, radius=radius, depth=depth,
        location=location, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    assign(obj, surface)
    return obj


def torus(name, location, major_radius, minor_radius, surface):
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_radius, minor_radius=minor_radius,
        major_segments=8, minor_segments=4, location=location)
    obj = bpy.context.object
    obj.name = name
    assign(obj, surface)
    return obj


def export(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(path), export_format="GLB",
        use_selection=False, export_apply=False, export_yup=False)


def build_ro1_chassis():
    reset()
    graphite = material("ro1_hero_graphite", (0.025, 0.045, 0.050), 0.58, 0.26)
    shell = material("ro1_hero_shell", (0.28, 0.42, 0.43), 0.18, 0.30)
    pearl = material("ro1_hero_pearl", (0.72, 0.80, 0.78), 0.12, 0.27)
    alloy = material("ro1_hero_alloy", (0.30, 0.44, 0.45), 0.78, 0.20)
    mint = material("ro1_hero_mint", (0.05, 0.78, 0.59), 0.16, 0.19,
                    (0.02, 0.42, 0.25))
    amber = material("ro1_hero_amber", (0.95, 0.38, 0.045), 0.06, 0.22,
                     (0.58, 0.10, 0.01))

    # The outer envelope is exactly the existing manifest contract:
    # x=[-.371,.412], y=[-.291,.291], z=[-.092,.2075]. Every feature is
    # inset from that envelope so visual changes cannot alter the footprint.
    box("base_skirt", (0.020, 0.0, -0.042), (0.744, 0.540, 0.100),
        graphite, 0.012)
    box("lower_hull", (0.000, 0.0, 0.037), (0.700, 0.500, 0.140),
        pearl, 0.020)
    box("upper_shell", (-0.020, 0.0, 0.112), (0.620, 0.440, 0.150),
        shell, 0.025)
    box("front_bumper", (0.395, 0.0, 0.037), (0.034, 0.520, 0.100),
        alloy, 0.010)
    box("rear_service", (-0.340, 0.0, 0.070), (0.040, 0.420, 0.120),
        graphite, 0.008)

    # A raised top platform gives the sensor and compute modules an obvious
    # visual hierarchy without pretending they are extra physical bodies.
    box("sensor_deck", (0.000, 0.0, 0.196), (0.300, 0.300, 0.023),
        graphite, 0.008)
    box("top_spine", (-0.015, 0.0, 0.199), (0.300, 0.044, 0.012),
        alloy, 0.004)
    box("top_status", (0.145, 0.0, 0.199), (0.026, 0.120, 0.009),
        mint, 0.003)

    # Forward identity: two amber lamps and one continuous mint safety band
    # make direction legible in a still frame and during slow teaching replay.
    box("front_face", (0.386, 0.0, 0.102), (0.006, 0.300, 0.100),
        graphite, 0.002)
    for side in (-1.0, 1.0):
        box("front_lamp_" + ("left" if side > 0 else "right"),
            (0.391, side * 0.112, 0.105), (0.008, 0.070, 0.026),
            amber, 0.004)
    box("front_safety_band", (0.393, 0.0, 0.063),
        (0.006, 0.340, 0.010), mint, 0.002)

    # Side rails and service seams explain the body structure without adding
    # floating parts. They stay inside the existing y=.291 side limit.
    for side in (-1.0, 1.0):
        label = "left" if side > 0 else "right"
        box("side_rail_" + label, (0.010, side * 0.278, 0.030),
            (0.520, 0.026, 0.060), alloy, 0.007)
        box("side_status_" + label, (-0.035, side * 0.253, 0.102),
            (0.180, 0.006, 0.014), mint, 0.002)
        box("side_seam_" + label, (-0.180, side * 0.254, 0.046),
            (0.100, 0.005, 0.008), graphite, 0.001)

    # Rear service language: a dark bay, two mint latches, and an amber
    # maintenance marker are visible from the follow camera's approach side.
    box("rear_bay", (-0.365, 0.0, 0.102), (0.006, 0.260, 0.070),
        alloy, 0.002)
    for side in (-1.0, 1.0):
        box("rear_latch_" + ("left" if side > 0 else "right"),
            (-0.369, side * 0.090, 0.102), (0.004, 0.026, 0.018),
            mint, 0.002)
    box("rear_marker", (-0.369, 0.0, 0.058), (0.004, 0.080, 0.010),
        amber, 0.001)

    # A small bearing witness on the top makes the LiDAR link readable while
    # remaining below the chassis manifest maximum z.
    cylinder("sensor_bearing", (0.0, 0.0, 0.205), 0.026, 0.005,
             alloy, 12)
    torus("sensor_status_ring", (0.0, 0.0, 0.2045), 0.024, 0.003, mint)
    export(MODELS / "robots" / "ro1" / "chassis.glb")


def build_ro1_lidar():
    reset()
    graphite = material("ro1_hero_lidar_graphite", (0.025, 0.045, 0.047),
                       0.58, 0.25)
    glass = material("ro1_hero_lidar_glass", (0.035, 0.16, 0.18), 0.30, 0.11)
    mint = material("ro1_hero_lidar_mint", (0.05, 0.82, 0.61), 0.08, 0.18,
                    (0.02, 0.44, 0.25))
    alloy = material("ro1_hero_lidar_alloy", (0.32, 0.48, 0.48), 0.78, 0.19)

    # Exact lidar envelope: x/y +/- .110 m, z [-.020,.105] m.
    cylinder("lidar_base", (0.0, 0.0, -0.010), 0.110, 0.020,
             graphite, 8)
    cylinder("lidar_body", (0.0, 0.0, 0.022), 0.102, 0.064,
             glass, 8)
    cylinder("lidar_scan_band", (0.0, 0.0, 0.052), 0.105, 0.010,
             mint, 8)
    cylinder("lidar_cap", (0.0, 0.0, 0.073), 0.090, 0.032,
             graphite, 8)
    torus("lidar_status_ring", (0.0, 0.0, 0.091), 0.078, 0.006, mint)
    box("lidar_top_mark", (0.0, 0.0, 0.097), (0.040, 0.040, 0.016),
        alloy, 0.004)
    export(MODELS / "robots" / "ro1" / "lidar_standard.glb")


def main():
    build_ro1_chassis()
    build_ro1_lidar()
    reset()
    print("RO1_HERO_ASSETS_OK")


if __name__ == "__main__":
    main()
