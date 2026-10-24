"""Author the SIM presentation GLBs without changing physics frames.

Run: /disk/blender-5.2.2-linux-x64/blender --background --python tools/modeling/refine_assets.py
MuJoCo owns collision and joints; this only changes visual materials/details.
"""
import math
from pathlib import Path
import bpy

ROOT = Path(__file__).resolve().parents[2]
MODELS = ROOT / "assets" / "models"

def clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)

def import_glb(path):
    clear()
    bpy.ops.import_scene.gltf(filepath=str(path))
    [bpy.data.objects.remove(o, do_unlink=True) for o in list(bpy.context.scene.objects)
     if o.type in {"CAMERA", "LIGHT"} or o.name == "Cube"]

def export_glb(path):
    bpy.ops.export_scene.gltf(filepath=str(path), export_format="GLB",
                              use_selection=False, export_apply=False,
                              export_yup=False)

def mat(name, color, metallic=0.0, rough=0.45, emission=None):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*color, 1.0)
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Roughness"].default_value = rough
    if emission is not None:
        b.inputs["Emission Color"].default_value = (*emission, 1.0)
        b.inputs["Emission Strength"].default_value = 2.4
    m.diffuse_color = (*color, 1.0)
    return m

def assign(name, material):
    o = bpy.data.objects.get(name)
    if o is not None and o.type == "MESH":
        o.data.materials.clear()
        o.data.materials.append(material)

def bevel(name, width):
    o = bpy.data.objects.get(name)
    if o is None or o.type != "MESH":
        return
    mod = o.modifiers.new("soft_edge", "BEVEL")
    mod.width, mod.segments, mod.limit_method = width, 3, "ANGLE"
    mod.angle_limit = math.radians(28.0)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.modifier_apply(modifier=mod.name)

def cylinder(name, loc, radius, depth, axis, material, vertices=32):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius,
                                        depth=depth, location=loc)
    o = bpy.context.object
    o.name = name
    o.rotation_euler = {"X": (0.0, math.pi / 2.0, 0.0),
                        "Y": (math.pi / 2.0, 0.0, 0.0),
                        "Z": (0.0, 0.0, 0.0)}.get(axis, (0.0, 0.0, 0.0))
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    assign(name, material)
    return o

def cube(name, loc, scale, material, bevel_width=0.004):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=loc)
    o = bpy.context.object
    o.name, o.scale = name, scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(name, material)
    if bevel_width > 0:
        bevel(name, bevel_width)
    return o

def refine_chassis():
    path = MODELS / "robots" / "ro1" / "chassis.glb"
    import_glb(path)
    pearl = mat("ro1_pearl", (0.82, 0.87, 0.85), 0.08, 0.30)
    shell = mat("ro1_shell", (0.60, 0.68, 0.65), 0.10, 0.36)
    graphite = mat("ro1_graphite", (0.035, 0.055, 0.052), 0.0, 0.38)
    slate = mat("ro1_slate", (0.17, 0.24, 0.22), 0.24, 0.30)
    mint = mat("ro1_mint", (0.25, 0.86, 0.70), 0.26, 0.25)
    amber = mat("ro1_amber", (0.98, 0.64, 0.18), 0.0, 0.30, (0.95, 0.40, 0.05))
    for n in ("lower_skirt", "top_deck", "face_panel", "eye_bezel_left",
              "eye_bezel_right"):
        assign(n, graphite)
    for n in ("lower_hull",): assign(n, pearl)
    for n in ("upper_hull", "rear_service", "shoulder_left", "shoulder_right"):
        assign(n, shell)
    for n in ("front_bumper", "front_chin", "side_rail_left", "side_rail_right"):
        assign(n, slate)
    for n in ("front_accent", "eye_left", "eye_right", "service_latch_left",
              "service_latch_right"):
        assign(n, mint)
    assign("rear_status", amber)
    for n in ("lower_skirt", "lower_hull", "upper_hull", "top_deck",
              "front_bumper", "shoulder_left", "shoulder_right"):
        bevel(n, 0.0045)
    cube("ro1_front_skid", (0.407, -0.055, 0.0), (0.016, 0.032, 0.34), slate, 0.006)
    cube("ro1_front_pinstripe", (0.392, -0.094, 0.0), (0.008, 0.006, 0.255), mint, 0.002)
    export_glb(path)

def refine_caster():
    path = MODELS / "robots" / "ro1" / "caster_wheel.glb"
    import_glb(path)
    rubber = mat("ro1_caster_rubber", (0.035, 0.040, 0.045), 0.0, 0.58)
    steel = mat("ro1_caster_steel", (0.48, 0.56, 0.54), 0.72, 0.22)
    assign("caster_tire", rubber); assign("caster_rim", steel); assign("caster_axle", steel)
    bevel("caster_tire", 0.004)
    cylinder("caster_crown_highlight", (0.0, 0.0, 0.0), 0.0405, 0.025, "Y", rubber)
    cylinder("caster_hub_cap", (0.0, -0.039, 0.0), 0.016, 0.008, "Y", steel, 24)
    export_glb(path)

def refine_lidar():
    path = MODELS / "robots" / "ro1" / "lidar_standard.glb"
    import_glb(path)
    graphite = mat("ro1_lidar_graphite", (0.035, 0.052, 0.050), 0.0, 0.30)
    glass = mat("ro1_lidar_glass", (0.025, 0.10, 0.12), 0.25, 0.08)
    mint = mat("ro1_lidar_mint", (0.26, 0.90, 0.73), 0.0, 0.20, (0.10, 0.55, 0.38))
    assign("base", graphite); assign("scanner", glass); assign("scan_bar", mint)
    cylinder("lidar_status_ring", (0.0, 0.0, 0.071), 0.073, 0.007, "Z", mint, 40)
    export_glb(path)

def refine_actors():
    path = MODELS / "actors" / "forklift.glb"
    import_glb(path)
    orange = mat("actor_forklift_orange", (0.92, 0.43, 0.10), 0.0, 0.34)
    dark = mat("actor_forklift_dark", (0.06, 0.10, 0.10), 0.05, 0.38)
    yellow = mat("actor_forklift_yellow", (0.98, 0.75, 0.22), 0.0, 0.30)
    for n in ("body",): assign(n, orange)
    for n in ("cab", "cab_roof", "chassis"): assign(n, dark)
    assign("stripe", yellow)
    export_glb(path)
    path = MODELS / "actors" / "worker.glb"
    import_glb(path)
    blue = mat("actor_worker_blue", (0.22, 0.39, 0.48), 0.0, 0.42)
    skin = mat("actor_worker_skin", (0.68, 0.51, 0.39), 0.0, 0.48)
    cap = mat("actor_worker_cap", (0.96, 0.72, 0.20), 0.0, 0.34)
    for n in ("torso", "arm_left", "arm_right", "leg_left", "leg_right"): assign(n, blue)
    assign("head", skin); assign("cap", cap)
    export_glb(path)

def main():
    refine_chassis(); refine_caster(); refine_lidar(); refine_actors(); clear()
    print("model assets refined")

if __name__ == "__main__":
    main()
