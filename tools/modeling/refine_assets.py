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
    # This tool imports an existing glTF first. Re-enable Blender's glTF Y-up
    # conversion on export so repeated refinement preserves the file frame.
    # Generators that start from a fresh canonical Z-up Blender scene use
    # export_yup=False instead.
        export_yup=True)

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
    existing = bpy.data.objects.get(name)
    if existing is not None:
        bpy.data.objects.remove(existing, do_unlink=True)
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
    existing = bpy.data.objects.get(name)
    if existing is not None:
        bpy.data.objects.remove(existing, do_unlink=True)
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=loc)
    o = bpy.context.object
    o.name, o.scale = name, scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(name, material)
    if bevel_width > 0:
        bevel(name, bevel_width)
    return o

def join_meshes(names, output_name):
 objects = [bpy.data.objects.get(name) for name in names]
 objects = [obj for obj in objects if obj is not None and obj.type == "MESH"]
 if not objects:
  return None
 bpy.ops.object.select_all(action="DESELECT")
 for obj in objects:
  obj.select_set(True)
 bpy.context.view_layer.objects.active = objects[0]
 bpy.ops.object.join()
 objects[0].name = output_name
 return objects[0]

def refine_drive_wheel():
    """Add readable traction and rotation cues without changing the link frame.

    The visual radius remains inside the configured 0.10 m contact radius. The
    bright witnesses make true MuJoCo roll obvious in a still frame; tread blocks
    describe grip but never become a second collision model.
    """
    path = MODELS / "robots" / "ro1" / "wheel_standard.glb"
    clear()
    rubber = mat("ro1_tire_rubber", (0.025, 0.032, 0.034), 0.0, 0.64)
    tread = mat("ro1_tire_tread", (0.055, 0.066, 0.066), 0.0, 0.56)
    alloy = mat("ro1_wheel_alloy", (0.38, 0.49, 0.48), 0.72, 0.22)
    graphite = mat("ro1_wheel_graphite", (0.055, 0.075, 0.074), 0.28, 0.34)
    mint = mat("ro1_wheel_mint", (0.23, 0.90, 0.72), 0.18, 0.20,
    (0.05, 0.50, 0.31))
    cylinder("tire", (0.0, 0.0, 0.0), 0.1, 0.1, "Y", rubber, 32)
    cylinder("rim", (0.0, 0.0, 0.0), 0.066, 0.106, "Y", alloy, 32)
    cylinder("hub", (0.0, 0.0, 0.0), 0.031, 0.112, "Y", graphite, 32)
    cube("rotation_witness_right", (0.058, 0.056, 0.0),
        (0.028, 0.010, 0.040), mint, 0.0)
    cube("rotation_witness_left", (0.058, -0.056, 0.0),
        (0.028, 0.010, 0.040), mint, 0.0)

    # Eighteen low-profile tread blocks stay below the declared contact radius.
    # They form one draw mesh after authoring and provide a clear rolling cadence.
    blocks = []
    for index in range(18):
        angle = 2.0 * math.pi * index / 18.0
        name = f"traction_block_{index:02d}"
        radius = 0.0955
        block = cube(
            name, (radius * math.cos(angle), 0.0, radius * math.sin(angle)),
            (0.0060, 0.086, 0.018), tread, 0.0)
        block.rotation_euler[1] = -angle
        blocks.append(name)
    join_meshes(blocks, "traction_blocks")

    # Sidewall ribs and six fasteners give scale at close range. They sit inside
    # the existing rotation witnesses, so the authored bounds remain unchanged.
    rings = []
    for side in (-1.0, 1.0):
        name = f"sidewall_ring_{'left' if side > 0 else 'right'}"
        bpy.ops.mesh.primitive_torus_add(
            major_radius=0.076, minor_radius=0.0032,
            major_segments=24, minor_segments=4,
            location=(0.0, side * 0.0515, 0.0),
            rotation=(math.pi / 2.0, 0.0, 0.0))
        bpy.context.object.name = name
        bpy.ops.object.transform_apply(
            location=False, rotation=True, scale=True)
        assign(name, graphite)
        rings.append(name)
    join_meshes(rings, "sidewall_rings")

    bolts = []
    for side in (-1.0, 1.0):
        for index in range(3):
            angle = (2.0 * math.pi * index / 3.0 +
                (math.pi / 3.0 if side < 0 else 0.0))
            name = f"hub_bolt_{'left' if side > 0 else 'right'}_{index}"
            cylinder(
                name,
                (0.021 * math.cos(angle), side * 0.058,
                0.021 * math.sin(angle)),
                0.0042, 0.004, "Y", alloy, 12)
            bolts.append(name)
    join_meshes(bolts, "hub_fasteners")
    export_glb(path)

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
    # Keep the cosmetic skid inside the authored chassis envelope; the
    # manifest bounds describe render geometry and are also used by the
    # grounding gate.
    cube("ro1_front_skid", (0.404, -0.055, 0.0), (0.016, 0.032, 0.34), slate, 0.0)
    cube("ro1_front_pinstripe", (0.392, -0.094, 0.0), (0.008, 0.006, 0.255), mint, 0.0)
    export_glb(path)

def refine_caster():
    path = MODELS / "robots" / "ro1" / "caster_wheel.glb"
    import_glb(path)
    rubber = mat("ro1_caster_rubber", (0.035, 0.040, 0.045), 0.0, 0.58)
    steel = mat("ro1_caster_steel", (0.48, 0.56, 0.54), 0.72, 0.22)
    # Keep the visual tire congruent with the MuJoCo caster compound: a
    # nominal-radius centre rib carries the normal load and two smaller
    # shoulders engage only under lean/side load.  The old flat cylinder was
    # visually wider than the physical contact patch and made a caster look
    # embedded when the wheel was viewed from the side.
    old_tire = bpy.data.objects.get("caster_tire")
    if old_tire is not None:
        bpy.data.objects.remove(old_tire, do_unlink=True)
    reference_radius = 0.055
    reference_half_width = 0.048
    crown_half_width = reference_half_width * 0.5
    shoulder_half_width = reference_half_width * 0.25
    shoulder_offset = crown_half_width + shoulder_half_width
    shoulder_radius = reference_radius * 0.90
    cylinder("caster_tire", (0.0, 0.0, 0.0), reference_radius,
             2.0 * crown_half_width, "Y", rubber, 12)
    cylinder("caster_tire_shoulder_left", (0.0, shoulder_offset, 0.0),
             shoulder_radius, 2.0 * shoulder_half_width, "Y", rubber, 12)
    cylinder("caster_tire_shoulder_right", (0.0, -shoulder_offset, 0.0),
             shoulder_radius, 2.0 * shoulder_half_width, "Y", rubber, 12)
    # The 24-sided cylinders are already smooth enough at the caster's
    # presentation scale.  Avoid a bevel modifier here: it multiplies the
    # tiny tire mesh beyond the manifest's close-up triangle budget without
    # improving the silhouette.
    for old_name in ("caster_rim", "caster_axle"):
        old = bpy.data.objects.get(old_name)
        if old is not None:
            bpy.data.objects.remove(old, do_unlink=True)
    # Low-poly metal inserts keep the caster readable at gameplay distance
    # while leaving room in the strict wheel mesh budget for the crown.
    cylinder("caster_rim", (0.0, 0.0, 0.0), reference_radius * 0.65,
             reference_half_width * 1.08, "Y", steel, 12)
    cylinder("caster_axle", (0.0, 0.0, 0.0), reference_radius * 0.24,
             reference_half_width * 1.25, "Y", steel, 12)
    cylinder("caster_hub_cap", (0.0, -reference_half_width * 0.8125, 0.0),
             reference_radius * 0.29, reference_half_width / 6.0,
             "Y", steel, 12)
    export_glb(path)

def refine_lidar():
    path = MODELS / "robots" / "ro1" / "lidar_standard.glb"
    import_glb(path)
    graphite = mat("ro1_lidar_graphite", (0.035, 0.052, 0.050), 0.0, 0.30)
    glass = mat("ro1_lidar_glass", (0.025, 0.10, 0.12), 0.25, 0.08)
    mint = mat("ro1_lidar_mint", (0.26, 0.90, 0.73), 0.0, 0.20, (0.10, 0.55, 0.38))
    assign("base", graphite); assign("scanner", glass); assign("scan_bar", mint)
    # A compact 16-sided ring is sufficient at the sensor's scale and keeps
    # the close-up asset within its declared triangle budget.
    cylinder("lidar_status_ring", (0.0, -0.071, 0.0), 0.073, 0.007, "Y", mint, 16)
    export_glb(path)

def main():
 refine_drive_wheel(); refine_chassis(); refine_caster(); refine_lidar()
 clear()
 print("model assets refined")

if __name__ == "__main__":
    main()
