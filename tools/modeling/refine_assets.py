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

def soften_imported_box(name, width=0.008):
    """Round one authored box once, without expanding its declared envelope.

    The legacy RO-1 chassis is intentionally low-poly.  A single-segment
    bevel makes the large service panels catch the scene light without adding
    another physics shape or pushing the asset outside its manifest bounds.
    The polygon-count guard keeps this refinement idempotent when the script
    is run again on the already exported GLB.
    """
    o = bpy.data.objects.get(name)
    if o is None or o.type != "MESH" or len(o.data.polygons) > 12:
        return
    mod = o.modifiers.new("ro1_soft_edge", "BEVEL")
    mod.width, mod.segments, mod.limit_method = width, 1, "NONE"
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.modifier_apply(modifier=mod.name)

def restore_box_mesh(name):
    """Collapse a prior beauty-pass bevel back to its authored box.

    Blender's glTF importer may rename generated panel objects, so rerunning
    the script must also clean any old detail meshes by material (handled in
    ``refine_chassis`` below).  Rebuilding only the nine known legacy boxes
    keeps the pass deterministic even after an interrupted export.
    """
    o = bpy.data.objects.get(name)
    if o is None or o.type != "MESH" or len(o.data.polygons) <= 12:
        return
    corners = [o.matrix_world @ v.co for v in o.data.vertices]
    lo = tuple(min(v[i] for v in corners) for i in range(3))
    hi = tuple(max(v[i] for v in corners) for i in range(3))
    material = o.data.materials[0] if o.data.materials else None
    bpy.data.objects.remove(o, do_unlink=True)
    cube(name, tuple((lo[i] + hi[i]) * 0.5 for i in range(3)),
         tuple(hi[i] - lo[i] for i in range(3)), material, 0.0)

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

    # A narrow crowned centre rib makes the load-bearing tread legible from
    # the side without changing the declared wheel envelope.  The physical
    # contact patch remains MuJoCo's tire/collision geometry; this is a
    # presentation witness only.  Keep the ring inside radius=0.100 m and
    # below the wheel triangle budget so every authored variant stays cheap.
    bpy.ops.mesh.primitive_torus_add(
        major_radius=0.097, minor_radius=0.003,
        major_segments=12, minor_segments=4,
        location=(0.0, 0.0, 0.0), rotation=(math.pi / 2.0, 0.0, 0.0))
    crown = bpy.context.object
    crown.name = "crowned_tread_rib"
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    assign(crown.name, tread)

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
    pearl = mat("ro1_pearl", (0.86, 0.90, 0.88), 0.08, 0.28)
    shell = mat("ro1_shell", (0.58, 0.68, 0.65), 0.10, 0.34)
    graphite = mat("ro1_graphite", (0.024, 0.040, 0.042), 0.0, 0.34)
    slate = mat("ro1_slate", (0.12, 0.19, 0.19), 0.24, 0.28)
    mint = mat("ro1_mint", (0.20, 0.92, 0.72), 0.26, 0.22)
    amber = mat("ro1_amber", (0.98, 0.64, 0.18), 0.0, 0.30, (0.95, 0.40, 0.05))
    top_glass = mat("ro1_top_glass", (0.010, 0.060, 0.068), 0.30, 0.16,
                    (0.015, 0.12, 0.13))
    top_status = mat("ro1_top_status", (0.20, 0.94, 0.74), 0.12, 0.18,
                     (0.08, 0.72, 0.46))
    soft_boxes = (
        "top_deck", "front_bumper", "front_chin", "face_panel",
        "side_rail_left", "side_rail_right", "shoulder_left",
        "shoulder_right", "rear_service",
    )
    for n in soft_boxes:
        restore_box_mesh(n)
    # Remove panels from a previous run even when Blender assigned them a
    # generated name such as Cube.002 during glTF import.
    for o in list(bpy.context.scene.objects):
        if o.type == "MESH" and any(
            m and m.name in {"ro1_top_glass", "ro1_top_status"}
            for m in o.data.materials
        ):
            bpy.data.objects.remove(o, do_unlink=True)
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

    # Keep the authored silhouette and soften only the five high-visibility
    # panels.  A single segment is enough for a clean specular break while
    # remaining well below the chassis' raised presentation budget.
    for n, width in (
        ("top_deck", 0.012), ("front_bumper", 0.010),
        ("front_chin", 0.010), ("face_panel", 0.007),
        ("rear_service", 0.006),
    ):
        soften_imported_box(n, width)

    # The remaining detail budget is spent on deliberate graphic surfaces.
    # They are embedded in existing panels so the vehicle reads as a designed
    # product rather than a collection of floating primitives.
    cube("ro1_top_glass", (-0.015, -0.2035, 0.0),
         (0.205, 0.004, 0.095), top_glass, 0.0)
    cube("ro1_top_status", (0.105, -0.2035, 0.0),
         (0.045, 0.004, 0.012), top_status, 0.0)
    cube("ro1_side_trim_left", (0.015, -0.078, 0.271),
         (0.105, 0.006, 0.004), top_status, 0.0)
    cube("ro1_side_trim_right", (0.015, -0.078, -0.271),
         (0.105, 0.006, 0.004), top_status, 0.0)
    cube("ro1_front_badge", (0.374, -0.072, 0.0),
         (0.006, 0.018, 0.082), amber, 0.0)
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


def refine_secondary_chassis():
    """Apply the RO product language to the other chassis families.

    Every variant gets the same functional hierarchy: a dark sensor/service
    deck, one forward identity mark and bilateral side trim.  The silhouette
    stays chassis-specific (four-wheel Ackermann, tracked, swerve and
    mecanum); these meshes only make the authored link asset read as a
    finished product.  Coordinates below are runtime X-forward/Y-left/Z-up
    and are converted to Blender's imported Z-up basis at the last moment.
    """
    variants = (
        ("ro2", "chassis.glb", "ro2_signal", (0.08, 0.82, 0.70),
         "sensor_deck_material", (
          ("ro2_front_sensor_band", (0.365, 0.0, 0.135),
           (0.008, 0.30, 0.024)),
          ("ro2_rear_status_strip", (-0.305, -0.238, 0.105),
           (0.12, 0.008, 0.020)),
          ("ro2_service_glass", (0.015, 0.0, 0.180),
           (0.140, 0.160, 0.006)),
          ("ro2_service_status", (0.105, 0.0, 0.186),
           (0.025, 0.080, 0.004)),
          ("ro2_side_trim_left", (-0.020, 0.247, 0.090),
           (0.140, 0.006, 0.008)),
          ("ro2_side_trim_right", (-0.020, -0.247, 0.090),
           (0.140, 0.006, 0.008)),
          ("ro2_front_badge", (0.375, 0.0, 0.110),
           (0.008, 0.100, 0.018)),)),
        ("ro3", "hull.glb", "ro3_signal", (0.88, 0.48, 0.10),
         "equipment_deck_material", (
          ("ro3_sensor_band", (0.285, 0.0, 0.235),
           (0.010, 0.22, 0.020)),
          ("ro3_rear_status_strip", (-0.270, -0.222, 0.125),
           (0.10, 0.008, 0.020)),
          ("ro3_service_glass", (-0.040, 0.0, 0.245),
           (0.170, 0.200, 0.006)),
          ("ro3_service_status", (0.080, 0.0, 0.252),
           (0.030, 0.120, 0.005)),
          ("ro3_side_trim_left", (-0.100, 0.248, 0.120),
           (0.180, 0.006, 0.008)),
          ("ro3_side_trim_right", (-0.100, -0.248, 0.120),
           (0.180, 0.006, 0.008)),
          ("ro3_front_badge", (0.360, 0.0, 0.120),
           (0.008, 0.120, 0.020)),)),
        ("ro4", "chassis.glb", "ro4_signal", (0.18, 0.72, 0.94),
         "lower_frame_material", (
          ("ro4_front_status", (0.250, 0.0, 0.115),
           (0.010, 0.25, 0.018)),
          ("ro4_side_status", (-0.18, -0.225, 0.095),
           (0.12, 0.008, 0.018)),
          ("ro4_service_glass", (0.0, 0.0, 0.112),
           (0.300, 0.180, 0.006)),
          ("ro4_service_status", (0.170, 0.0, 0.120),
           (0.040, 0.120, 0.005)),
          ("ro4_side_trim_left", (0.0, 0.252, 0.060),
           (0.180, 0.006, 0.008)),
          ("ro4_side_trim_right", (0.0, -0.252, 0.060),
           (0.180, 0.006, 0.008)),
          ("ro4_front_badge", (0.265, 0.0, 0.080),
           (0.008, 0.140, 0.016)),)),
        ("ro5", "chassis.glb", "ro5_signal", (0.96, 0.55, 0.16),
         "lower_frame_material", (
          ("ro5_front_status", (0.250, 0.0, 0.140),
           (0.010, 0.25, 0.018)),
          ("ro5_side_status", (-0.18, -0.225, 0.112),
           (0.12, 0.008, 0.018)),
          ("ro5_service_glass", (0.0, 0.0, 0.120),
           (0.300, 0.180, 0.006)),
          ("ro5_service_status", (0.170, 0.0, 0.128),
           (0.040, 0.120, 0.005)),
          ("ro5_side_trim_left", (0.0, 0.252, 0.070),
           (0.180, 0.006, 0.008)),
          ("ro5_side_trim_right", (0.0, -0.252, 0.070),
           (0.180, 0.006, 0.008)),
          ("ro5_front_badge", (0.265, 0.0, 0.090),
           (0.008, 0.140, 0.016)),)),
    )
    for robot, filename, material_name, color, glass_material_name, details in variants:
        path = MODELS / "robots" / robot / filename
        import_glb(path)
        signal = mat(material_name, color, 0.12, 0.24,
                     tuple(channel * 0.55 for channel in color))
        glass = bpy.data.materials.get(glass_material_name)
        if glass is None:
            raise RuntimeError(f"missing product glass material: {glass_material_name}")
        # The identity material is reserved for authored product marks.  Drop
        # previous marks before adding the current family layout so repeated
        # Blender exports stay deterministic and never double the details.
        for obj in list(bpy.context.scene.objects):
            if obj.type == "MESH" and any(
                material and material.name == material_name
                for material in obj.data.materials
            ):
                bpy.data.objects.remove(obj, do_unlink=True)
        for name, location, scale in details:
            # Existing GLBs are imported into Blender's Z-up basis.  The
            # manifest/runtime basis is (x, y, z) with +z up; convert the
            # authored detail rather than accidentally widening the vertical
            # envelope when exporting it back to glTF.
            x, y, z = location
            dx, dy, dz = scale
            surface = glass if "service_glass" in name else signal
            cube(name, (x, -z, y), (dx, dz, dy), surface, 0.0)
        export_glb(path)


def main():
 refine_drive_wheel(); refine_chassis(); refine_caster(); refine_lidar()
 refine_secondary_chassis()
 clear()
 print("model assets refined")

if __name__ == "__main__":
    main()
