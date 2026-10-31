"""Build modular forklift and worker presentation assets.

Run:
 /disk/blender-5.2.2-linux-x64/blender --background \
 --python tools/modeling/build_actor_assets.py

The actor root stays at the physical ground contact origin in canonical
metres (+X forward, +Y left, +Z up). Forklift parts remain separate objects
so future MuJoCo wheel, steer, mast, and carriage links can bind them without
remodelling. The worker contains a named visual skeleton and a deterministic
walk clip; the SIM actor root remains the sole source of world pose.
"""

from pathlib import Path
import math

import bpy
from mathutils import Matrix, Vector


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "assets" / "models" / "actors"


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def material(name, color, metallic=0.0, roughness=0.45, emission=None):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    shader = result.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1.0)
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    if emission is not None:
        shader.inputs["Emission Color"].default_value = (*emission, 1.0)
        shader.inputs["Emission Strength"].default_value = 2.6
    result.diffuse_color = (*color, 1.0)
    return result


def assign(obj, surface):
    obj.data.materials.append(surface)


def bevel(obj, width=0.008, segments=2):
    modifier = obj.modifiers.new("manufactured_edge", "BEVEL")
    modifier.width = width
    modifier.segments = segments
    modifier.limit_method = "ANGLE"
    modifier.angle_limit = math.radians(24.0)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=modifier.name)


def box(name, location, dimensions, surface, bevel_width=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(obj, surface)
    if bevel_width > 0.0:
        bevel(obj, bevel_width)
    return obj


def cylinder(name, location, radius, depth, axis, surface, vertices=24):
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


def torus(name, location, major_radius, minor_radius, surface,
          major_segments=12, minor_segments=4, normal_axis="Y"):
    """Create a thin service ring in the requested wheel/link frame.

    The actor files are presentation meshes only.  Keeping the ring inside
    the existing wheel envelope makes it a useful visual witness for the
    physical joint without introducing a second collision or animation
    source.
    """
    rotation = {
        "X": (0.0, math.pi / 2.0, 0.0),
        "Y": (math.pi / 2.0, 0.0, 0.0),
        "Z": (0.0, 0.0, 0.0),
    }[normal_axis]
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_radius, minor_radius=minor_radius,
        major_segments=major_segments, minor_segments=minor_segments,
        location=location, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    assign(obj, surface)
    return obj


def uv_sphere(name, location, scale, surface):
    bpy.ops.mesh.primitive_uv_sphere_add(
        segments=20, ring_count=10, radius=1.0, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(obj, surface)
    return obj


def export_glb(filename, selected=None, origin=None):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if selected is not None: [obj.select_set(False) for obj in bpy.context.scene.objects]
    if selected is not None: [obj.select_set(True) for obj in selected]
    if selected: bpy.context.view_layer.objects.active = selected[0]
    offset = Matrix.Translation(-Vector(origin)) if origin is not None else None
    restore = [(obj, obj.matrix_world.copy()) for obj in selected or []]
    if offset is not None: [setattr(obj, "matrix_world", offset @ obj.matrix_world) for obj in selected or []]
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT / filename), export_format="GLB",
        use_selection=selected is not None, export_apply=False, export_yup=False)
    [setattr(obj, "matrix_world", matrix) for obj, matrix in restore]


def build_forklift():
    reset_scene()
    orange = material("forklift_safety_orange", (0.93, 0.34, 0.055), 0.15, 0.30)
    graphite = material("forklift_graphite", (0.035, 0.052, 0.052), 0.30, 0.32)
    steel = material("forklift_steel", (0.31, 0.38, 0.38), 0.78, 0.20)
    rubber = material("forklift_rubber", (0.020, 0.026, 0.027), 0.0, 0.68)
    glass = material("forklift_glass", (0.055, 0.19, 0.22), 0.20, 0.10)
    cream = material("forklift_seat", (0.60, 0.58, 0.49), 0.0, 0.62)
    amber = material(
        "forklift_beacon", (1.0, 0.52, 0.06), 0.0, 0.18,
        (1.0, 0.24, 0.015))
    mint = material(
        "forklift_identity", (0.08, 0.82, 0.67), 0.10, 0.22,
        (0.02, 0.32, 0.22))

    box("forklift_chassis", (-0.04, 0.0, 0.21),
        (0.92, 0.56, 0.22), graphite, 0.025)
    box("forklift_counterweight", (-0.39, 0.0, 0.43),
        (0.34, 0.62, 0.48), orange, 0.07)
    box("forklift_hood", (-0.11, 0.0, 0.52),
        (0.50, 0.54, 0.22), orange, 0.035)
    box("forklift_floor", (0.10, 0.0, 0.48),
        (0.42, 0.52, 0.055), steel, 0.012)
    box("forklift_seat_base", (-0.04, 0.0, 0.62),
        (0.22, 0.28, 0.08), graphite, 0.018)
    box("forklift_seat", (-0.08, 0.0, 0.73),
        (0.18, 0.30, 0.16), cream, 0.035)
    seat_back = box("forklift_seat_back", (-0.18, 0.0, 0.86),
                    (0.08, 0.32, 0.32), cream, 0.032)
    seat_back.rotation_euler[1] = math.radians(-7.0)
    cylinder("forklift_steering_column", (0.20, 0.0, 0.70),
             0.020, 0.28, "Z", steel, 16).rotation_euler[1] = math.radians(22.0)
    cylinder("forklift_steering_wheel", (0.25, 0.0, 0.83),
             0.085, 0.018, "X", graphite, 20)

    for side in (-1.0, 1.0):
        suffix = "left" if side > 0 else "right"
        box(f"forklift_guard_front_{suffix}", (0.24, side * 0.255, 0.83),
            (0.055, 0.055, 0.64), steel, 0.008)
        box(f"forklift_guard_rear_{suffix}", (-0.24, side * 0.255, 0.83),
            (0.055, 0.055, 0.64), steel, 0.008)
        box(f"forklift_guard_rail_{suffix}", (0.0, side * 0.255, 1.06),
            (0.52, 0.045, 0.045), steel, 0.007)
    box("forklift_guard_roof", (0.0, 0.0, 1.12),
        (0.57, 0.58, 0.07), graphite, 0.018)
    box("forklift_windshield", (0.255, 0.0, 0.88),
        (0.018, 0.43, 0.29), glass, 0.006)

    # Compact headlamps and a rear status lens make the vehicle orientation
    # obvious in the teaching view.  They stay within the authored base
    # envelope and are rendered from the same MuJoCo base body pose.
    for side in (-1.0, 1.0):
        suffix = "left" if side > 0 else "right"
        box(f"forklift_headlamp_{suffix}", (0.205, side * 0.235, 0.59),
            (0.018, 0.038, 0.052), mint, 0.0)
        box(f"forklift_rear_lens_{suffix}", (-0.555, side * 0.235, 0.48),
            (0.008, 0.030, 0.038), amber, 0.0)

    # Named modules preserve the future joint boundary: rear steer wheels,
    # front driven wheels, mast, carriage, and forks remain independent.
    for x, radius, role in ((-0.30, 0.135, "rear_steer"),
                            (0.28, 0.165, "front_drive")):
        for side in (-1.0, 1.0):
            suffix = "left" if side > 0 else "right"
            wheel = cylinder(
                f"forklift_{role}_wheel_{suffix}",
 (x, side * (0.35 - 0.046), radius),
                radius, 0.09, "Y", rubber, 28)
            bevel(wheel, 0.007, 2)
            cylinder(
                f"forklift_{role}_hub_{suffix}",
 (x, side * (0.35 - 0.048), radius),
                radius * 0.48, 0.096, "Y", steel, 20)
            # A shallow sidewall ring reads as a tire sidewall at gameplay
            # distance and remains well inside the measured wheel envelope.
            torus(f"forklift_{role}_sidewall_{suffix}",
                  (x, side * (0.35 - 0.048), radius), radius * 0.70,
                  0.004, steel, 8, 3, "Y")

    for side in (-1.0, 1.0):
        suffix = "left" if side > 0 else "right"
        box(f"forklift_mast_rail_{suffix}", (0.47, side * 0.22, 0.63),
            (0.075, 0.07, 1.06), graphite, 0.010)
        box(f"forklift_inner_mast_{suffix}", (0.525, side * 0.13, 0.63),
            (0.045, 0.045, 0.94), steel, 0.006)
    box("forklift_mast_crossbar", (0.50, 0.0, 1.02),
        (0.10, 0.51, 0.07), orange, 0.008)
    box("forklift_carriage", (0.57, 0.0, 0.40),
        (0.08, 0.50, 0.22), steel, 0.012)
    for side in (-1.0, 1.0):
        suffix = "left" if side > 0 else "right"
        box(f"forklift_fork_{suffix}", (0.875, side * 0.17, 0.075),
            (0.60, 0.075, 0.075), steel, 0.012)
        box(f"forklift_fork_heel_{suffix}", (0.59, side * 0.17, 0.24),
            (0.075, 0.075, 0.37), steel, 0.010)
        box(f"forklift_fork_warning_{suffix}", (0.88, side * 0.17, 0.118),
            (0.16, 0.006, 0.009), amber, 0.0)

    box("forklift_identity_strip", (-0.16, -0.316, 0.46),
        (0.38, 0.012, 0.035), mint, 0.004)
    box("forklift_beacon", (-0.02, 0.0, 1.165),
        (0.075, 0.075, 0.03), amber, 0.009)
    export_glb("forklift.glb")

    # The complete actor remains a fallback, while these authored modules bind
    # to MuJoCo child-body poses. Each export is translated into its link pivot
    # once, so the renderer does not add draw-time offsets.
    objects = list(bpy.context.scene.objects)
    drive_all = [obj for obj in objects if obj.name.startswith("forklift_front_drive_")]
    drive = [obj for obj in drive_all if obj.name.endswith("_left")]
    steer_all = [obj for obj in objects if obj.name.startswith("forklift_rear_steer_")]
    steer = [obj for obj in steer_all if obj.name.endswith("_left")]
    mast = [obj for obj in objects if obj.name.startswith((
        "forklift_mast_", "forklift_inner_mast_"))]
    carriage = [obj for obj in objects if obj.name == "forklift_carriage"]
    fork_all = [obj for obj in objects if obj.name.startswith("forklift_fork_")]
    fork = [obj for obj in fork_all if obj.name.endswith("_left")]
    modules = set(drive_all + steer_all + mast + carriage + fork_all)
    base = [obj for obj in objects if obj not in modules]
    export_glb("forklift_base.glb", base, (0.0, 0.0, 0.0))
    export_glb("forklift_drive_wheel.glb", drive, (0.28, 0.304, 0.165))
    export_glb("forklift_steer_wheel.glb", steer, (-0.30, 0.304, 0.135))
    export_glb("forklift_mast.glb", mast, (0.47, 0.0, 0.0))
    export_glb("forklift_carriage.glb", carriage, (0.57, 0.0, 0.40))
    export_glb("forklift_fork.glb", fork, (0.875, 0.17, 0.075))


def create_worker_rig():
    bpy.ops.object.armature_add(enter_editmode=True, location=(0.0, 0.0, 0.0))
    rig = bpy.context.object
    rig.name = "worker_rig"
    armature = rig.data
    armature.name = "worker_skeleton"
    root = armature.edit_bones[0]
    root.name = "root"
    root.head = (0.0, 0.0, 0.0)
    root.tail = (0.0, 0.0, 0.12)

    def bone(name, head, tail, parent):
        result = armature.edit_bones.new(name)
        result.head = head
        result.tail = tail
        result.parent = parent
        return result

    pelvis = bone("pelvis", (0.0, 0.0, 0.58), (0.0, 0.0, 0.72), root)
    torso = bone("torso", (0.0, 0.0, 0.70), (0.0, 0.0, 1.04), pelvis)
    head = bone("head", (0.0, 0.0, 1.02), (0.0, 0.0, 1.24), torso)
    leg_left = bone("leg_left", (0.0, 0.075, 0.61),
                    (0.0, 0.075, 0.09), pelvis)
    leg_right = bone("leg_right", (0.0, -0.075, 0.61),
                     (0.0, -0.075, 0.09), pelvis)
    arm_left = bone("arm_left", (0.0, 0.13, 0.96),
                    (0.0, 0.17, 0.52), torso)
    arm_right = bone("arm_right", (0.0, -0.13, 0.96),
                     (0.0, -0.17, 0.52), torso)
    bpy.ops.object.mode_set(mode="POSE")

    action = bpy.data.actions.new("WorkerWalk")
    rig.animation_data_create()
    rig.animation_data.action = action
    for frame, angle in ((0, -0.40), (10, 0.40), (20, -0.40)):
        for name, sign in (("leg_left", 1.0), ("leg_right", -1.0),
                           ("arm_left", -0.75), ("arm_right", 0.75)):
            pose = rig.pose.bones[name]
            pose.rotation_mode = "XYZ"
            pose.rotation_euler[1] = angle * sign
            pose.keyframe_insert("rotation_euler", frame=frame)
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = 20
    bpy.ops.object.mode_set(mode="OBJECT")
    return rig


def bind_rigid(obj, rig, bone_name):
 group = obj.vertex_groups.new(name=bone_name)
 group.add(range(len(obj.data.vertices)), 1.0, "REPLACE")
 modifier = obj.modifiers.new("worker_skin", "ARMATURE")
 modifier.object = rig
 obj.parent = rig
 obj.matrix_parent_inverse = rig.matrix_world.inverted()


def build_worker():
    reset_scene()
    navy = material("worker_navy", (0.045, 0.10, 0.16), 0.0, 0.50)
    blue = material("worker_blue", (0.08, 0.36, 0.52), 0.0, 0.40)
    hi_vis = material(
        "worker_hi_vis", (0.82, 0.92, 0.16), 0.0, 0.34,
        (0.15, 0.18, 0.01))
    skin = material("worker_skin", (0.62, 0.40, 0.27), 0.0, 0.52)
    graphite = material("worker_graphite", (0.035, 0.05, 0.055), 0.1, 0.50)
    helmet = material("worker_helmet", (0.98, 0.62, 0.06), 0.0, 0.28)
    rig = create_worker_rig()

    parts = []
    parts.append((box("worker_torso", (0.0, 0.0, 0.83),
                      (0.26, 0.30, 0.42), blue, 0.045), "torso"))
    parts.append((box("worker_vest_front", (0.134, 0.0, 0.84),
                      (0.012, 0.25, 0.30), hi_vis, 0.004), "torso"))
    parts.append((box("worker_belt", (0.0, 0.0, 0.63),
                      (0.27, 0.31, 0.055), graphite, 0.010), "pelvis"))
    parts.append((uv_sphere("worker_head", (0.0, 0.0, 1.13),
                            (0.105, 0.10, 0.12), skin), "head"))
    parts.append((box("worker_helmet", (0.0, 0.0, 1.255),
                      (0.25, 0.23, 0.06), helmet, 0.025), "head"))
    parts.append((box("worker_helmet_brim", (0.075, 0.0, 1.225),
                      (0.17, 0.25, 0.025), helmet, 0.010), "head"))

    for side in (-1.0, 1.0):
        suffix = "left" if side > 0 else "right"
        leg_bone = f"leg_{suffix}"
        arm_bone = f"arm_{suffix}"
        parts.append((box(f"worker_leg_{suffix}", (0.0, side * 0.075, 0.35),
                          (0.14, 0.13, 0.50), navy, 0.035), leg_bone))
        parts.append((box(f"worker_boot_{suffix}", (0.045, side * 0.075, 0.07),
                          (0.23, 0.14, 0.14), graphite, 0.025), leg_bone))
        parts.append((box(f"worker_arm_{suffix}", (0.0, side * 0.17, 0.76),
                          (0.12, 0.09, 0.42), blue, 0.035), arm_bone))
        parts.append((uv_sphere(f"worker_hand_{suffix}",
                                (0.0, side * 0.17, 0.52),
                                (0.065, 0.055, 0.075), skin), arm_bone))
        parts.append((box(f"worker_vest_stripe_{suffix}",
                          (0.136, side * 0.08, 0.80),
                          (0.010, 0.045, 0.20), hi_vis, 0.003), "torso"))

    # Exact contract extents remain visible geometry, not empty bounds helpers.
    parts.append((box("worker_chest_badge", (0.145, 0.0, 0.94),
                      (0.030, 0.08, 0.08), hi_vis, 0.005), "torso"))
    parts.append((box("worker_radio", (-0.145, -0.12, 0.91),
                      (0.030, 0.07, 0.15), graphite, 0.008), "torso"))
    for obj, bone_name in parts:
        bind_rigid(obj, rig, bone_name)
    export_glb("worker.glb")


def main():
    build_forklift()
    build_worker()
    reset_scene()
    print("modular actor assets generated")


if __name__ == "__main__":
    main()
