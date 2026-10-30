"""Build the RO-6 arm presentation links in the MuJoCo link frame.

The physics model creates every arm capsule from its joint origin along local
X.  These assets use the same contract: x=0 is the joint pivot and +X points
to the next joint/tool.  They are presentation meshes only; MuJoCo remains
the pose and contact authority.

Run:
  /disk/blender-5.2.2-linux-x64/blender --background \
    --python tools/modeling/build_arm_assets.py
"""

from pathlib import Path
import math

import bpy


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "assets" / "models" / "robots" / "ro6"


def reset():
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
        shader.inputs["Emission Strength"].default_value = 2.2
    result.diffuse_color = (*color, 1.0)
    return result


def assign(obj, surface):
    obj.data.materials.append(surface)


def bevel(obj, width=0.004):
    modifier = obj.modifiers.new("manufactured_edge", "BEVEL")
    modifier.width = width
    modifier.segments = 2
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
    if edge > 0.0:
        bevel(obj, edge)
    return obj


def cylinder(name, location, radius, depth, axis, surface, vertices=16):
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


def export(filename):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT / filename), export_format="GLB",
        use_selection=False, export_apply=False, export_yup=False)


def common_materials(prefix):
    return {
        "graphite": material(prefix + "_graphite", (0.035, 0.055, 0.060),
                              0.45, 0.28),
        "alloy": material(prefix + "_alloy", (0.38, 0.52, 0.54),
                           0.78, 0.20),
        "mint": material(prefix + "_mint", (0.06, 0.72, 0.56),
                          0.20, 0.24, (0.02, 0.30, 0.18)),
        "polymer": material(prefix + "_polymer", (0.13, 0.20, 0.22),
                              0.10, 0.35),
        "amber": material(prefix + "_amber", (0.95, 0.42, 0.06),
                           0.05, 0.22, (0.45, 0.08, 0.01)),
    }


def build_turret():
    reset()
    m = common_materials("ro6_turret")
    # The base bearing is centred on the first joint; the link beam reaches
    # exactly 0.30 m along +X, matching ArmParams.link_length_m[0].
    cylinder("slew_base", (0.0, 0.0, 0.0), 0.105, 0.10, "Z", m["graphite"], 20)
    cylinder("slew_bearing", (0.018, 0.0, 0.0), 0.068, 0.034, "X", m["alloy"], 16)
    box("shoulder_housing", (0.085, 0.0, 0.0), (0.17, 0.18, 0.10),
        m["polymer"], 0.008)
    box("link_beam", (0.15, 0.0, 0.0), (0.30, 0.065, 0.065),
        m["graphite"], 0.006)
    cylinder("shoulder_cap", (0.055, 0.0, 0.0), 0.047, 0.012, "X", m["alloy"], 16)
    box("status_strip", (0.215, -0.034, 0.0), (0.11, 0.006, 0.018),
        m["mint"], 0.001)
    export("turret.glb")


def build_upper_arm():
    reset()
    m = common_materials("ro6_upper")
    # Exact link bounds are [0, .25] x [-.055,.055] x [-.055,.055].
    cylinder("shoulder_axis", (0.02, 0.0, 0.0), 0.055, 0.040, "X", m["alloy"], 16)
    box("beam", (0.125, 0.0, 0.0), (0.25, 0.072, 0.072),
        m["graphite"], 0.006)
    cylinder("elbow_cap", (0.225, 0.0, 0.0), 0.043, 0.020, "X", m["alloy"], 16)
    box("elbow_light", (0.202, -0.034, 0.0), (0.040, 0.006, 0.020),
        m["amber"], 0.001)
    export("upper_arm.glb")


def build_forearm():
    reset()
    m = common_materials("ro6_forearm")
    # Exact link bounds are [0, .18] x [-.05,.05] x [-.05,.05].
    cylinder("elbow_axis", (0.018, 0.0, 0.0), 0.050, 0.036, "X", m["alloy"], 16)
    box("beam", (0.090, 0.0, 0.0), (0.18, 0.064, 0.064),
        m["graphite"], 0.005)
    cylinder("wrist_cap", (0.165, 0.0, 0.0), 0.038, 0.018, "X", m["alloy"], 16)
    box("service_mark", (0.117, -0.031, 0.0), (0.055, 0.006, 0.016),
        m["mint"], 0.001)
    export("forearm.glb")


def build_tool():
    reset()
    m = common_materials("ro6_tool")
    # Exact tool envelope is [0, .12] x [-.0605,.0605] x [-.072,.072].
    # The broad wrist housing communicates the final joint frame while the
    # two fingers remain inside the same authored envelope.
    box("wrist", (0.0125, 0.0, 0.0), (0.025, 0.121, 0.144),
        m["alloy"], 0.006)
    box("palm", (0.0475, 0.0, 0.0), (0.070, 0.095, 0.090),
        m["graphite"], 0.006)
    box("finger_left", (0.095, 0.043, 0.0), (0.050, 0.035, 0.030),
        m["polymer"], 0.004)
    box("finger_right", (0.095, -0.043, 0.0), (0.050, 0.035, 0.030),
        m["polymer"], 0.004)
    box("grip_light", (0.098, 0.0, 0.046), (0.036, 0.008, 0.012),
        m["mint"], 0.001)
    export("tool.glb")


def main():
    build_turret()
    build_upper_arm()
    build_forearm()
    build_tool()
    reset()
    print("RO6_ARM_ASSETS_OK")


if __name__ == "__main__":
    main()
