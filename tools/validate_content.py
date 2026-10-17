#!/usr/bin/env python3
"""Validate Robot Simu data files and all references between them."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from jsonschema import Draft202012Validator

from validate_locales import validate_locales


class ContentError(RuntimeError):
    pass


@dataclass(frozen=True)
class Document:
    path: Path
    data: dict[str, Any]


def _read(path: Path) -> Document:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ContentError(f"{path}: cannot read JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ContentError(f"{path}: root must be an object")
    return Document(path, data)


def _pointer(parts: Iterable[Any]) -> str:
    escaped = [str(p).replace("~", "~0").replace("/", "~1") for p in parts]
    return "/" + "/".join(escaped) if escaped else "/"


def _schema(doc: Document, schema_path: Path) -> None:
    schema = _read(schema_path).data
    errors = sorted(
        Draft202012Validator(schema).iter_errors(doc.data), key=lambda e: list(e.path)
    )
    if errors:
        error = errors[0]
        raise ContentError(
            f"{doc.path}:{_pointer(error.absolute_path)}: {error.message}"
        )


def _finite_numbers(value: Any, path: Path, pointer: tuple[Any, ...] = ()) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ContentError(f"{path}:{_pointer(pointer)}: number must be finite")
    if isinstance(value, dict):
        for key, child in value.items():
            _finite_numbers(child, path, pointer + (key,))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _finite_numbers(child, path, pointer + (index,))


def _inside(point: list[float], size: list[float]) -> bool:
    return 0 <= point[0] < size[0] and 0 <= point[1] < size[1]


def _inside_box(point: list[float], box: dict[str, Any]) -> bool:
    return (
        box["min"][0] <= point[0] < box["max"][0]
        and box["min"][1] <= point[1] < box["max"][1]
    )


def _register(ids: dict[str, str], object_id: str, location: str) -> None:
    previous = ids.get(object_id)
    if previous is not None:
        raise ContentError(
            f"{location}: duplicate id '{object_id}', first declared at {previous}"
        )
    ids[object_id] = location


def _support_margin(contacts: list[tuple[float, float]]) -> float:
    points = sorted(set(contacts))
    if len(points) < 3:
        return -1.0

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-9:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-9:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        return -1.0
    margin = math.inf
    for index, start in enumerate(hull):
        end = hull[(index + 1) % len(hull)]
        edge_length = math.hypot(end[0] - start[0], end[1] - start[1])
        margin = min(margin, cross(start, end, (0.0, 0.0)) / max(1e-9, edge_length))
    return margin


def load_and_validate(root: Path) -> dict[str, Any]:
    root = root.resolve()
    validate_locales(root)
    schema_dir = root / "schema"
    campaign = _read(root / "campaign.json")
    chapters = _read(root / "chapters.json")
    levels = [_read(p) for p in sorted((root / "levels").glob("*.json"))]
    maps = [_read(p) for p in sorted((root / "maps").glob("*.json"))]
    surface_library = _read(root / "surfaces.json")
    scenes = [_read(p) for p in sorted((root / "scenes").glob("*.json"))]
    component_docs = [
        _read(p)
        for p in sorted((root / "components").glob("*.json"))
        if p.name != "policies.json"
    ]
    policies = _read(root / "components" / "policies.json")
    task_policies = _read(root / "task_policies.json")
    stories = [_read(p) for p in sorted((root / "story").glob("*.json"))]
    contracts = [_read(p) for p in sorted((root / "contracts").glob("*.json"))]
    presentations = [_read(p) for p in sorted((root / "presentation").glob("*.json"))]
    robots = [_read(p) for p in sorted((root / "robots").glob("*.json"))]
    abilities = _read(root / "abilities.json")
    strings = _read(root / "strings.json")
    audio_cues = _read(root / "audio" / "cues.json")

    if (
        not levels
        or not maps
        or not scenes
        or not component_docs
        or not stories
        or not presentations
        or not robots
    ):
        raise ContentError(
            f"{root}: levels, maps, scenes, components, robots, presentation and story must not be empty"
        )

    _schema(campaign, schema_dir / "campaign.schema.json")
    _schema(chapters, schema_dir / "chapters.schema.json")
    for doc in levels:
        _schema(doc, schema_dir / "level.schema.json")
    _schema(surface_library, schema_dir / "surface.schema.json")
    for doc in maps:
        _schema(doc, schema_dir / "map.schema.json")
    for doc in scenes:
        _schema(doc, schema_dir / "scene.schema.json")
    for doc in component_docs:
        _schema(doc, schema_dir / "components.schema.json")
    _schema(policies, schema_dir / "policies.schema.json")
    _schema(task_policies, schema_dir / "task-policies.schema.json")
    for doc in stories:
        _schema(doc, schema_dir / "story.schema.json")
    for doc in contracts:
        _schema(doc, schema_dir / "contract.schema.json")
    for doc in presentations:
        _schema(doc, schema_dir / "presentation.schema.json")
    _schema(abilities, schema_dir / "abilities.schema.json")
    _schema(strings, schema_dir / "strings.schema.json")
    _schema(audio_cues, schema_dir / "audio-cues.schema.json")
    # String coverage: every tr("key") literal in client AND game code must
    # resolve (typos fail here, not on the debrief screen). Sim event sentences
    # live in strings.json too (event.*): player-facing text is content,
    # never C++ literals. Keys are always full literals, never constructed.
    import re as _re
    from pathlib import Path as _Path

    _src_dir = _Path(__file__).resolve().parent.parent / "src"
    used_keys: set[str] = set()
    for source in sorted(_src_dir.glob("sim/game/client/*.cpp")) + sorted(
        _src_dir.glob("sim/game/mission/*.cpp")
 ) + sorted(
 _src_dir.glob("sim/game/scenario/*.cpp")
    ):
        text = source.read_text(encoding="utf-8")
        for match in _re.finditer(r'\btrf?\("([^"]+)"', text):
            used_keys.add(match.group(1))
        for match in _re.finditer(
            r'"((?:app|chapter|garage|prediction|running|debrief|world|editor|ui)\.[a-z_]+)"',
            text,
        ):
            used_keys.add(match.group(1))
    # Data-driven labels are references too. Keeping them in content removes UI
    # chassis-type branches, so the validator must account for them explicitly.
    for robot in robots:
        label_key = robot.data["drive"].get("label_key")
        if label_key:
            used_keys.add(label_key)
    string_keys = set(strings.data["strings"])
    orphan = sorted(used_keys - string_keys)
    if orphan:
        raise ContentError(f"strings.json: unknown UI keys {orphan}")
    unused = sorted(string_keys - used_keys)
    if unused:
        raise ContentError(f"strings.json: unused keys {unused}")
    for doc in robots:
        for key in (
            "max_wheel_speed_mps",
            "max_wheel_accel_mps2",
            "stall_force_N",
            "free_speed_mps",
            "pwm_limit",
            "integral_limit",
            "wheel_equiv_mass_kg",
 "controller_bandwidth_radps",
 "controller_integral_time_s",
 "controller_derivative_time_s",
 "coulomb_smoothing_speed_mps",
 "viscous_drag_N_per_mps",
        ):
            if key not in doc.data["drive"]["motor"]:
                raise ContentError(f"{doc.path}:/drive/motor: robot misses '{key}'")
        _schema(doc, schema_dir / "robot.schema.json")

    for doc in [
        campaign,
        *levels,
        *maps,
        *scenes,
        *component_docs,
        policies,
        task_policies,
        *stories,
        *presentations,
        *robots,
        audio_cues,
    ]:
        _finite_numbers(doc.data, doc.path)

    cue_ids: set[str] = set()
    for index, cue in enumerate(audio_cues.data["cues"]):
        if cue["id"] in cue_ids:
            raise ContentError(f"{audio_cues.path}:/cues/{index}/id: duplicate cue")
        cue_ids.add(cue["id"])
        if not (root / cue["file"]).is_file():
            raise ContentError(
                f"{audio_cues.path}:/cues/{index}/file: missing audio '{cue['file']}'"
            )
    event_codes: set[str] = set()
    for index, binding in enumerate(audio_cues.data["events"]):
        if binding["cue"] not in cue_ids:
            raise ContentError(
                f"{audio_cues.path}:/events/{index}/cue: unknown cue '{binding['cue']}'"
            )
        for code in binding["codes"]:
            if code in event_codes:
                raise ContentError(
                    f"{audio_cues.path}:/events/{index}/codes: duplicate event code '{code}'"
                )
            event_codes.add(code)
    transition = audio_cues.data["decision_transition"]
    for field in ("enter_cue", "exit_cue"):
        if transition[field] not in cue_ids:
            raise ContentError(
                f"{audio_cues.path}:/decision_transition/{field}: unknown cue '{transition[field]}'"
            )

    global_ids: dict[str, str] = {}
    _register(global_ids, campaign.data["id"], f"{campaign.path}:/id")
    _register(
        global_ids, campaign.data["companion"]["id"], f"{campaign.path}:/companion/id"
    )

    level_by_id: dict[str, Document] = {}
    order_to_id: dict[int, str] = {}
    for doc in levels:
        level_id = doc.data["id"]
        _register(global_ids, level_id, f"{doc.path}:/id")
        if doc.path.stem != level_id:
            raise ContentError(f"{doc.path}:/id: id must match filename")
        if doc.data["order"] in order_to_id:
            raise ContentError(
                f"{doc.path}:/order: duplicate order {doc.data['order']}"
            )
        order_to_id[doc.data["order"]] = level_id
        level_by_id[level_id] = doc

    map_by_id: dict[str, Document] = {}
    for doc in maps:
        map_id = doc.data["id"]
        _register(global_ids, map_id, f"{doc.path}:/id")
        if doc.path.stem != map_id:
            raise ContentError(f"{doc.path}:/id: id must match filename")
        map_by_id[map_id] = doc

        materials = {m["id"] for m in surface_library.data["materials"]}
        for index, rect in enumerate(doc.data["surfaces"]):
            if rect["material"] not in materials:
                raise ContentError(
                    f"{doc.path}:/surfaces/{index}: unknown material '{rect['material']}'"
                )
        size = doc.data["size_m"]
        for collection in ("obstacles", "surfaces"):
            for index, box in enumerate(doc.data[collection]):
                _register(global_ids, box["id"], f"{doc.path}:/{collection}/{index}/id")
                mn, mx = box["min"], box["max"]
                if (
                    not _inside(mn, size)
                    or mx[0] > size[0]
                    or mx[1] > size[1]
                    or mn[0] >= mx[0]
                    or mn[1] >= mx[1]
                ):
                    raise ContentError(
                        f"{doc.path}:/{collection}/{index}: box must have positive area inside map bounds"
                    )
        for name, pose in doc.data["markers"].items():
            if not _inside(pose, size):
                raise ContentError(
                    f"{doc.path}:/markers/{name}: marker is outside map bounds"
                )
            permanent = [box for box in doc.data["obstacles"] if "active_in" not in box]
            if any(_inside_box(pose, box) for box in permanent):
                raise ContentError(
                    f"{doc.path}:/markers/{name}: marker is inside a permanent obstacle"
                )
        for index, actor in enumerate(doc.data["actors"]):
            _register(global_ids, actor["id"], f"{doc.path}:/actors/{index}/id")
            for path_index, point in enumerate(actor["path"]):
                if not _inside(point, size):
                    raise ContentError(
                        f"{doc.path}:/actors/{index}/path/{path_index}: point is outside map bounds"
                    )
                if any(_inside_box(point, box) for box in permanent):
                    raise ContentError(
                        f"{doc.path}:/actors/{index}/path/{path_index}: point is inside a permanent obstacle"
                    )
            for active_id in actor["active_in"]:
                if active_id not in level_by_id:
                    # Level collection is complete at this point; map validation follows level registration.
                    raise ContentError(
                        f"{doc.path}:/actors/{index}/active_in: unknown level '{active_id}'"
                    )
        for index, obstacle in enumerate(doc.data["obstacles"]):
            for active_id in obstacle.get("active_in", []):
                if active_id not in level_by_id:
                    raise ContentError(
                        f"{doc.path}:/obstacles/{index}/active_in: unknown level '{active_id}'"
                    )

    scene_by_id: dict[str, Document] = {}
    for doc in scenes:
        scene = doc.data
        scene_id = scene["id"]
        _register(global_ids, scene_id, f"{doc.path}:/id")
        scene_by_id[scene_id] = doc
        prefab_ids: set[str] = set()
        for prefab_index, prefab in enumerate(scene["prefabs"]):
            if prefab["id"] in prefab_ids:
                raise ContentError(
                    f"{doc.path}:/prefabs/{prefab_index}/id: duplicate prefab"
                )
            prefab_ids.add(prefab["id"])
            for part_index, part in enumerate(prefab["parts"]):
                model = part.get("model_3d")
                if model and not (root / model).is_file():
                    raise ContentError(
                        f"{doc.path}:/prefabs/{prefab_index}/parts/{part_index}/model_3d: missing model '{model}'"
                    )
        goal_visual_ids: set[str] = set()
        for visual_index, visual in enumerate(scene.get("goal_visuals", [])):
            if visual["id"] in goal_visual_ids:
                raise ContentError(
                    f"{doc.path}:/goal_visuals/{visual_index}/id: duplicate goal visual"
                )
            goal_visual_ids.add(visual["id"])
            for state, prefab_id in visual["states"].items():
                if prefab_id not in prefab_ids:
                    raise ContentError(
                        f"{doc.path}:/goal_visuals/{visual_index}/states/{state}: "
                        f"unknown prefab '{prefab_id}'"
                    )
        payload_visual = scene.get("payload_visual")
        if payload_visual:
            for field in ("container_prefab", "item_prefab", "damaged_item_prefab"):
                if payload_visual[field] not in prefab_ids:
                    raise ContentError(
                        f"{doc.path}:/payload_visual/{field}: unknown prefab "
                        f"'{payload_visual[field]}'"
                    )
        instance_ids: set[str] = set()
        for instance_index, instance in enumerate(scene["instances"]):
            if instance["id"] in instance_ids:
                raise ContentError(
                    f"{doc.path}:/instances/{instance_index}/id: duplicate instance"
                )
            instance_ids.add(instance["id"])
            if instance["prefab"] not in prefab_ids:
                raise ContentError(
                    f"{doc.path}:/instances/{instance_index}/prefab: unknown prefab '{instance['prefab']}'"
                )
            for active_id in instance.get("active_in", []):
                if active_id not in level_by_id:
                    raise ContentError(
                        f"{doc.path}:/instances/{instance_index}/active_in: unknown level '{active_id}'"
                    )
        shots = {
            "focus_weights": scene["camera"]["focus_weights"],
            "event_shot/focus_weights": scene["camera"]["event_shot"]["focus_weights"],
            "success_shot/focus_weights": scene["camera"]["success_shot"][
                "focus_weights"
            ],
        }
        for shot_path, weights in shots.items():
            if not math.isclose(sum(weights.values()), 1.0, abs_tol=1e-6):
                raise ContentError(
                    f"{doc.path}:/camera/{shot_path}: weights must sum to 1"
                )
        if scene["lighting"]["fog_end_m"] <= scene["lighting"]["fog_start_m"]:
            raise ContentError(
                f"{doc.path}:/lighting: fog_end_m must exceed fog_start_m"
            )
        if scene["floor"]["seam_m"] >= scene["floor"]["tile_size_m"]:
            raise ContentError(
                f"{doc.path}:/floor/seam_m: seam must be smaller than a tile"
            )
        if not math.isclose(scene["effects"]["robot_scale"], 1.0, abs_tol=1e-9):
            raise ContentError(
                f"{doc.path}:/effects/robot_scale: official scenes must preserve physical scale 1.0"
            )
        floor_texture = root / scene["floor"]["albedo_texture"]
        if not floor_texture.is_file():
            raise ContentError(
                f"{doc.path}:/floor/albedo_texture: missing texture '{floor_texture}'"
            )

    for doc in maps:
        if doc.data["scene_3d"] not in scene_by_id:
            raise ContentError(
                f"{doc.path}:/scene_3d: unknown scene '{doc.data['scene_3d']}'"
            )

    components: dict[str, tuple[str, dict[str, Any]]] = {}
    components_by_slot: dict[str, list[dict[str, Any]]] = {}
    ability_ids: set[str] = set()
    for index, ability in enumerate(abilities.data["abilities"]):
        _register(global_ids, ability["id"], f"abilities.json:/abilities/{index}/id")
        ability_ids.add(ability["id"])
    for doc in component_docs:
        slot = doc.data["slot"]
        components_by_slot.setdefault(slot, [])
        for index, component in enumerate(doc.data["components"]):
            component_id = component["id"]
            _register(global_ids, component_id, f"{doc.path}:/components/{index}/id")
            components[component_id] = (slot, component)
            components_by_slot[slot].append(component)
            model = root / component["visual"]["model_3d"]
            if not model.is_file():
                raise ContentError(
                    f"{doc.path}:/components/{index}/visual/model_3d: missing model '{model}'"
                )

    robot_by_id: dict[str, dict[str, Any]] = {}
    for doc in robots:
        robot = doc.data
        robot_id = robot["id"]
        _register(global_ids, robot_id, f"{doc.path}:/id")
        robot_by_id[robot_id] = robot
        if "unlock_ability" in robot and robot["unlock_ability"] not in ability_ids:
            raise ContentError(
                f"{doc.path}:/unlock_ability: unknown ability '{robot['unlock_ability']}'"
            )
        mount_ids: set[str] = set()
        mounts_by_id: dict[str, dict[str, Any]] = {}
        for index, mount in enumerate(robot["mounts"]):
            if mount["id"] in mount_ids:
                raise ContentError(
                    f"{doc.path}:/mounts/{index}/id: duplicate mount '{mount['id']}'"
                )
            mount_ids.add(mount["id"])
            mounts_by_id[mount["id"]] = mount
        slots: set[str] = set()
        for index, slot in enumerate(robot["slots"]):
            if slot["slot"] in slots:
                raise ContentError(
                    f"{doc.path}:/slots/{index}/slot: duplicate slot '{slot['slot']}'"
                )
            slots.add(slot["slot"])
            for mount in slot["mounts"]:
                if mount not in mount_ids:
                    raise ContentError(
                        f"{doc.path}:/slots/{index}/mounts: unknown mount '{mount}'"
                    )
        for index, part in enumerate(robot["base_parts"]):
            if part["mount"] not in mount_ids:
                raise ContentError(
                    f"{doc.path}:/base_parts/{index}/mount: unknown mount '{part['mount']}'"
                )
            model = root / part["visual"]["model_3d"]
            if not model.is_file():
                raise ContentError(
                    f"{doc.path}:/base_parts/{index}/visual/model_3d: missing model '{model}'"
                )
        if slots != {"tires", "sensor", "computer", "motor", "imu"}:
            raise ContentError(
                f"{doc.path}:/slots: robot must define tires, sensor, computer, motor and imu slots"
            )
        for index, support in enumerate(robot.get("passive_supports", [])):
            steer_mount = support["steer_mount"]
            roll_mount = support["roll_mount"]
            if steer_mount not in mount_ids or roll_mount not in mount_ids:
                raise ContentError(
                    f"{doc.path}:/passive_supports/{index}: unknown mount"
                )
            if (
                steer_mount == roll_mount
                or mounts_by_id[steer_mount]["motion"] != "caster"
                or mounts_by_id[roll_mount]["motion"] != "wheel"
            ):
                raise ContentError(
                    f"{doc.path}:/passive_supports/{index}: expected distinct caster "
                    "steer and rolling wheel mounts"
                )
        tire_slot = next(slot for slot in robot["slots"] if slot["slot"] == "tires")
        for mount in tire_slot["mounts"]:
            if mounts_by_id[mount]["motion"] not in {"wheel", "track"}:
                raise ContentError(
                    f"{doc.path}:/slots: tire mount '{mount}' must declare wheel or track motion"
                )
        if robot["drive"]["type"] == "differential":
            contacts = [
                tuple(mounts_by_id[mount]["position_m"][:2])
                for mount in tire_slot["mounts"]
            ]
            contacts.extend(
                tuple(mounts_by_id[support["steer_mount"]]["position_m"][:2])
                for support in robot.get("passive_supports", [])
            )
            if _support_margin(contacts) < 0.02:
                raise ContentError(
                    f"{doc.path}:/mounts: differential support polygon must contain "
                    "base_link with at least 0.02 m margin"
                )

    policy_by_id: dict[str, dict[str, Any]] = {}
    policy_by_id: dict[str, dict[str, Any]] = {}
    anchors: list[float] = []
    for index, policy in enumerate(policies.data["policies"]):
        policy_id = policy["id"]
        _register(global_ids, policy_id, f"{policies.path}:/policies/{index}/id")
        policy_by_id[policy_id] = policy
        anchor = policy.get("style")
        if not isinstance(anchor, (int, float)) or not 0.0 <= anchor <= 1.0:
            raise ContentError(
                f"{policies.path}:/policies/{index}/style: style anchor must be within [0, 1]"
            )
        anchors.append(float(anchor))
    if sorted(anchors) != anchors or len(set(anchors)) != len(anchors):
        raise ContentError(
            f"{policies.path}:/policies: style anchors must be strictly increasing"
        )

    task_policy_by_id: dict[str, dict[str, Any]] = {}
    for index, policy in enumerate(task_policies.data["task_policies"]):
        policy_id = policy["id"]
        _register(
            global_ids, policy_id, f"{task_policies.path}:/task_policies/{index}/id"
        )
        task_policy_by_id[policy_id] = policy
    for level_id, level_doc in level_by_id.items():
        policy_id = level_doc.data["objective"].get("task_policy", "task.balanced")
        if policy_id not in task_policy_by_id:
            raise ContentError(
                f"{level_doc.path}:/objective/task_policy: unknown task policy '{policy_id}'"
            )

    if sum(1 for doc in presentations if doc.data["is_default"]) != 1:
        raise ContentError(
            f"{root / 'presentation'}: exactly one default presentation is required"
        )
    presentation_by_id: dict[str, dict[str, Any]] = {}
    presentation_levels: set[str] = set()
    for doc in presentations:
        presentation = doc.data
        presentation_id = presentation["id"]
        _register(global_ids, presentation_id, f"{doc.path}:/id")
        presentation_by_id[presentation_id] = presentation
        level = level_by_id.get(presentation["level"])
        if level is None:
            raise ContentError(
                f"{doc.path}:/level: unknown level '{presentation['level']}'"
            )
        if presentation["level"] in presentation_levels:
            raise ContentError(
                f"{doc.path}:/level: duplicate presentation for level '{presentation['level']}'"
            )
        presentation_levels.add(presentation["level"])
        offered_task_policies = presentation.get("task_policies", [])
        if (
            level.data["objective"].get("goal_order", "authored") == "autonomous"
            and not offered_task_policies
        ):
            raise ContentError(
                f"{doc.path}:/task_policies: autonomous goal sets must offer a task policy"
            )
        for policy_index, policy_id in enumerate(offered_task_policies):
            if policy_id not in task_policy_by_id:
                raise ContentError(
                    f"{doc.path}:/task_policies/{policy_index}: unknown task policy '{policy_id}'"
                )
        columns = {
            column["slot"]: column
            for column in presentation["garage"]["component_columns"]
        }
        if len(columns) != len(presentation["garage"]["component_columns"]):
            raise ContentError(f"{doc.path}:/garage/component_columns: duplicate slot")
        required_slots = set(level.data["loadout_rules"]["required_slots"])
        if set(columns) != required_slots:
            raise ContentError(
                f"{doc.path}:/garage/component_columns: slots must match level requirements"
            )
        for column_index, column in enumerate(
            presentation["garage"]["component_columns"]
        ):
            if not column.get("player_choice", True) and len(column["options"]) != 1:
                raise ContentError(
                    f"{doc.path}:/garage/component_columns/{column_index}: "
                    "a fixed column must offer exactly one component"
                )
            for option_index, component_id in enumerate(column["options"]):
                component = components.get(component_id)
                if component is None:
                    raise ContentError(
                        f"{doc.path}:/garage/component_columns/{column_index}/options/"
                        f"{option_index}: unknown component '{component_id}'"
                    )
                if component[0] != column["slot"]:
                    raise ContentError(
                        f"{doc.path}:/garage/component_columns/{column_index}/options/"
                        f"{option_index}: component belongs to '{component[0]}'"
                    )
        for policy_index, policy_id in enumerate(presentation["garage"]["policies"]):
            if policy_id not in policy_by_id:
                raise ContentError(
                    f"{doc.path}:/garage/policies/{policy_index}: "
                    f"unknown policy '{policy_id}'"
                )
        choice_ids: set[str] = set()
        for choice_index, choice in enumerate(presentation["prediction"]["choices"]):
            if choice["id"] in choice_ids:
                raise ContentError(
                    f"{doc.path}:/prediction/choices/{choice_index}/id: "
                    f"duplicate choice id '{choice['id']}'"
                )
            choice_ids.add(choice["id"])
            _register(
                global_ids,
                choice["id"],
                f"{doc.path}:/prediction/choices/{choice_index}/id",
            )
        goal_lines = presentation["running"].get("goal_lines", [])
        if goal_lines and len(goal_lines) != len(level.data["objective"]["goals"]) + 1:
            raise ContentError(
                f"{doc.path}:/running/goal_lines: expected one line for every "
                "goal index plus the completed state"
            )
        for field in ("default_build", "recommended_build"):
            build = presentation["garage"][field]
            cost = 0
            selected_slots = set(build) - {"style", "task_policy"}
            if selected_slots != required_slots:
                raise ContentError(
                    f"{doc.path}:/garage/{field}: component slots must match level requirements"
                )
            for slot in required_slots:
                component_id = build[slot]
                component = components.get(component_id)
                if component is None or component[0] != slot:
                    raise ContentError(
                        f"{doc.path}:/garage/{field}/{slot}: invalid component '{component_id}'"
                    )
                if component_id not in columns[slot]["options"]:
                    raise ContentError(
                        f"{doc.path}:/garage/{field}/{slot}: component is not offered by its column"
                    )
                cost += component[1]["cost"]
            style = build.get("style")
            if not isinstance(style, (int, float)) or not 0.0 <= style <= 1.0:
                raise ContentError(
                    f"{doc.path}:/garage/{field}/style: style must be within [0, 1]"
                )
            task_policy = build.get("task_policy")
            if task_policy is not None:
                if (
                    level.data["objective"].get("goal_order", "authored")
                    != "autonomous"
                ):
                    raise ContentError(
                        f"{doc.path}:/garage/{field}/task_policy: requires autonomous goal order"
                    )
                if task_policy not in task_policy_by_id:
                    raise ContentError(
                        f"{doc.path}:/garage/{field}/task_policy: unknown task policy"
                    )
                if task_policy not in offered_task_policies:
                    raise ContentError(
                        f"{doc.path}:/garage/{field}/task_policy: policy is not offered by this level"
                    )
            if cost > level.data["loadout_rules"]["budget"]:
                raise ContentError(
                    f"{doc.path}:/garage/{field}: build cost exceeds level budget"
                )

    # DIY unlock chain: official builds may only use parts the player can own
    # by then (starting inventory plus component rewards of earlier levels).
    for name, component in components.items():
        if component[0] == "sensor":
            behavior = component[1]["behavior"]
            for key in (
                "min_range_m",
                "range_m",
                "rays",
                "scan_hz",
                "noise_sigma_m",
                "latency_s",
                "terrain_reflectivity",
                "obstacle_reflectivity",
                "actor_reflectivity",
                "incidence_floor",
                "distance_attenuation_per_m2",
                "power_w",
            ):
                if key not in behavior:
                    raise ContentError(
                        f"components: sensor '{name}' misses behavior '{key}'"
                    )
            if behavior["min_range_m"] >= behavior["range_m"]:
                raise ContentError(
                    f"components: sensor '{name}' minimum range must be below maximum range"
                )
            channels = behavior.get("vertical_channels", 1)
            if not isinstance(channels, (int, float)) or int(channels) != channels or not 1 <= channels <= 128:
                raise ContentError(
                    f"components: sensor '{name}' vertical_channels must be an integer within [1, 128]"
                )
            if channels > 1:
                angles = []
                for ring in range(int(channels)):
                    key = f"vertical_angle_{ring}_deg"
                    if key not in behavior or not -90.0 <= behavior[key] <= 90.0:
                        raise ContentError(
                            f"components: sensor '{name}' requires {key} within [-90, 90]"
                        )
                    angles.append(behavior[key])
                if len(set(angles)) != len(angles):
                    raise ContentError(
                        f"components: sensor '{name}' vertical beam angles must be unique"
                    )
                for key in (
                    "projection_min_height_m",
                    "projection_max_height_m",
                    "projection_ground_seed_max_height_m",
                    "projection_ground_max_slope_deg",
                ):
                    if key not in behavior:
                        raise ContentError(
                            f"components: sensor '{name}' misses multi-line behavior '{key}'"
                        )
                if behavior["projection_min_height_m"] >= behavior["projection_max_height_m"]:
                    raise ContentError(
                        f"components: sensor '{name}' projection height interval is empty"
                    )
                if not 0.0 <= behavior["projection_ground_max_slope_deg"] < 90.0:
                    raise ContentError(
                        f"components: sensor '{name}' ground slope limit must be within [0, 90)"
                    )
            for key in (
                "terrain_reflectivity",
                "obstacle_reflectivity",
                "actor_reflectivity",
                "incidence_floor",
            ):
                if not 0.0 <= behavior[key] <= 1.0:
                    raise ContentError(
                        f"components: sensor '{name}' behavior '{key}' must be within [0, 1]"
                    )
    for name, component in components.items():
        if component[0] == "motor":
            for key in (
                "max_wheel_speed_mps",
                "stall_force_N",
                "free_speed_mps",
                "power_scale",
 "controller_bandwidth_radps",
 "controller_integral_time_s",
 "controller_derivative_time_s",
 "coulomb_smoothing_speed_mps",
 "viscous_drag_N_per_mps",
            ):
                if key not in component[1]["behavior"]:
                    raise ContentError(
                        f"components: motor '{name}' misses behavior '{key}'"
                    )
    for name, component in components.items():
        if component[0] == "imu":
            for key in (
                "gyro_noise_density_radps_sqrt_hz",
                "accel_noise_density_mps2_sqrt_hz",
                "noise_bandwidth_hz",
                "gyro_bias_stddev_radps",
                "gyro_bias_random_walk_radps_sqrt_s",
                "power_w",
            ):
                if key not in component[1]["behavior"]:
                    raise ContentError(
                        f"components: imu '{name}' misses behavior '{key}'"
                    )
    presentation_by_level = {doc.data["level"]: doc.data for doc in presentations}
    owned: set[str] = set(campaign.data["starting_inventory"])
    for level_id in campaign.data["levels"] + campaign.data["playgrounds"]:
        presentation = presentation_by_level.get(level_id)
        if presentation is None:
            raise ContentError(
                f"{root / 'presentation'}: missing presentation for '{level_id}'"
            )
        if presentation.get("robot") and presentation["robot"] not in robot_by_id:
            raise ContentError(
                f"{level_id}:/robot: unknown robot '{presentation['robot']}'"
            )
        for chassis in presentation.get("chassis_options", []):
            if chassis not in robot_by_id:
                raise ContentError(
                    f"{level_id}:/chassis_options: unknown robot '{chassis}'"
                )
        loadout_access = level_by_id[level_id].data["loadout_rules"].get(
            "access",
            "open" if level_by_id[level_id].data.get("mode") == "playground" else "campaign",
        )
        for field in ("default_build", "recommended_build"):
            build = presentation["garage"][field]
            for slot in ("tires", "sensor", "computer", "motor", "imu"):
                component_id = build.get(slot)
                if component_id and loadout_access != "open" and component_id not in owned:
                    raise ContentError(
                        f"{level_id}:/garage/{field}/{slot}: '{component_id}' "
                        f"is not owned yet (no earlier reward grants it)"
                    )
        for reward in level_by_id[level_id].data.get("rewards", []):
            if reward in components:
                owned.add(reward)
            elif reward.startswith("ability."):
                if reward not in ability_ids:
                    raise ContentError(
                        f"{level_id}:/rewards: unknown ability '{reward}'"
                    )
            elif not reward.startswith("story."):
                raise ContentError(f"{level_id}:/rewards: unknown reward '{reward}'")

    if presentation_levels != set(level_by_id):
        missing = sorted(set(level_by_id) - presentation_levels)
        raise ContentError(
            f"{root / 'presentation'}: missing presentation for levels {missing}"
        )

    for doc in stories:
        _register(global_ids, doc.data["id"], f"{doc.path}:/id")
        for index, beat in enumerate(doc.data["beats"]):
            _register(global_ids, beat["id"], f"{doc.path}:/beats/{index}/id")
            if beat["level"] not in level_by_id:
                raise ContentError(
                    f"{doc.path}:/beats/{index}/level: unknown level '{beat['level']}'"
                )
    for doc in contracts:
        for index, job in enumerate(doc.data["jobs"]):
            _register(global_ids, job["id"], f"{doc.path}:/jobs/{index}/id")
            if job["level"] not in level_by_id:
                raise ContentError(
                    f"{doc.path}:/jobs/{index}/level: unknown level '{job['level']}'"
                )

    campaign_levels = campaign.data["levels"]
    playgrounds = campaign.data["playgrounds"]
    if set(campaign_levels) & set(playgrounds):
        raise ContentError(
            f"{campaign.path}: guided levels and playgrounds must be disjoint"
        )
    if set(campaign_levels) | set(playgrounds) != set(level_by_id):
        raise ContentError(
            f"{campaign.path}: guided levels and playgrounds must list every level exactly once"
        )
    sorted_ids = [level_id for _, level_id in sorted(order_to_id.items())]
    listed_ids = campaign_levels + playgrounds
    if listed_ids != sorted_ids:
        raise ContentError(
            f"{campaign.path}: guided levels followed by playgrounds must match level order fields"
        )
    if campaign.data["starting_level"] not in campaign_levels:
        raise ContentError(f"{campaign.path}:/starting_level: must be a guided level")
    if campaign.data["companion"]["robot"] not in robot_by_id:
        raise ContentError(f"{campaign.path}:/companion/robot: unknown robot")
    for index, component_id in enumerate(campaign.data["starting_inventory"]):
        if component_id not in components:
            raise ContentError(
                f"{campaign.path}:/starting_inventory/{index}: unknown component '{component_id}'"
            )
    for index, policy_id in enumerate(campaign.data["starting_policies"]):
        if policy_id not in policy_by_id:
            raise ContentError(
                f"{campaign.path}:/starting_policies/{index}: unknown policy '{policy_id}'"
            )

    for doc in levels:
        level = doc.data
        map_doc = map_by_id.get(level["map"])
        if map_doc is None:
            raise ContentError(f"{doc.path}:/map: unknown map '{level['map']}'")
        environment = level.get("environment")
        if environment:
            times = [keyframe["time_s"] for keyframe in environment["keyframes"]]
            if not math.isclose(times[0], 0.0, abs_tol=1e-9):
                raise ContentError(
                    f"{doc.path}:/environment/keyframes: timeline must start at 0 s"
                )
            if any(current <= previous for previous, current in zip(times, times[1:])):
                raise ContentError(
                    f"{doc.path}:/environment/keyframes: times must be strictly increasing"
                )
            for keyframe_index, keyframe in enumerate(environment["keyframes"]):
                for material in keyframe["material_grip"]:
                    if material not in materials:
                        raise ContentError(
                            f"{doc.path}:/environment/keyframes/{keyframe_index}/material_grip: "
                            f"unknown material '{material}'"
                        )
        scene_doc = scene_by_id[map_doc.data["scene_3d"]]
        scene_goal_visuals = {
            visual["id"] for visual in scene_doc.data.get("goal_visuals", [])
        }
        for goal_index, goal in enumerate(level["objective"].get("goals", [])):
            visual_id = goal.get("visual")
            if goal["action"] == "collect" and not visual_id:
                raise ContentError(
                    f"{doc.path}:/objective/goals/{goal_index}/visual: collect goals require a scene visual"
                )
            if visual_id and visual_id not in scene_goal_visuals:
                raise ContentError(
                    f"{doc.path}:/objective/goals/{goal_index}/visual: unknown scene goal visual "
                    f"'{visual_id}'"
                )
        payload_visual = scene_doc.data.get("payload_visual")
        if payload_visual:
            presentation = presentation_by_level[level["id"]]
            applicable_robots = presentation.get("chassis_options") or [
                presentation.get("robot", campaign.data["companion"]["robot"])
            ]
            for robot_id in applicable_robots:
                robot_mounts = {
                    mount["id"] for mount in robot_by_id[robot_id]["mounts"]
                }
                if payload_visual["mount"] not in robot_mounts:
                    raise ContentError(
                        f"{scene_doc.path}:/payload_visual/mount: robot '{robot_id}' has no "
                        f"mount '{payload_visual['mount']}'"
                    )
        for index, prerequisite in enumerate(level["prerequisites"]):
            if prerequisite not in level_by_id:
                raise ContentError(
                    f"{doc.path}:/prerequisites/{index}: unknown level '{prerequisite}'"
                )
            if level_by_id[prerequisite].data["order"] >= level["order"]:
                raise ContentError(
                    f"{doc.path}:/prerequisites/{index}: prerequisite must have a lower order"
                )
        markers = map_doc.data["markers"]
        for field in ("spawn", "via", "goal"):
            marker = level["objective"].get(field)
            if marker is not None and marker not in markers:
                raise ContentError(
                    f"{doc.path}:/objective/{field}: unknown marker '{marker}'"
                )
        minimum_cost = 0
        for index, slot in enumerate(level["loadout_rules"]["required_slots"]):
            choices = components_by_slot.get(slot, [])
            if not choices:
                raise ContentError(
                    f"{doc.path}:/loadout_rules/required_slots/{index}: no components for slot '{slot}'"
                )
            minimum_cost += min(item["cost"] for item in choices)
        if minimum_cost > level["loadout_rules"]["budget"]:
            raise ContentError(
                f"{doc.path}:/loadout_rules/budget: no legal build fits the budget"
            )

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(level_id: str) -> None:
        if level_id in visiting:
            raise ContentError(
                f"{level_by_id[level_id].path}:/prerequisites: unlock graph contains a cycle"
            )
        if level_id in visited:
            return
        visiting.add(level_id)
        for predecessor in level_by_id[level_id].data["prerequisites"]:
            visit(predecessor)
        visiting.remove(level_id)
        visited.add(level_id)

    chapter_ids = {chapter["id"] for chapter in chapters.data["chapters"]}
    for level_id, level in level_by_id.items():
        chapter = level.data.get("chapter", 1)
        if chapter not in chapter_ids:
            raise ContentError(f"{level.path}:/chapter: unknown chapter {chapter}")
        visit(level_id)

    return {
        "campaign": campaign.data,
        "levels": {key: doc.data for key, doc in level_by_id.items()},
        "maps": {key: doc.data for key, doc in map_by_id.items()},
        "scenes": {key: doc.data for key, doc in scene_by_id.items()},
        "components": {key: value[1] for key, value in components.items()},
        "policies": policy_by_id,
        "presentations": presentation_by_id,
        "robots": robot_by_id,
        "stories": [doc.data for doc in stories],
        "contracts": [doc.data for doc in contracts],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "assets",
    )
    args = parser.parse_args()
    try:
        catalog = load_and_validate(args.root)
    except ContentError as exc:
        print(f"CONTENT INVALID: {exc}")
        return 1
    print(
        "CONTENT VALID: "
        f"{len(catalog['levels'])} levels, {len(catalog['maps'])} maps, "
        f"{len(catalog['scenes'])} scenes, "
        f"{len(catalog['components'])} components, {len(catalog['policies'])} policies, "
        f"{len(catalog['robots'])} robots, {len(catalog['presentations'])} presentations"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
