#!/usr/bin/env python3

from __future__ import annotations

import json
import shutil
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from validate_content import ContentError, load_and_validate  # noqa: E402


class ContentValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.assets = Path(self.temp.name) / "assets"
        shutil.copytree(ROOT / "assets", self.assets)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def edit(self, relative: str, mutate) -> None:
        path = self.assets / relative
        data = json.loads(path.read_text(encoding="utf-8"))
        mutate(data)
        path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def test_repository_content_is_valid(self) -> None:
        catalog = load_and_validate(self.assets)
        levels = set(catalog["levels"].keys())
        guided = set(catalog["campaign"]["levels"])
        playgrounds = set(catalog["campaign"]["playgrounds"])
        self.assertFalse(guided & playgrounds)
        self.assertEqual(levels, guided | playgrounds)
        self.assertIn("vehicle_proving_ground", playgrounds)
        served = {p["level"] for p in catalog["presentations"].values()}
        self.assertEqual(levels, served)
        scenes = set(catalog["scenes"].keys())
        for level in catalog["levels"].values():
            game_map = catalog["maps"][level["map"]]
            self.assertIn(game_map["scene_3d"], scenes)
        robots = set(catalog["robots"].keys())
        self.assertIn(catalog["campaign"]["companion"]["robot"], robots)
        self.assertEqual("boot", catalog["presentations"]["presentation.boot"]["level"])
        proving_ground = catalog["presentations"]["presentation.vehicle_proving_ground"]
        self.assertEqual("robot.ro2", proving_ground["robot"])
        self.assertEqual(
            "operator_drive",
            catalog["levels"]["vehicle_proving_ground"]["control_mode"],
        )
        proving_map = catalog["maps"]["proving_ground_map"]
        self.assertEqual([48.0, 32.0], proving_map["size_m"])
        self.assertEqual(
            {"standing_water", "sand", "gravel", "steel_deck", "mud"},
            {surface["material"] for surface in proving_map["surfaces"]},
        )
        self.assertEqual(
            {"longitudinal_ramp", "ramp_highland", "cross_slope_lane", "rough_lane"},
            {terrain["id"] for terrain in proving_map["terrain"]},
        )
        self.assertEqual(
            "\u6d1b\u4e00", catalog["campaign"]["companion"]["display_name"]
        )

    def test_v1_content_without_effect_extensions_remains_valid(self) -> None:
        def remove_surface_effects(doc) -> None:
            for material in doc["materials"]:
                material.pop("drive_effect", None)

        def remove_scene_effects(doc) -> None:
            for key in (
                "trail_duration_s",
                "surface_particle_count",
                "surface_particle_scale",
                "risk_indicator_radius_m",
            ):
                doc["effects"].pop(key, None)

        self.edit("surfaces.json", remove_surface_effects)
        for scene_path in self.assets.glob("scenes/*.json"):
            relative = scene_path.relative_to(self.assets).as_posix()
            self.edit(relative, remove_scene_effects)
        load_and_validate(self.assets)

    def test_glb_meshes_have_vertex_normals(self) -> None:
        model_count = 0
        for path in self.assets.glob("models/**/*.glb"):
            payload = path.read_bytes()
            self.assertEqual(0x46546C67, struct.unpack_from("<I", payload)[0], path)
            json_length, json_type = struct.unpack_from("<II", payload, 12)
            self.assertEqual(0x4E4F534A, json_type, path)
            document = json.loads(payload[20 : 20 + json_length])
            for mesh in document["meshes"]:
                for primitive in mesh["primitives"]:
                    self.assertIn("POSITION", primitive["attributes"], path)
                    self.assertIn("NORMAL", primitive["attributes"], path)
                    position = document["accessors"][
                        primitive["attributes"]["POSITION"]
                    ]
                    normal = document["accessors"][primitive["attributes"]["NORMAL"]]
                    self.assertEqual(position["count"], normal["count"], path)
            model_count += 1
        # No absolute count: the kit grows with content. The pin is coverage:
        # every model referenced by robots/scenes/components resolves, and
        # every shipped glb above passed POSITION/NORMAL validation.
        referenced: set[str] = set()
        for robot_path in self.assets.glob("robots/*.json"):
            robot = json.loads(robot_path.read_text(encoding="utf-8"))
            for part in robot.get("base_parts", []):
                model = (part.get("visual") or {}).get("model_3d")
                if model:
                    referenced.add(model)
        for scene_path in self.assets.glob("scenes/*.json"):
            scene = json.loads(scene_path.read_text(encoding="utf-8"))
            for prefab in scene.get("prefabs", []):
                for part in prefab.get("parts", []):
                    if part.get("model_3d"):
                        referenced.add(part["model_3d"])
        for map_path in self.assets.glob("maps/*.json"):
            game_map = json.loads(map_path.read_text(encoding="utf-8"))
            for actor in game_map.get("actors", []):
                # Actor self-description: visual model must resolve (the
                # framework poses it; nothing is hardcoded in C++).
                referenced.add(actor["visual"]["model"])
        for component_path in self.assets.glob("components/*.json"):
            try:
                doc = json.loads(component_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                continue
            items = doc if isinstance(doc, list) else doc.get("components", [])
            for item in items if isinstance(items, list) else []:
                model = (
                    (item.get("visual") or {}).get("model_3d")
                    if isinstance(item, dict)
                    else None
                )
                if model:
                    referenced.add(model)
        # Actor kit is code-loaded, not JSON-referenced: pin its contract.
        referenced.update(["models/actors/forklift.glb", "models/actors/worker.glb"])
        self.assertTrue(referenced)
        for model in sorted(referenced):
            self.assertTrue((self.assets / model).exists(), model)

    def test_floor_texture_is_packaged(self) -> None:
        texture = self.assets / "textures/environment/concrete_floor.png"
        payload = texture.read_bytes()
        self.assertEqual(b"\x89PNG\r\n\x1a\n", payload[:8])
        width, height = struct.unpack_from(">II", payload, 16)
        self.assertEqual((512, 512), (width, height))

    def test_rejects_bad_presentation_component_reference(self) -> None:
        self.edit(
            "presentation/vertical_slice.json",
            lambda doc: doc["garage"]["default_build"].__setitem__(
                "tires", "sensor.missing"
            ),
        )
        with self.assertRaisesRegex(ContentError, "invalid component"):
            load_and_validate(self.assets)

    def test_rejects_presentation_for_unknown_level(self) -> None:
        self.edit(
            "presentation/vertical_slice.json",
            lambda doc: doc.__setitem__("level", "missing_level"),
        )
        with self.assertRaisesRegex(ContentError, "unknown level"):
            load_and_validate(self.assets)

    def test_rejects_bad_level_reference(self) -> None:
        self.edit(
            "levels/lost_signal.json", lambda doc: doc.__setitem__("map", "missing_map")
        )
        with self.assertRaisesRegex(ContentError, "unknown map"):
            load_and_validate(self.assets)

    def test_rejects_bad_scene_reference(self) -> None:
        self.edit(
            "maps/research_base.json",
            lambda doc: doc.__setitem__("scene_3d", "scene.missing"),
        )
        with self.assertRaisesRegex(ContentError, "unknown scene"):
            load_and_validate(self.assets)

    def test_rejects_bad_scene_prefab_reference(self) -> None:
        self.edit(
            "scenes/research_base.json",
            lambda doc: doc["instances"][0].__setitem__("prefab", "prefab.missing"),
        )
        with self.assertRaisesRegex(ContentError, "unknown prefab"):
            load_and_validate(self.assets)

    def test_rejects_bad_event_camera_weights(self) -> None:
        self.edit(
            "scenes/research_base.json",
            lambda doc: doc["camera"]["event_shot"]["focus_weights"].__setitem__(
                "actor", 0.9
            ),
        )
        with self.assertRaisesRegex(ContentError, "weights must sum to 1"):
            load_and_validate(self.assets)

    def test_rejects_missing_floor_texture(self) -> None:
        (self.assets / "textures/environment/concrete_floor.png").unlink()
        with self.assertRaisesRegex(ContentError, "missing texture"):
            load_and_validate(self.assets)

    def test_rejects_marker_outside_map(self) -> None:
        self.edit(
            "maps/research_base.json",
            lambda doc: doc["markers"].__setitem__("signal_tower", [99, 10, 0]),
        )
        with self.assertRaisesRegex(ContentError, "outside map bounds"):
            load_and_validate(self.assets)

    def test_rejects_duplicate_component_id(self) -> None:
        def duplicate(doc):
            doc["components"][1]["id"] = doc["components"][0]["id"]

        self.edit("components/tires.json", duplicate)
        with self.assertRaisesRegex(ContentError, "duplicate id"):
            load_and_validate(self.assets)

    def test_rejects_unlock_cycle(self) -> None:
        self.edit(
            "levels/boot.json", lambda doc: doc["prerequisites"].append("lost_signal")
        )
        with self.assertRaisesRegex(
            ContentError, "prerequisite must have a lower order|cycle"
        ):
            load_and_validate(self.assets)

    def test_rejects_build_that_cannot_fit_budget(self) -> None:
        self.edit(
            "levels/lost_signal.json",
            lambda doc: doc["loadout_rules"].__setitem__("budget", 1),
        )
        with self.assertRaisesRegex(
            ContentError, "no legal build|build cost exceeds level budget"
        ):
            load_and_validate(self.assets)

    def test_rejects_missing_component_model(self) -> None:
        (self.assets / "models/robots/ro1/lidar_fast.glb").unlink()
        with self.assertRaisesRegex(ContentError, "missing model"):
            load_and_validate(self.assets)


if __name__ == "__main__":
    unittest.main()
