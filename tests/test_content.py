#!/usr/bin/env python3

from __future__ import annotations

import copy
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
        self.assertEqual(22, len(catalog["levels"]))
        self.assertEqual(22, len(catalog["presentations"]))
        self.assertEqual(3, len(catalog["robots"]))
        self.assertEqual(2, len(catalog["scenes"]))
        self.assertEqual("boot", catalog["presentations"]["presentation.boot"]["level"])
        self.assertEqual("洛一", catalog["campaign"]["companion"]["display_name"])

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
        self.assertEqual(17, model_count)

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
            item = copy.deepcopy(doc["components"][0])
            doc["components"].append(item)

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
