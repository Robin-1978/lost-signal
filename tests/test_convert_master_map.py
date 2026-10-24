#!/usr/bin/env python3
import json
import subprocess
import tempfile
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="lost_signal_convert_") as td:
        base = Path(td)
        (base / "assets" / "maps").mkdir(parents=True)
        (base / "assets" / "surfaces.json").write_text(json.dumps({
            "materials": [{
                "id": "gravel", "friction_mu": 0.6,
                "rolling_drag": 0.02, "contact_time_constant_s": 0.01,
                "contact_damping_ratio": 1.0,
                "lidar_reflectivity": 0.3,
            }]
        }) + "\n")
        source = {
            "obstacles": [],
            "world_objects": [{
                "id": "mesh_prop", "position_m": [1, 2, 0],
                "collisions": [{"type": "mesh", "mesh": "prop.obj"}],
            }],
            "surfaces": [{"id": "road", "material": "gravel",
                           "min": [0, 0], "max": [4, 4]}],
            "terrain": [],
            "markers": {"dock": [1, 1, 0]},
            "actors": [{
                "id": "sleeping", "dormant_until_summoned": True,
                "path": [[0, 0]],
            }, {
                "id": "worker", "type": "patrol", "path": [[0, 0], [1, 0]],
                "visual": {"kind": "pedestrian"},
            }],
            "conflict_zones": [{"id": "crossing", "min": [0, 0],
                                "max": [2, 2]}],
        }
        (base / "assets" / "maps" / "fixture.json").write_text(
            json.dumps(source) + "\n")
        out = base / "out"
        result = subprocess.run([
            "python3", str(root / "tools" / "convert_master_map.py"),
            "fixture", "--src-dir", str(base), "--out-dir", str(out),
        ], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        world = json.loads((out / "fixture.json").read_text())
        semantics = json.loads((out / "fixture.semantics.json").read_text())
        actors = json.loads((out / "fixture.actors.json").read_text())
        assert len(world["boxes"]) == 0
        assert len(world["zones"]) == 1
        assert len(semantics["non_box_collisions"]) == 1
        assert len(semantics["conflict_zones"]) == 1
        assert [a["id"] for a in semantics["dormant_actors"]] == ["sleeping"]
        assert [a["id"] for a in actors["actors"]] == ["worker"]
        assert actors["actors"][0]["visual_kind"] == "pedestrian"
    print("convert_master_map ok")


if __name__ == "__main__":
    main()
