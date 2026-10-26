"""Refresh content fingerprints after an intentional GLB/GLTF rebuild.

This tool changes only ``content_hash``. Semantic frames, pivots, grounding,
budgets, licenses, and link bindings remain reviewable authored data. Run the
Blender ``inspect_assets.py`` gate immediately afterwards.
"""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODELS = ROOT / "assets" / "models"
MANIFEST = MODELS / "manifest.json"


def content_hash(path):
    value = 14695981039346656037
    for byte in path.read_bytes():
        value ^= byte
        value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return f"fnv1a64:{value:016x}"


def main():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    if manifest.get("format") != "lost-signal-assets/2":
        raise RuntimeError("unsupported asset manifest format")
    for asset in manifest["assets"]:
        path = MODELS / asset["path"]
        if not path.is_file():
            raise RuntimeError(f"missing asset: {asset['path']}")
        asset["content_hash"] = content_hash(path)
    MANIFEST.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")


if __name__ == "__main__":
    main()
