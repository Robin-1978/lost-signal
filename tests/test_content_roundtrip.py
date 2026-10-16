"""Require every repository JSON file to use the canonical save format.

Canonical = json.dumps(parse(text), indent=2, ensure_ascii=False) + "\\n".
The editor writes this format. Keeping hand-written content byte-compatible
with tool output prevents accidental whole-file reformatting.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path

ASSETS = Path(__file__).resolve().parent.parent / "assets"


def canonical(text: str) -> str:
    return json.dumps(json.loads(text), indent=2, ensure_ascii=False) + "\n"


class ContentRoundtripTests(unittest.TestCase):
    def test_every_json_is_canonical(self) -> None:
        files = sorted(ASSETS.rglob("*.json"))
        self.assertGreater(
            len(files), 40, "assets should contain more than 40 JSON files"
        )
        bad = []
        for path in files:
            original = path.read_text(encoding="utf-8")
            if canonical(original) != original:
                bad.append(str(path.relative_to(ASSETS)))
        self.assertEqual([], bad, "non-canonical JSON files:\n" + "\n".join(bad))

    def test_canonical_is_semantics_preserving(self) -> None:
        for path in sorted(ASSETS.rglob("*.json")):
            original = path.read_text(encoding="utf-8")
            self.assertEqual(
                json.loads(original), json.loads(canonical(original)), str(path)
            )


if __name__ == "__main__":
    unittest.main()
