"""内容 JSON 存盘规范回归：仓库内所有 JSON 必须符合 canonical 格式。

Canonical = json.dumps(parse(text), indent=2, ensure_ascii=False) + "\\n"。
编辑器（plan §4 第 7 项）存盘即按此格式写；本测试保证手写 JSON 与工具存盘
逐字节一致，杜绝整文件重排事故（如 ro1.json 缩进惨案）。
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
        self.assertGreater(len(files), 40, "assets 应该有 40+ JSON 文件")
        bad = []
        for path in files:
            original = path.read_text(encoding="utf-8")
            if canonical(original) != original:
                bad.append(str(path.relative_to(ASSETS)))
        self.assertEqual([], bad, f"非 canonical JSON 文件:\n" + "\n".join(bad))

    def test_canonical_is_semantics_preserving(self) -> None:
        for path in sorted(ASSETS.rglob("*.json")):
            original = path.read_text(encoding="utf-8")
            self.assertEqual(json.loads(original), json.loads(canonical(original)), str(path))


if __name__ == "__main__":
    unittest.main()
