#!/usr/bin/env python3

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADLESS = (
    Path(sys.argv.pop(1))
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-")
    else None
)


class RunArtifactIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if HEADLESS is None:
            raise RuntimeError("usage: test_run_artifact.py HEADLESS_EXECUTABLE")
        cls.executable = HEADLESS

    def run_headless(self, *arguments: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [str(self.executable), "--assets", str(ROOT / "assets"), *arguments],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_manifest_reproduces_and_rejects_content_mismatch(self):
        with tempfile.TemporaryDirectory() as temporary:
            first = Path(temporary) / "first.run.json"
            second = Path(temporary) / "second.run.json"
            created = self.run_headless(
                "--level", "lost_signal", "--recommended", "--artifact", str(first)
            )
            self.assertEqual(created.returncode, 0, created.stderr)
            reproduced = self.run_headless(
                "--reproduce", str(first), "--artifact", str(second)
            )
            self.assertEqual(reproduced.returncode, 0, reproduced.stderr)

            baseline = json.loads(first.read_text(encoding="utf-8"))
            candidate = json.loads(second.read_text(encoding="utf-8"))
            self.assertEqual(
                baseline["schema"], {"name": "lost-signal.run", "version": 2}
            )
            self.assertEqual(
                baseline["configuration"]["comparable"]["algorithms"]["task_ordering"],
 "",
            )
            self.assertEqual(
                baseline["configuration"]["digest"],
                candidate["configuration"]["digest"],
            )
            self.assertEqual(baseline["result"], candidate["result"])
            self.assertEqual(baseline["events"], candidate["events"])
            self.assertEqual(baseline["decisions"], candidate["decisions"])

            comparable = baseline["configuration"]["comparable"]
            comparable["content_sha256"] = "0" * 64
            packed = json.dumps(
                comparable, sort_keys=True, separators=(",", ":")
            ).encode()
            baseline["configuration"]["digest"] = hashlib.sha256(packed).hexdigest()
            first.write_text(json.dumps(baseline), encoding="utf-8")
            rejected = self.run_headless("--reproduce", str(first))
            self.assertEqual(rejected.returncode, 1)
            self.assertIn("content hash does not match", rejected.stderr)


if __name__ == "__main__":
    unittest.main()
