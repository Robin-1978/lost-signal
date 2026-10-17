#!/usr/bin/env python3

import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "compare_runs.py"


def artifact(computer: str = "computer.performance", seed: int = 47) -> dict:
    comparable = {
        "content_sha256": "content-1",
        "experiment": {
            "condition_id": "authored-base",
            "fixed_step_s": 0.02,
            "mission_id": "lost_signal",
            "mode": "mission",
            "physics_backend": "bullet",
            "requested_duration_s": None,
            "runtime_provider": "channel.in-process",
            "seed": seed,
        },
        "hardware": {
            "components": {"computer": computer},
            "robot": "robot.ro1",
            "style": 0.0,
        },
 "algorithms": {
 "global_planner": "astar-grid",
 "local_planner": "dwa-yaw-rate",
 "task_ordering": "task.balanced",
 },
    }
    packed = json.dumps(comparable, sort_keys=True, separators=(",", ":")).encode()
    return {
 "schema": {"name": "lost-signal.run", "version": 2},
        "producer": {"source_revision": "abc"},
        "configuration": {
            "comparable": comparable,
            "digest": hashlib.sha256(packed).hexdigest(),
        },
        "result": {"status": "succeeded", "time_s": 89.18, "collisions": 0},
    }


class CompareRunsTest(unittest.TestCase):
    def run_compare(
        self, left: dict, right: dict, *arguments: str
    ) -> subprocess.CompletedProcess:
        with tempfile.TemporaryDirectory() as temporary:
            first = Path(temporary) / "first.json"
            second = Path(temporary) / "second.json"
            first.write_text(json.dumps(left), encoding="utf-8")
            second.write_text(json.dumps(right), encoding="utf-8")
            return subprocess.run(
                ["python3", str(TOOL), str(first), str(second), *arguments],
                text=True,
                capture_output=True,
                check=False,
            )

    def test_accepts_like_for_like_runs(self):
        result = self.run_compare(artifact(), artifact())
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("compatible: yes", result.stdout)

    def test_accepts_one_declared_variable(self):
        result = self.run_compare(
            artifact("computer.light"),
            artifact("computer.performance"),
            "--vary",
            "hardware.components.computer",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("computer.light", result.stdout)

    def test_rejects_undeclared_and_invariant_differences(self):
        undeclared = self.run_compare(artifact("computer.light"), artifact())
        self.assertEqual(undeclared.returncode, 2)
        self.assertIn("without --vary", undeclared.stderr)
        invariant = self.run_compare(
            artifact(seed=47), artifact(seed=48), "--vary", "experiment.seed"
        )
        self.assertEqual(invariant.returncode, 2)
        self.assertIn("invariant input", invariant.stderr)

    def test_rejects_tampered_configuration(self):
        changed = artifact()
        changed["configuration"]["comparable"]["experiment"]["seed"] = 99
        result = self.run_compare(artifact(), changed)
        self.assertEqual(result.returncode, 2)
        self.assertIn("digest does not match", result.stderr)


if __name__ == "__main__":
    unittest.main()
