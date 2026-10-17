#!/usr/bin/env python3
"""Keep the real-robot algorithm boundary independent of the game world."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
DEPLOYABLE_DIRS = (
    ROOT / "src/core",
    ROOT / "src/runtime",
    ROOT / "src/autonomy",
    ROOT / "src/executive",
 ROOT / "src/map_io",
)
FORBIDDEN_LAYERS = ("mission", "scenario", "sim", "physics", "client")


class DeployableBoundaryTest(unittest.TestCase):
    def test_deployable_sources_do_not_include_world_layers(self) -> None:
        include = re.compile(
            r'^\s*#\s*include\s*[<"](?:' + "|".join(FORBIDDEN_LAYERS) + r")/"
        )
        violations: list[str] = []
        for directory in DEPLOYABLE_DIRS:
            for source in sorted(directory.rglob("*")):
                if source.suffix not in {".h", ".hpp", ".c", ".cc", ".cpp"}:
                    continue
                for line_number, line in enumerate(source.read_text().splitlines(), 1):
                    if include.match(line):
                        violations.append(
                            f"{source.relative_to(ROOT)}:{line_number}: {line.strip()}"
                        )
        self.assertEqual(
            [], violations, "deployable code depends on game/simulator layers"
        )

    def test_autonomy_target_contains_only_deployable_sources(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        target = re.search(
            r"add_library\(lost_signal_autonomy\s+STATIC(?P<body>.*?)\n\)",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(target)
        sources = re.findall(r"src/([^\s)]+)", target.group("body"))
        self.assertTrue(sources)
        self.assertTrue(
            all(
                source.startswith(("autonomy/", "core/", "runtime/"))
                for source in sources
            ),
            f"lost_signal_autonomy contains a world-layer source: {sources}",
        )

        links = re.search(
            r"target_link_libraries\(lost_signal_autonomy\s+PUBLIC(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(links)
        for forbidden in (
            "lost_signal_scenario",
            "lost_signal_physics",
            "lost_signal_vehicle",
        ):
            self.assertNotIn(forbidden, links.group("body"))

    def test_executive_target_contains_only_deployable_sources(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        target = re.search(
            r"add_library\(lost_signal_executive\s+STATIC(?P<body>.*?)\n\)",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(target)
        sources = re.findall(r"src/([^\s)]+)", target.group("body"))
        self.assertTrue(sources)
        self.assertTrue(
            all(source.startswith("executive/") for source in sources),
            f"lost_signal_executive contains a world-layer source: {sources}",
        )

        links = re.search(
            r"target_link_libraries\(lost_signal_executive\s+PUBLIC(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(links)
        for forbidden in (
            "lost_signal_scenario",
            "lost_signal_physics",
            "lost_signal_vehicle",
        ):
            self.assertNotIn(forbidden, links.group("body"))


if __name__ == "__main__":
    unittest.main()
