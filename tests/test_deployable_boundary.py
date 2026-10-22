#!/usr/bin/env python3
"""Enforce the one-way SIM -> ROBOT source and target dependency."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
ROBOT_DIR = ROOT / "src/robot"
SIM_HARDWARE_DIR = ROOT / "src/sim/hardware"


class DeployableBoundaryTest(unittest.TestCase):
    def test_sim_scenario_uses_robot_application_boundary(self) -> None:
        scenario_dir = ROOT / "src/sim/game/scenario"
        violations: list[str] = []
        forbidden = re.compile(
            r'(?:robot/autonomy/pipeline/autonomy_coordinator\.hpp|'
            r'\bAutonomyCoordinator\s*[&*]|\bcoordinator\s*\(\)|'
            r'autonomy_coordinator_)'
        )
        for source in sorted(scenario_dir.rglob("*")):
            if source.suffix not in {".h", ".hpp", ".c", ".cc", ".cpp"}:
                continue
            for line_number, line in enumerate(source.read_text().splitlines(), 1):
                if forbidden.search(line):
                    violations.append(f"{source.relative_to(ROOT)}:{line_number}: {line.strip()}")
        self.assertEqual([], violations,
            "SIM scenario must use RobotApplication, not the concrete autonomy coordinator")

    def test_robot_sources_do_not_include_sim(self) -> None:
        include = re.compile(r'^\s*#\s*include\s*[<"]sim/')
        violations: list[str] = []
        for source in sorted(ROBOT_DIR.rglob("*")):
            if source.suffix not in {".h", ".hpp", ".c", ".cc", ".cpp"}:
                continue
            for line_number, line in enumerate(source.read_text().splitlines(), 1):
                if include.match(line):
                    violations.append(
                        f"{source.relative_to(ROOT)}:{line_number}: {line.strip()}"
                    )
        self.assertEqual([], violations, "ROBOT code depends on SIM headers")

    def test_sim_hardware_sources_do_not_include_game(self) -> None:
        include = re.compile(r'^\s*#\s*include\s*[<"]sim/game/')
        violations: list[str] = []
        for source in sorted(SIM_HARDWARE_DIR.rglob("*")):
            if source.suffix not in {".h", ".hpp", ".c", ".cc", ".cpp"}:
                continue
            for line_number, line in enumerate(source.read_text().splitlines(), 1):
                if include.match(line):
                    violations.append(
                        f"{source.relative_to(ROOT)}:{line_number}: {line.strip()}"
                    )
        self.assertEqual([], violations, "SIM hardware code depends on game headers")

    def test_robot_targets_contain_only_robot_sources(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        target_pattern = re.compile(
            r"add_library\((lost_signal_robot_[a-z_]+)\s+(?:STATIC|INTERFACE)"
            r"(?P<body>[^)]*)\)"
        )
        targets = list(target_pattern.finditer(cmake))
        self.assertTrue(targets, "no ROBOT targets found")
        for target in targets:
            sources = re.findall(r"src/([^\s)]+)", target.group("body"))
            self.assertTrue(
                all(source.startswith("robot/") for source in sources),
                f"{target.group(1)} contains a SIM source: {sources}",
            )

    def test_robot_targets_do_not_link_sim_targets(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        links = re.compile(
            r"target_link_libraries\((lost_signal_robot_[a-z_]+)\s+"
            r"(?:PUBLIC|PRIVATE|INTERFACE)(?P<body>.*?)\)",
            re.DOTALL,
        )
        targets = list(links.finditer(cmake))
        self.assertTrue(targets, "no ROBOT link declarations found")
        for target in targets:
            self.assertNotRegex(
                target.group("body"),
                r"\blost_signal_sim_",
                f"{target.group(1)} links a SIM target",
            )

    def test_sim_hardware_targets_contain_only_hardware_sources(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        target_pattern = re.compile(
            r"add_library\((lost_signal_sim_(?:physics|vehicle|sensors|geometry))\s+STATIC"
            r"(?P<body>[^)]*)\)"
        )
        targets = list(target_pattern.finditer(cmake))
        self.assertEqual(4, len(targets), "expected four SIM hardware targets")
        for target in targets:
            sources = re.findall(r"src/([^\s)]+)", target.group("body"))
            self.assertTrue(
                all(source.startswith("sim/hardware/") for source in sources),
                f"{target.group(1)} contains a game source: {sources}",
            )

    def test_sim_hardware_targets_do_not_link_game_targets(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        links = re.compile(
            r"target_link_libraries\((lost_signal_sim_(?:physics|vehicle|sensors|geometry))\s+"
            r"(?:PUBLIC|PRIVATE|INTERFACE)(?P<body>.*?)\)",
            re.DOTALL,
        )
        targets = list(links.finditer(cmake))
        self.assertGreaterEqual(len(targets), 4, "no SIM hardware link declarations found")
        for target in targets:
            self.assertNotRegex(
                target.group("body"),
                r"\blost_signal_sim_(?:scenario|game|client)\b",
                f"{target.group(1)} links a game target",
            )

    def test_sim_game_target_contains_only_game_sources(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        target = re.search(
            r"add_library\(lost_signal_sim_game\s+STATIC(?P<body>[^)]*)\)",
            cmake,
        )
        self.assertIsNotNone(target, "no SIM game target found")
        sources = re.findall(r"src/([^\s)]+)", target.group("body"))
        self.assertTrue(sources, "SIM game target has no sources")
        self.assertTrue(
            all(source.startswith("sim/game/") for source in sources),
            f"lost_signal_sim_game contains a hardware source: {sources}",
        )

    def test_sim_hardware_aggregate_does_not_link_game(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        target = re.search(
            r"target_link_libraries\(lost_signal_sim_hardware\s+INTERFACE(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(target, "no SIM hardware aggregate found")
        self.assertNotRegex(target.group("body"), r"\blost_signal_sim_game\b")

    def test_robot_only_build_gate_is_declared(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn('option(LOST_SIGNAL_BUILD_SIM ', cmake)
        self.assertRegex(cmake, r"(?s)if\(NOT LOST_SIGNAL_BUILD_SIM\).*?return\(\)")


if __name__ == "__main__":
    unittest.main()
