#!/usr/bin/env python3
"""Fail-closed validation for the external hardware release gate record."""

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

FORMAT = "lost-signal-hardware-release-gate/1"
STATUSES = {"PASS", "FAIL", "PENDING_EXTERNAL"}
GATE_NAMES = {
    "native_hil",
    "emergency_stop",
    "braking_distance",
    "payload_low_grip",
    "transport_latency",
    "lidar_range_profile",
    "calibration_artifact",
    "现场长时间回放",
}
SHA256 = re.compile(r"^[0-9a-fA-F]{64}$")


def fail(message: str) -> int:
    print(f"hardware release record: FAIL: {message}", file=sys.stderr)
    return 1


def require(mapping, key, label):
    if not isinstance(mapping, dict) or key not in mapping:
        raise ValueError(f"missing {label}.{key}")
    return mapping[key]


def validate(record: dict) -> list[str]:
    errors: list[str] = []
    gates = record.get("gates")
    provisional = (
        record.get("overall_status") == "PENDING_EXTERNAL"
        and isinstance(gates, dict)
        and set(gates) == GATE_NAMES
        and all(
            isinstance(gate, dict)
            and gate.get("status") == "PENDING_EXTERNAL"
            for gate in gates.values()
        )
    )
    if record.get("format") != FORMAT:
        errors.append(f"format must be {FORMAT}")
    for key in ("record_id", "captured_at_utc", "operator"):
        if not provisional and (not isinstance(record.get(key), str) or not record[key] or "REPLACE" in record[key]):
            errors.append(f"{key} must be a concrete value")

    board = record.get("board")
    for key in ("board_id", "firmware_revision", "profile_id", "robot_id", "robot_description_hash"):
        if not provisional and (not isinstance(board, dict) or not isinstance(board.get(key), str) or not board[key] or "REPLACE" in board[key]):
            errors.append(f"board.{key} must be a concrete value")
    if not provisional and (not isinstance(board, dict) or not isinstance(board.get("profile_revision"), int) or board["profile_revision"] <= 0):
        errors.append("board.profile_revision must be a positive integer")

    calibration = record.get("calibration")
    if not isinstance(calibration, dict):
        errors.append("calibration must be an object")
    else:
        if not provisional and calibration.get("provenance") != "measured":
            errors.append("calibration.provenance must be measured; nominal is commissioning-only")
        digest = calibration.get("artifact_sha256", "")
        if not provisional and (not isinstance(digest, str) or not SHA256.fullmatch(digest)):
            errors.append("calibration.artifact_sha256 must be a 64-character SHA-256")
        if not provisional and (not isinstance(calibration.get("sample_count"), int) or calibration["sample_count"] <= 0):
            errors.append("calibration.sample_count must be positive")

    if not isinstance(gates, dict) or set(gates) != GATE_NAMES:
        errors.append("gates must contain exactly the eight declared gate names")
        gates = gates if isinstance(gates, dict) else {}
    pending = False
    failed = False
    for name in GATE_NAMES:
        gate = gates.get(name)
        status = gate.get("status") if isinstance(gate, dict) else None
        evidence = gate.get("evidence_ref") if isinstance(gate, dict) else None
        if status not in STATUSES:
            errors.append(f"gates.{name}.status must be PASS, FAIL, or PENDING_EXTERNAL")
        if status == "PASS" and (not isinstance(evidence, str) or not evidence or "REPLACE" in evidence):
            errors.append(f"gates.{name} PASS requires evidence_ref")
        if status == "PENDING_EXTERNAL":
            pending = True
        if status == "FAIL":
            failed = True

    measurements = record.get("measurements")
    if not isinstance(measurements, dict):
        errors.append("measurements must be an object")
    else:
        for key in ("control_period_ns", "feedback_period_ns", "transport_latency_p95_ns", "transport_latency_max_ns"):
            if not provisional and (not isinstance(measurements.get(key), int) or measurements[key] <= 0):
                errors.append(f"measurements.{key} must be positive")
        for key in ("emergency_stop_distance_m", "braking_distance_empty_m", "braking_distance_loaded_m", "lidar_max_range_m"):
            if not provisional and (not isinstance(measurements.get(key), (int, float)) or measurements[key] <= 0):
                errors.append(f"measurements.{key} must be positive")
        if not provisional and (not isinstance(measurements.get("low_grip_surface"), str) or not measurements["low_grip_surface"] or measurements["low_grip_surface"] == "REPLACE"):
            errors.append("measurements.low_grip_surface must identify the measured surface")

    overall = record.get("overall_status")
    if overall not in STATUSES:
        errors.append("overall_status must be PASS, FAIL, or PENDING_EXTERNAL")
    if overall == "PASS" and (pending or failed or errors):
        errors.append("overall PASS is forbidden while any gate is pending, failed, or invalid")
    if pending and overall != "PENDING_EXTERNAL":
        errors.append("pending gates require overall_status=PENDING_EXTERNAL")
    if failed and overall != "FAIL":
        errors.append("failed gates require overall_status=FAIL")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("record", type=Path)
    args = parser.parse_args()
    try:
        record = json.loads(args.record.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return fail(str(exc))
    if not isinstance(record, dict):
        return fail("root must be an object")
    errors = validate(record)
    if errors:
        for error in errors:
            print(f"hardware release record: FAIL: {error}", file=sys.stderr)
        return 1
    digest = hashlib.sha256(args.record.read_bytes()).hexdigest()
    print(f"hardware release record: {record['overall_status']} sha256={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
