#!/usr/bin/env python3
"""Validate the preserved IT ARENA track package and report known inconsistencies."""

from __future__ import annotations

import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


EXPECTED_ZIP_SHA256 = "de448ba10c614e0f635d44b2f36bab29ebf455c323de442562dd01a8296758e4"


def fail(message: str) -> None:
    print(f"[fail] {message}", file=sys.stderr)
    raise SystemExit(1)


def expect_close(label: str, actual: float, expected: float, tolerance: float = 1e-6) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
        fail(f"{label}: expected {expected}, found {actual}")
    print(f"[ok] {label}: {actual}")


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    archive = repo / "assets" / "track" / "original" / "it_arena_track_final.zip"
    output = repo / "assets" / "track" / "source" / "it_arena_track" / "output_final"
    scene_path = output / "scene.json"
    world_path = output / "world.sdf"

    if not archive.is_file():
        fail(f"missing preserved archive: {archive}")

    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest != EXPECTED_ZIP_SHA256:
        fail(f"archive hash changed: {digest}")
    print(f"[ok] preserved ZIP SHA-256: {digest}")

    try:
        scene = json.loads(scene_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"cannot parse scene.json: {exc}")

    expect_close("lap length (m)", float(scene["track"]["lap_length_m"]), 46.6329)
    expect_close("main width (m)", float(scene["track"]["width_m"]), 0.35)

    branch_widths = [float(branch["width_m"]) for branch in scene["branches"]]
    if branch_widths != [0.12, 0.12]:
        fail(f"unexpected branch widths: {branch_widths}")
    print(f"[ok] branch widths (m): {branch_widths}")

    marker_ids = sorted(int(marker["id"]) for marker in scene["aruco_markers"]["markers"])
    if marker_ids != [0, 20, 30, 45]:
        fail(f"unexpected physical marker IDs: {marker_ids}")
    print(f"[ok] physical marker IDs: {marker_ids}")

    minimum_radius = float(scene["track"]["min_centerline_radius_general_m"])
    expect_close("general minimum radius (m)", minimum_radius, 0.298695418669103)
    if scene["verification"].get("min_radius_general_ok") is not False:
        fail("expected the supplied minimum-radius verification to be false")
    print("[known issue] supplied scene marks the general minimum-radius check as failed")

    try:
        ET.parse(world_path)
    except (OSError, ET.ParseError) as exc:
        fail(f"cannot parse world.sdf: {exc}")
    print("[ok] world.sdf is well-formed XML")

    missing = [
        path.name
        for path in (output / "aruco").glob("aruco_id*.png")
        if not path.is_file()
    ]
    if missing:
        fail(f"missing ArUco textures: {missing}")

    print("[warning] README values are not validated because they describe a different output revision")
    print("Track provenance and observed-output validation passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
