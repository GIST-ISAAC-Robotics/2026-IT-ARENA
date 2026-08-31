"""공식 v2026.08.31 입력·보정 범위·실행 월드 회귀 검사."""

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from build_official_track import (  # noqa: E402
    ARCHIVE, DESTINATION, EXPECTED_ARCHIVE_SHA256, check, sha256,
)
from experimental_facilities import marker_cells  # noqa: E402
import build_official_track as official_builder  # noqa: E402


def test_pinned_official_release_is_preserved():
    assert ARCHIVE.stat().st_size == 746_603
    assert sha256(ARCHIVE) == EXPECTED_ARCHIVE_SHA256
    with ZipFile(ARCHIVE) as archive:
        assert sum(not item.is_dir() for item in archive.infolist()) == 23
        assert {"design_final.json", "track_gen.py", "output_final/world.sdf", "output_final/scene.json"} <= set(archive.namelist())


def test_official_runtime_hash_manifest_is_current():
    assert check(DESTINATION)["status"] == "ok"


def test_vehicle_change_invalidates_official_clearance_report(monkeypatch, tmp_path):
    changed = tmp_path / "vehicle.yaml"
    changed.write_bytes(official_builder.VEHICLE.read_bytes() + b"\n# changed input\n")
    monkeypatch.setattr(official_builder, "VEHICLE", changed)
    with pytest.raises(ValueError, match="재생성"):
        check(DESTINATION)


def test_official_geometry_and_classification_are_explicit():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    provenance = json.loads((DESTINATION / "provenance.json").read_text(encoding="utf-8"))
    assert scene["track"]["width_m"] == .45
    assert [branch["width_m"] for branch in scene["branches"]] == [.2, .2]
    assert scene["track"]["lap_length_m"] == 46.6329
    assert provenance["official_values"]["vehicle_envelope_m"] == {"length": .2, "width": .15}
    assert provenance["runtime_corrections"]["facilities"]["status"] == "mixed_official_dimensions_and_provisional_runtime_geometry"
    assert provenance["friction_scope"]["road"] == "unspecified_engine_default"
    for report in provenance["geometry_checks"].values():
        assert report["static_footprint_checks_pass"], report
        assert not any(route["blocked_samples"] for route in report["routes"].values())


def test_course_markers_follow_print_sheet_and_wall_correction():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    provenance = json.loads((DESTINATION / "provenance.json").read_text(encoding="utf-8"))
    markers = scene["aruco_markers"]
    assert markers["dictionary"] == "DICT_4X4_50"
    assert markers["printed_board_size_m"] == .10
    assert markers["black_code_size_m"] == .07
    assert markers["quiet_zone_each_side_m"] == .015
    assert markers["mount_bottom_height_m"] == .05
    assert markers["mount_type"] == "wall_attached"
    assert {item["id"] for item in markers["markers"]} == {0, 20, 30, 45}
    corrections = provenance["runtime_corrections"]["marker_wall_attachment"]
    assert {item["id"] for item in corrections} == {0, 20, 30, 45}
    assert all(.40 < item["route_center_to_wall_surface_m"] < .45 for item in corrections)
    assert all(item["official_generated_pose"] != item["runtime_wall_attached_pose"] for item in corrections)

    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    for marker_id in (0, 20, 30, 45):
        link = model.find(f"link[@name='aruco_{marker_id}']")
        assert link is not None
        assert link.find("collision[@name='stand_collision']") is None
        backing = link.find("visual[@name='backing']")
        np.testing.assert_allclose(list(map(float, backing.findtext("geometry/box/size").split()))[1:], [.1, .1])
        actual = np.full((6, 6), 255, dtype=np.uint8)
        for visual in link.findall("visual"):
            if visual.attrib["name"].startswith("ink_"):
                _, row, col = visual.attrib["name"].split("_")
                actual[int(row), int(col)] = 0
        np.testing.assert_array_equal(actual, marker_cells(marker_id))


def test_only_official_walls_carry_track_friction_coefficients():
    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    specified = []
    for collision in model.findall(".//collision"):
        mu = collision.findtext("surface/friction/ode/mu")
        mu2 = collision.findtext("surface/friction/ode/mu2")
        if mu is not None or mu2 is not None:
            specified.append((collision.attrib["name"], mu, mu2))
    assert len(specified) == 694
    assert all(name.startswith("walls_") and math.isclose(float(mu), .8) and math.isclose(float(mu2), .8)
               for name, mu, mu2 in specified)
    bump = model.find("link[@name='safety_bump_0']")
    assert bump is not None and not bump.findall("collision/surface/friction")


def test_runtime_facility_metadata_matches_geometry_and_selected_start_line():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    design = json.loads((DESTINATION / "design.json").read_text(encoding="utf-8"))
    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    beam = model.find("link[@name='tl_beam']/visual[@name='cross_track_beam']")
    beam_z = float(beam.findtext("pose").split()[2])
    assert scene["traffic_light"]["gantry_height_m"] == beam_z == .42
    assert scene["traffic_light"]["lamp_center_height_m"] == .30
    assert scene["speed_bumps"]["bump_length_m"] == .05
    assert scene["speed_bumps"]["bump_height_m"] == .01
    assert len(model.findall("link[@name='safety_bump_0']/collision")) == 20
    assert scene["finish_line"]["s_m"] == design["features"]["start_line"]["s"]
    marker_zero = next(marker for marker in scene["aruco_markers"]["markers"] if marker["id"] == 0)
    assert marker_zero["s_m"] == 0
    assert not math.isclose(scene["finish_line"]["s_m"], marker_zero["s_m"])
