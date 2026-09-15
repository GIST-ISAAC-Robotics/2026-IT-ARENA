"""공식 v2026.09.14 입력·보정 범위·실행 월드 회귀 검사."""

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
import build_official_track as official_builder  # noqa: E402


def test_pinned_official_release_is_preserved():
    assert ARCHIVE.stat().st_size == 805_607
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
    assert provenance["runtime_corrections"]["facilities"]["status"] == (
        "official_v2026.09.14_markers_unchanged_provisional_light_bump_finish"
    )
    verification = scene["verification"]
    assert verification["surface_coverage_ok"] is True
    assert verification["grass_coverage_ok"] is True
    assert verification["surface_missing_pct"] == .0538
    assert verification["surface_max_gap_m"] == .001
    assert verification["grass_missing_pct"] == .2311
    assert verification["grass_on_road_pct"] == .1305
    assert provenance["friction_scope"]["road"] == "unspecified_engine_default"
    for report in provenance["geometry_checks"].values():
        assert report["static_footprint_checks_pass"], report
        assert not any(route["blocked_samples"] for route in report["routes"].values())


def test_official_gantry_markers_are_unchanged_including_scene_and_pixels(tmp_path):
    with ZipFile(ARCHIVE) as archive:
        archive.extractall(tmp_path)
    official_builder.verify_preserved_markers(tmp_path / "output_final/world.sdf", DESTINATION / "world.sdf")
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    raw_scene = json.loads((tmp_path / "output_final/scene.json").read_text(encoding="utf-8"))
    assert scene["aruco_markers"] == raw_scene["aruco_markers"]
    assert scene["aruco_markers"]["mount_bottom_height_m"] == .35
    assert scene["aruco_markers"]["mount_center_height_m"] == .4
    provenance = json.loads((DESTINATION / "provenance.json").read_text(encoding="utf-8"))
    placement = provenance["runtime_corrections"]["marker_placement"]
    assert placement["status"] == "official_gantry_unchanged"
    assert {item["id"] for item in placement["placements"]} == {0, 20, 30, 45}
    assert all(not item["placement_changed"] for item in placement["placements"])


def test_marker_preservation_guard_rejects_yaw_change(tmp_path):
    with ZipFile(ARCHIVE) as archive:
        archive.extractall(tmp_path)
    raw = tmp_path / "output_final/world.sdf"
    tree = ET.parse(raw)
    link = tree.find("./world/model[@name='it_arena_track_static']/link[@name='aruco_30']")
    pose = link.find("pose")
    values = pose.text.split()
    values[5] = str(float(values[5]) + .1)
    pose.text = " ".join(values)
    tree.write(raw)
    with pytest.raises(ValueError, match="30.*변경"):
        official_builder.verify_preserved_markers(raw, DESTINATION / "world.sdf")


def test_provisional_gantry_supports_are_separate_and_clear_of_the_printed_face():
    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    supports = [link for link in model.findall("link") if link.attrib['name'].startswith('team_gantry_support_')]
    assert len(supports) == 4
    for link in supports:
        assert len(link.findall('collision')) == (6 if link.attrib['name'].endswith('_30') else 7)
        for collision in link.findall('collision'):
            xyz = list(map(float, collision.findtext('pose').split()))
            size = list(map(float, collision.findtext('geometry/box/size').split()))
            if 'post' in collision.attrib['name']:
                assert abs(xyz[1]) - size[1] / 2 >= .29 - 1e-9
            elif 'beam' in collision.attrib['name']:
                assert xyz[2] - size[2] / 2 > .45
            else:
                assert abs(xyz[1]) - size[1] / 2 >= .05 - 1e-9


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


def test_official_filled_starting_grid_visuals_are_preserved_without_provisional_numbers():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    merged = model.find("link[@name='track_geometry_merged']")
    assert model.find("link[@name='start_grid_paint']") is None
    assert not [
        visual for visual in model.findall(".//visual")
        if visual.attrib["name"].startswith("slot_")
    ]

    paint = scene["starting_grid"]["paint"]
    assert paint == {
        "representation": "official_filled_slots",
        "source": "official v2026.09.14 world.sdf grid_slot_0..5 visuals",
        "length_m": .25,
        "width_m": .17,
        "paint_center_height_m": .0037,
        "paint_thickness_m": .001,
        "material": "white",
        "collision": False,
        "numbering": "none",
    }

    with ZipFile(ARCHIVE) as archive:
        raw_model = ET.fromstring(archive.read("output_final/world.sdf")).find(
            "./world/model[@name='it_arena_track_static']"
        )
    for index in range(6):
        raw_link = raw_model.find(f"link[@name='grid_slot_{index}']")
        raw_visual = raw_link.find(f"visual[@name='grid_slot_{index}_vis']")
        runtime_visual = merged.find(f"visual[@name='grid_slot_{index}__grid_slot_{index}_vis']")
        assert runtime_visual is not None
        assert runtime_visual.findtext("pose") == raw_link.findtext("pose")
        assert runtime_visual.findtext("geometry/box/size") == raw_visual.findtext("geometry/box/size")
        assert runtime_visual.findtext("material/ambient") == raw_visual.findtext("material/ambient")
        assert runtime_visual.findtext("material/diffuse") == raw_visual.findtext("material/diffuse")
        assert merged.find(f"collision[@name='grid_slot_{index}__grid_slot_{index}_col']") is None


def test_runtime_facility_metadata_matches_geometry_and_selected_start_line():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    design = json.loads((DESTINATION / "design.json").read_text(encoding="utf-8"))
    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    beam = model.find("link[@name='tl_beam']/visual[@name='cross_track_beam']")
    beam_z = float(beam.findtext("pose").split()[2])
    assert scene["traffic_light"]["gantry_height_m"] == beam_z == .42
    assert scene["traffic_light"]["lamp_center_height_m"] == .30
    assert scene["traffic_light"]["provisional_route_offset_m"] == .60
    start = scene["start_finish"]
    assert math.hypot(scene["traffic_light"]["pose"]["x"] - start["pose"]["x"],
                      scene["traffic_light"]["pose"]["y"] - start["pose"]["y"]) > .59
    assert scene["speed_bumps"]["bump_length_m"] == .05
    assert scene["speed_bumps"]["bump_height_m"] == .01
    assert len(model.findall("link[@name='safety_bump_0']/collision")) == 20
    assert scene["finish_line"]["s_m"] == design["features"]["start_line"]["s"]
    assert scene["start_finish"]["s_m"] == 0
    marker_zero = next(marker for marker in scene["aruco_markers"]["markers"] if marker["id"] == 0)
    assert marker_zero["s_m"] == 0
    assert math.isclose(scene["finish_line"]["s_m"], marker_zero["s_m"])
