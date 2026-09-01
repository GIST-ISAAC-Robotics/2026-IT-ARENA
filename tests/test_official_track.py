"""공식 v2026.09.02 입력·보정 범위·실행 월드 회귀 검사."""

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
    assert ARCHIVE.stat().st_size == 883_631
    assert sha256(ARCHIVE) == EXPECTED_ARCHIVE_SHA256
    with ZipFile(ARCHIVE) as archive:
        assert sum(not item.is_dir() for item in archive.infolist()) == 24
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
        "official_v2026.09.02_geometry_with_provisional_marker_mount_and_facility_representation"
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


def test_course_markers_follow_print_sheet_and_use_documented_angled_wall_brackets():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    provenance = json.loads((DESTINATION / "provenance.json").read_text(encoding="utf-8"))
    markers = scene["aruco_markers"]
    assert markers["dictionary"] == "DICT_4X4_50"
    assert markers["printed_board_size_m"] == .10
    assert markers["black_code_size_m"] == .07
    assert markers["quiet_zone_each_side_m"] == .015
    assert markers["mount_bottom_height_m"] == .05
    assert markers["mount_type"] == "wall_attached_angled_bracket"
    assert {item["id"] for item in markers["markers"]} == {0, 20, 30, 45}
    placement = provenance["runtime_corrections"]["marker_placement"]
    assert placement["status"] == "team_provisional_angled_wall_bracket_pending_issue_12_meeting_decision"
    assert {item["id"] for item in placement["placements"]} == {0, 20, 30, 45}
    assert all(item["placement_changed"] for item in placement["placements"])
    for item in placement["placements"]:
        official = item["official_scene_pose"]
        runtime = item["runtime_link_pose_xyz_rpy"]
        assert not math.isclose(official["yaw_rad"], runtime[5], abs_tol=1e-3)
        assert item["bracket_length_m"] > 0
        assert item["official_input_verified_before_runtime_change"] is True

    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    for marker_id in (0, 20, 30, 45):
        link = model.find(f"link[@name='aruco_{marker_id}']")
        assert link is not None
        assert link.find("collision[@name='stand_collision']") is None
        visual = link.find(f"visual[@name='aruco_{marker_id}_vis']")
        collision = link.find(f"collision[@name='aruco_{marker_id}_col']")
        assert visual is not None and collision is not None
        np.testing.assert_allclose(
            list(map(float, visual.findtext("geometry/box/size").split())),
            [.005, .1, .1],
        )
        assert visual.find("material/script") is None
        assert visual.findtext("material/diffuse") == "1 1 1 1"
        assert visual.findtext("material/pbr/metal/albedo_map") == f"aruco/aruco_id{marker_id}.png"
        brackets = [visual for visual in link.findall("visual") if visual.attrib["name"].startswith("mounting_bracket_")]
        bracket_collisions = [
            collision for collision in link.findall("collision")
            if collision.attrib["name"].startswith("mounting_bracket_")
        ]
        assert len(brackets) == len(bracket_collisions) == 2
        assert link.find("visual[@name='sim_readable_white_face']") is not None
        assert len([
            item for item in link.findall("visual")
            if item.attrib["name"].startswith("sim_readable_ink_")
        ]) >= 20

    assert markers["rendering"] == (
        "official v2026.09.02 PNG/PBR board preserved with a matching thin SDF-cell front face"
    )


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
        "source": "official v2026.09.02 world.sdf grid_slot_0..5 visuals",
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
    assert scene["speed_bumps"]["bump_length_m"] == .05
    assert scene["speed_bumps"]["bump_height_m"] == .01
    assert len(model.findall("link[@name='safety_bump_0']/collision")) == 20
    assert scene["finish_line"]["s_m"] == design["features"]["start_line"]["s"]
    assert scene["start_finish"]["s_m"] == 0
    marker_zero = next(marker for marker in scene["aruco_markers"]["markers"] if marker["id"] == 0)
    assert marker_zero["s_m"] == 0
    assert math.isclose(scene["finish_line"]["s_m"], marker_zero["s_m"])
