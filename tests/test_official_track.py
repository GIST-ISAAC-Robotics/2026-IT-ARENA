"""공식 v2026.11.01 입력·보정 범위·실행 월드 회귀 검사. 옛 보존본은 별도 대조."""

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
from official_2x5_facilities import lamp_channels, verify_official_signal  # noqa: E402

PREVIOUS_PROFILE = REPO / 'config/tracks/official_v2026.10.24.yaml'


def test_pinned_official_release_is_preserved():
    assert ARCHIVE.stat().st_size == 790_959
    assert sha256(ARCHIVE) == EXPECTED_ARCHIVE_SHA256
    with ZipFile(ARCHIVE) as archive:
        assert sum(not item.is_dir() for item in archive.infolist()) == 23
        assert {"design_final.json", "track_gen.py", "output_final/world.sdf", "output_final/scene.json"} <= set(archive.namelist())


def test_official_runtime_hash_manifest_is_current():
    assert check(DESTINATION)["status"] == "ok"


def test_previous_release_archive_remains_verifiable():
    previous = REPO / 'assets/track/official/v2026.09.14/it_arena_track_v2026.09.14.zip'
    assert official_builder.verify_archive(previous)['size_bytes'] == 805_607
    previous = REPO / 'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'
    assert official_builder.verify_archive(previous)['size_bytes'] == 812_042


def test_published_colors_survive_runtime_conversion():
    with ZipFile(ARCHIVE) as archive:
        source = ET.fromstring(archive.read('output_final/world.sdf'))
    model = ET.parse(DESTINATION / 'world.sdf').find("./world/model[@name='it_arena_track_static']")
    expected = {'ground_plane_link': [.55, .40, .26, 1],
                'track_surface': [.90, .90, .90, 1],
                'walls': [.50, .50, .50, 1], 'grass': [.13, .55, .13, 1]}
    for name, color in expected.items():
        link = source.find(f"./world/model/link[@name='{name}']")
        for visual in link.findall('visual'):
            original = visual.attrib['name']
            converted = model.find(f"link[@name='{name}']/visual[@name='{original}']")
            if converted is None:
                converted = model.find(f"link[@name='track_geometry_merged']/visual[@name='{name}__{original}']")
            assert converted is not None, (name, original)
            assert [float(x) for x in visual.findtext('material/diffuse').split()] == color
            assert converted.findtext('material/diffuse') == visual.findtext('material/diffuse')


def test_official_bump_removal_has_no_runtime_geometry():
    model = ET.parse(DESTINATION / 'world.sdf').find("./world/model[@name='it_arena_track_static']")
    assert not [link for link in model.findall('link') if 'bump' in link.attrib['name']]
    assert not (DESTINATION / 'meshes').exists()
    scene = json.loads((DESTINATION / 'scene.json').read_text(encoding='utf-8'))
    assert scene['speed_bumps']['count'] == 0 and scene['speed_bumps']['bumps'] == []


def test_published_mesh_cannot_silently_be_stretched_to_proposed_length(tmp_path):
    import yaml
    profile = yaml.safe_load(PREVIOUS_PROFILE.read_text(encoding='utf-8'))
    profile['facilities']['speed_bump']['length_m'] = .15
    with pytest.raises(ValueError, match='혼용'):
        official_builder.apply_published_bump_visual(tmp_path, profile)


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
        "official_v2026.11.01_2x5_signal_black_grid_no_bump_team_lamp_control_finish_no_track_post"
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


def test_official_signal_structure_is_kept_without_team_supports():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    assert model.find("link[@name='tl_post']") is None
    for name in ('tl_beam', 'tl_panel'):
        assert model.find(f"link[@name='{name}']") is not None
    assert not [link for link in model.findall("link") if link.attrib['name'].startswith('team_gantry_support_')]
    assert 'team_gantry_supports' not in scene
    for stale in ('lamp_red', 'lamp_yellow', 'lamp_green', 'lamp_red_1', 'lamp_red_4', 'safety_bump_0'):
        assert model.find(f"link[@name='{stale}']") is None


def test_lamp_links_keep_official_geometry_and_start_dark(tmp_path):
    with ZipFile(ARCHIVE) as archive:
        archive.extractall(tmp_path)
    raw = tmp_path / 'output_final/world.sdf'
    verify_official_signal(raw, DESTINATION / 'world.sdf', remove_track_post=True)
    source = ET.parse(raw).find("./world/model[@name='it_arena_track_static']")
    model = ET.parse(DESTINATION / 'world.sdf').find("./world/model[@name='it_arena_track_static']")
    channels = lamp_channels()
    assert len(channels) == 10
    for channel in channels:
        path = f"link[@name='{channel['name']}']/visual[@name='{channel['name']}_vis']/material/emissive"
        assert source.findtext(path) == channel['lit_rgba']
        assert model.findtext(path) == '0 0 0 1'
    scene = json.loads((DESTINATION / 'scene.json').read_text(encoding='utf-8'))
    assert scene['traffic_light']['lamp_channels'] == [
        {key: item[key] for key in ('name', 'column', 'row')} for item in channels]
    assert 'udp_port' not in scene['traffic_light']
    assert scene['traffic_light']['udp_visual_controller_connected'] is False


def test_signal_guard_rejects_moved_lamp_or_panel(tmp_path):
    with ZipFile(ARCHIVE) as archive:
        archive.extractall(tmp_path)
    raw = tmp_path / 'output_final/world.sdf'
    for name in ('tl_panel', 'lamp_green_top'):
        tree = ET.parse(raw)
        link = tree.find(f"./world/model[@name='it_arena_track_static']/link[@name='{name}']")
        pose = link.find('pose')
        original = pose.text
        values = pose.text.split()
        values[2] = str(float(values[2]) + .01)
        pose.text = ' '.join(values)
        moved = tmp_path / 'moved.sdf'
        tree.write(moved)
        with pytest.raises(ValueError, match=name):
            verify_official_signal(raw, moved)
        pose.text = original


def test_id45_uses_official_left_pose_not_the_old_right_shift():
    scene = json.loads((DESTINATION / 'scene.json').read_text(encoding='utf-8'))
    markers = {marker['id']: marker for marker in scene['aruco_markers']['markers']}
    assert markers[45]['role'] == 'traffic_light_left' and markers[0]['role'] == 'traffic_light_right'
    light = scene['traffic_light']['pose']
    left = (-math.sin(light['yaw_rad']), math.cos(light['yaw_rad']))
    offset = [(markers[i]['pose']['x']-light['x'])*left[0]+(markers[i]['pose']['y']-light['y'])*left[1] for i in (45, 0)]
    assert offset[0] == pytest.approx(.145, abs=.005) and offset[1] == pytest.approx(-.145, abs=.005)
    assert markers[45]['s_m'] == markers[0]['s_m'] == scene['traffic_light']['s_m']


def test_legacy_profile_can_still_generate_pillar_stress_geometry():
    import copy
    import yaml
    from experimental_facilities import add_gantry_supports, add_traffic_light

    config = yaml.safe_load(PREVIOUS_PROFILE.read_text(encoding='utf-8'))['facilities']
    legacy = copy.deepcopy(config)
    legacy['traffic_light'].pop('ground_post_enabled')
    legacy['gantry_supports']['enabled'] = True
    scene = json.loads((DESTINATION / 'scene.json').read_text(encoding='utf-8'))
    model = ET.parse(DESTINATION / 'world.sdf').find("./world/model[@name='it_arena_track_static']")
    # 다른 생성 시설을 변경하지 않고 옛 프로필의 생략 시 호환을 독립 확인합니다.
    lights = ET.Element('model', name='test_lights')
    res = {'experimental_facilities': legacy, 'meta': {'track_w': .45},
           'traffic_light': {**scene['traffic_light']['pose'], 'yaw': scene['traffic_light']['pose']['yaw_rad'], 'width': .45}}
    add_traffic_light(lights, res)
    assert len(lights.findall("link[@name='tl_post']/collision")) == 2
    add_gantry_supports(model, res)
    supports = [link for link in model.findall('link') if link.attrib['name'].startswith('team_gantry_support_')]
    assert len(supports) == 4
    assert [len(link.findall('collision')) for link in supports] == [7, 7, 6, 7]


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
        "source": "official v2026.11.01 world.sdf grid_slot_0..5 visuals",
        "length_m": .25,
        "width_m": .17,
        "paint_center_height_m": .0037,
        "paint_thickness_m": .001,
        "material": "black",
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
        assert raw_visual.findtext("material/diffuse") == "0.05 0.05 0.05 1"
        assert merged.find(f"collision[@name='grid_slot_{index}__grid_slot_{index}_col']") is None


def test_runtime_facility_metadata_matches_geometry_and_selected_start_line():
    scene = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    design = json.loads((DESTINATION / "design.json").read_text(encoding="utf-8"))
    model = ET.parse(DESTINATION / "world.sdf").find("./world/model[@name='it_arena_track_static']")
    panel = [float(value) for value in model.findtext("link[@name='tl_panel']/pose").split()]
    assert panel[2] == scene["traffic_light"]["lamp_center_height_m"] == .419
    assert scene["traffic_light"]["face_tilt_to_floor_deg"] == 60
    start = scene["start_finish"]
    assert scene["traffic_light"]["s_m"] == pytest.approx(46.271)
    assert math.hypot(scene["traffic_light"]["pose"]["x"] - start["pose"]["x"],
                      scene["traffic_light"]["pose"]["y"] - start["pose"]["y"]) > .35
    assert scene["finish_line"]["s_m"] == design["features"]["start_line"]["s"]
    assert scene["start_finish"]["s_m"] == 0
    assert next(marker for marker in scene["aruco_markers"]["markers"] if marker["id"] == 0)["s_m"] == pytest.approx(46.271)


def test_user_confirmed_post_removal_preserves_other_signal_geometry(tmp_path):
    with ZipFile(ARCHIVE) as archive:
        archive.extractall(tmp_path)
    raw = tmp_path / 'output_final/world.sdf'
    runtime = DESTINATION / 'world.sdf'
    source = ET.parse(raw).find("./world/model[@name='it_arena_track_static']")
    model = ET.parse(runtime).find("./world/model[@name='it_arena_track_static']")
    assert source.find("link[@name='tl_post']/collision[@name='tl_post_col']") is not None
    assert model.find("link[@name='tl_post']") is None
    assert not any('tl_post' in e.get('name', '') for e in model.iter())
    verify_official_signal(raw, runtime, remove_track_post=True)
    with pytest.raises(ValueError, match='tl_post'):
        verify_official_signal(raw, runtime)  # Removal must be explicit, not silently accepted.
    with pytest.raises(ValueError, match='tl_post'):
        verify_official_signal(raw, raw, remove_track_post=True)
    scene = json.loads((DESTINATION / 'scene.json').read_text(encoding='utf-8'))
    assert scene['traffic_light']['track_post_removed'] is True
    assert scene['traffic_light']['track_post_basis'] == 'user_confirmed_absent_2026_11_02'
