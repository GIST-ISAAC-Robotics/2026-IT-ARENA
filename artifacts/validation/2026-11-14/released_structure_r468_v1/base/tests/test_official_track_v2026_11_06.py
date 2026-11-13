"""공식 v2026.11.06(610 mm 직각) 별도 실행 월드·빌더 등록·실행 선택 회귀 검사.

기존 v2026.11.01 실행 월드(it_arena_official)는 바이트 보존이며 기본 선택도 그대로다.
"""

import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import build_official_track as builder  # noqa: E402
from official_2x5_facilities import lamp_channels, verify_official_signal  # noqa: E402

PROFILE = REPO / "config/tracks/official_v2026.11.06.yaml"
WORLD = REPO / "src/arena_gazebo/worlds/it_arena_official_v2026_11_06"
ARCHIVE = REPO / "assets/track/official/v2026.11.06/it_arena_track_v2026.11.06.zip"
OLD_WORLD_SDF_SHA256 = "e4fe4d0d2fa13a1a60898e15a5b21cb36a3080835319dda09ccb97ae65c9e048"
QUERY = "./world/model[@name='it_arena_track_static']"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_new_release_is_registered_with_official_member_count_and_widths():
    assert builder.sha256(ARCHIVE) == "d1f3fbc1c16c6901130f260f5baa80ebf00040bcbaf3ccdb175e63738ed325fa"
    assert builder.verify_archive(ARCHIVE) == {"size_bytes": 691_524, "sha256": builder.sha256(ARCHIVE), "members": 25}
    spec = builder.release_spec("v2026.11.06")
    assert spec["main_width_m"] == .61 and spec["shortcut_widths_m"] == [.20, .20]
    # 이전 릴리스의 등록값과 무결성 검사는 그대로다.
    for release in ("v2026.09.14", "v2026.10.24", "v2026.11.01"):
        old = builder.release_spec(release)
        assert old["members"] == 23 and old["main_width_m"] == .45
        zip_path = REPO / f"assets/track/official/{release}/it_arena_track_{release}.zip"
        assert builder.verify_archive(zip_path)["members"] == 23


def test_member_count_mismatch_is_still_rejected(monkeypatch):
    sha, size, members, main, shortcuts = builder.RELEASES["v2026.11.06"]
    monkeypatch.setitem(builder.RELEASES, "v2026.11.06", (sha, size, 23, main, shortcuts))
    with pytest.raises(ValueError, match="파일 수"):
        builder.verify_archive(ARCHIVE)


def test_new_runtime_manifest_is_current_and_old_world_is_unchanged():
    result = builder.check(WORLD, PROFILE)
    assert result["status"] == "ok" and result["archive"]["members"] == 25
    assert builder.check(None, PROFILE)["status"] == "ok"  # 프로필 등록 경로
    assert builder.sha256(builder.DESTINATION / "world.sdf") == OLD_WORLD_SDF_SHA256
    old = builder.check(builder.DESTINATION)
    assert old["status"] == "ok"


def test_registered_runtime_worlds_cannot_be_cross_written():
    new_profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    old_profile = yaml.safe_load(builder.PROFILE.read_text(encoding="utf-8"))
    assert builder.runtime_destination(new_profile) == WORLD
    assert builder.runtime_destination(old_profile) == builder.DESTINATION
    with pytest.raises(ValueError):
        builder.runtime_destination(new_profile, builder.DESTINATION)
    with pytest.raises(ValueError):
        builder.runtime_destination(old_profile, WORLD)


def test_provenance_records_official_610mm_values():
    provenance = json.loads((WORLD / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["profile"] == "official_v2026.11.06"
    assert provenance["upstream"]["commit"] == "45691aec825ebfe4ff3edd9f221118e0c5026d1a"
    assert provenance["official_values"]["main_width_m"] == .61
    assert provenance["official_values"]["shortcut_widths_m"] == [.20, .20]
    assert provenance["official_values"]["lap_length_m"] == pytest.approx(44.64, abs=5e-4)
    assert all(report["static_footprint_checks_pass"] for report in provenance["geometry_checks"].values())
    scene = json.loads((WORLD / "scene.json").read_text(encoding="utf-8"))
    assert scene["track"]["width_m"] == .61
    assert [branch["width_m"] for branch in scene["branches"]] == [.20, .20]


def test_only_post_removed_lamps_dark_and_official_signal_markers_preserved(tmp_path):
    with ZipFile(ARCHIVE) as archive:
        archive.extractall(tmp_path)
    source = tmp_path / "output_final/world.sdf"
    verify_official_signal(source, WORLD / "world.sdf", remove_track_post=True)
    builder.verify_preserved_markers(source, WORLD / "world.sdf")
    official = ET.parse(source).find(QUERY)
    runtime = ET.parse(WORLD / "world.sdf").find(QUERY)
    assert official.find("link[@name='tl_post']") is not None
    assert runtime.find("link[@name='tl_post']") is None
    assert runtime.find("link[@name='tl_beam']") is not None and runtime.find("link[@name='tl_panel']") is not None
    for channel in lamp_channels():
        name = channel["name"]
        assert official.find(f"link[@name='{name}']/visual/material").findtext("emissive") == channel["lit_rgba"]
        material = runtime.find(f"link[@name='{name}']/visual[@name='{name}_vis']/material")
        assert material.findtext("emissive") == "0 0 0 1"
        assert material.findtext("diffuse") == material.findtext("ambient") == ".004 .004 .004 1"
    assert runtime.find("link[@name='finish_line_paint']") is not None
    assert ET.parse(WORLD / "world.sdf").getroot().find("world").get("name") == "it_arena_track"
    scene = json.loads((WORLD / "scene.json").read_text(encoding="utf-8"))
    assert scene["traffic_light"]["initial_state"] == "off"
    assert scene["traffic_light"]["track_post_removed"] is True
    official_scene = json.loads((tmp_path / "output_final/scene.json").read_text(encoding="utf-8"))
    assert scene["aruco_markers"] == official_scene["aruco_markers"]
    assert scene["starting_grid"]["slots"] == official_scene["starting_grid"]["slots"]


def test_signal_controller_links_exist_in_new_world():
    controller = load("traffic_light_controller_1106", REPO / "src/arena_gazebo/scripts/traffic_light_controller.py")
    runtime = ET.parse(WORLD / "world.sdf").find(QUERY)
    for lamp in controller.LAMPS:
        assert runtime.find(f"link[@name='{lamp['name']}']/visual[@name='{lamp['name']}_vis']") is not None


def test_launch_selects_new_world_only_explicitly():
    launch = load("arena_simulation_launch_1106", REPO / "src/arena_bringup/launch/simulation.launch.py")
    assert launch.TRACK_DIRECTORIES["official_v2026_11_06"] == WORLD.name
    assert launch.TRACK_DIRECTORIES["official"] == "it_arena_official"
    assert "official_v2026_11_06" in launch.OFFICIAL_2X5_TRACKS
    arguments = {a.name: a for a in launch.generate_launch_description().get_launch_arguments()}
    assert arguments["track"].default_value[0].text == "official"
    demo = load("arena_demo_launch_1106", REPO / "src/arena_bringup/launch/demo.launch.py")
    arguments = {a.name: a for a in demo.generate_launch_description().get_launch_arguments()}
    assert arguments["track"].default_value[0].text == "official"
    assert set(arguments["track"].choices) == {"official", "official_v2026_11_06"}


def test_runner_evaluates_the_selected_world():
    runner = load("validate_basic_autonomy_1106", REPO / "scripts/validate_basic_autonomy.py")
    assert runner.DEMO_TRACK == "official" and runner.DEMO_WORLD == builder.DESTINATION
    assert runner.TRACK_WORLDS["official_v2026_11_06"] == WORLD
    text = (REPO / "scripts/validate_basic_autonomy.py").read_text(encoding="utf-8")
    assert "extra_arguments.append('track:='+args.track)" in text
    assert "world=TRACK_WORLDS[args.track]" in text
