"""WSL의 ROS 환경에서 실행: python3 -m pytest tests -q"""

import copy
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from build_experimental_track import (  # noqa: E402
    DESTINATION, PROFILE, SOURCE, derive_design, inspect_geometry, prepare, sha256,
)
from build_runtime_world import build_runtime_world  # noqa: E402
from validate_track import verify_preserved_sources  # noqa: E402


@pytest.fixture(scope="module")
def prepared():
    return prepare(PROFILE)


def test_original_archive_and_all_extracted_files_preserved():
    assert verify_preserved_sources(REPO)["source_files_matched"] == 23


def test_design_changes_only_widths_and_documented_marker_side():
    original = json.loads((SOURCE / "design_final.json").read_text(encoding="utf-8"))
    before = copy.deepcopy(original)
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    derived = derive_design(original, profile)
    assert original == before
    restored = copy.deepcopy(derived)
    restored["track_width_m"] = original["track_width_m"]
    for branch, source_branch in zip(restored["branches"], original["branches"]):
        branch["width_m"] = source_branch["width_m"]
    restored["features"]["aruco"] = original["features"]["aruco"]
    assert restored == original
    assert derived["track_width_m"] == .45
    assert [branch["width_m"] for branch in derived["branches"]] == [.25, .25]
    for new, old in zip(derived["features"]["aruco"], original["features"]["aruco"]):
        expected = {**old, "side": "right"} if int(old["id"]) == 30 else old
        assert new == expected


@pytest.mark.parametrize("key,value", [("main_width_m", -1), ("main_width_m", float("nan")),
                                      ("branch_widths_m", [.25]), ("branch_widths_m", [.25, 0])])
def test_invalid_profile_is_rejected(key, value):
    original = json.loads((SOURCE / "design_final.json").read_text(encoding="utf-8"))
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    profile[key] = value
    with pytest.raises(ValueError):
        derive_design(original, profile)


def test_centerline_coordinates_and_distances_are_identical():
    for name, width in (("centerline.csv", .45), ("branch_0.csv", .25), ("branch_1.csv", .25)):
        original = np.loadtxt(SOURCE / "output_final" / name, delimiter=",", skiprows=1)
        derived = np.loadtxt(DESTINATION / name, delimiter=",", skiprows=1)
        # CSV에는 폭 열도 있으므로 파일 전체가 아니라 x/y/s가 동일해야 합니다.
        np.testing.assert_array_equal(derived[:, :3], original[:, :3])
        np.testing.assert_array_equal(derived[:, 3], np.full(len(derived), width))


def test_grid_and_marker_identity_are_preserved():
    original = json.loads((SOURCE / "output_final/scene.json").read_text(encoding="utf-8"))
    derived = json.loads((DESTINATION / "scene.json").read_text(encoding="utf-8"))
    assert original["starting_grid"]["slots"] == derived["starting_grid"]["slots"]
    for new, old in zip(derived["aruco_markers"]["markers"], original["aruco_markers"]["markers"]):
        assert (new["id"], new["s_m"]) == (old["id"], old["s_m"])
        assert sha256(DESTINATION / f"aruco/aruco_id{new['id']}.png") == sha256(SOURCE / f"output_final/aruco/aruco_id{new['id']}.png")


def test_all_generated_output_hashes_match():
    saved = json.loads((DESTINATION / "provenance.json").read_text(encoding="utf-8"))
    assert saved["status"] == "experimental_not_official"
    for name, expected in saved["output_sha256"].items():
        assert sha256(DESTINATION / name) == expected


def test_static_vehicle_clearance_on_actual_world(prepared):
    _, result, _, footprint, _ = prepared
    report = inspect_geometry(DESTINATION / "world.sdf", result, footprint)
    assert report["static_footprint_checks_pass"], report


def test_reverting_marker_override_exposes_obstruction(prepared, tmp_path):
    generator, _, design, footprint, _ = prepared
    original_side = copy.deepcopy(design)
    for marker in original_side["features"]["aruco"]:
        if int(marker["id"]) == 30:
            marker["side"] = "left"
    result = generator.build_all_from_design(original_side)
    generator.write_sdf(result, generator.build_geometry_boxes(result), str(tmp_path))
    report = inspect_geometry(tmp_path / "world.sdf", result, footprint)
    assert report["routes"]["branch_1"]["blocked_samples"]


def test_original_runtime_world_geometry_is_unchanged(tmp_path):
    build_runtime_world(SOURCE / "output_final", tmp_path)
    # 저장소의 줄바꿈 정규화 여부와 관계없이 같은 XML이어야 합니다.
    preserved = REPO / "src/arena_gazebo/worlds/it_arena_track/world.sdf"
    assert ET.canonicalize(from_file=preserved) == ET.canonicalize(from_file=tmp_path / "world.sdf")


def test_original_world_cannot_be_overwritten_by_experimental():
    with pytest.raises(ValueError, match="원본 재현용"):
        build_runtime_world(DESTINATION, REPO / "src/arena_gazebo/worlds/it_arena_track")


def test_organizer_assets_cannot_be_used_as_output():
    with pytest.raises(ValueError, match="보존된"):
        build_runtime_world(SOURCE / "output_final", SOURCE / "unexpected")


def test_vehicle_dimensions_and_steered_envelope(prepared):
    _, result, _, footprint, _ = prepared
    config = yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text(encoding="utf-8"))["vehicle"]
    drive = config["drivetrain"]
    assert drive["wheelbase_m"] == .145
    assert drive["track_width_m"] == .135
    assert drive["wheelbase_m"] + 2 * drive["wheel_radius_m"] <= footprint["length_m"]
    assert drive["track_width_m"] + drive["wheel_width_m"] <= footprint["width_m"]
    model = ET.fromstring(xacro.process_file(str(REPO / "src/arena_description/models/arena_car/model.sdf.xacro")).toxml())
    plugin = model.find("./model/plugin[@name='gz::sim::systems::AckermannSteering']")
    assert float(plugin.findtext("wheel_base")) == drive["wheelbase_m"]
    assert float(plugin.findtext("wheel_separation")) == drive["track_width_m"]
    angles = np.linspace(0, drive["max_steering_angle_rad"], 2001)
    half_x = max(drive["wheel_radius_m"] * np.cos(angles) + drive["wheel_width_m"] / 2 * np.sin(angles))
    half_y = max(drive["wheel_radius_m"] * np.sin(angles) + drive["wheel_width_m"] / 2 * np.cos(angles))
    swept = {"length_m": max(footprint["length_m"], drive["wheelbase_m"] + 2 * half_x),
             "width_m": max(footprint["width_m"], drive["track_width_m"] + 2 * half_y)}
    assert swept["width_m"] > footprint["width_m"]  # 조향 돌출을 검차 위반으로 판정하지 않습니다.
    report = inspect_geometry(DESTINATION / "world.sdf", result, swept)
    assert report["static_footprint_checks_pass"], report


def test_launch_exposes_original_and_experimental_profiles():
    path = REPO / "src/arena_bringup/launch/simulation.launch.py"
    spec = importlib.util.spec_from_file_location("arena_simulation_launch", path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    assert launch.TRACK_DIRECTORIES == {"original": "it_arena_track", "experimental": "it_arena_experimental"}
    arguments = {argument.name: argument for argument in launch.generate_launch_description().get_launch_arguments()}
    assert arguments["track"].default_value[0].text == "experimental"
