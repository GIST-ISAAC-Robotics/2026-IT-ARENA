#!/usr/bin/env python3
"""공식 v2026.09.02 릴리스를 보존하며 Gazebo 실행 월드를 생성합니다.

노면·잔디·벽·중심선·분기·그리드 슬롯은 릴리스를 따릅니다. 코스 ArUco의
ID·경로 위치·판·텍스처는 보존하되 방향은 #12 회의 결론 전 팀 시험용으로
기울입니다. 신호등과 방지턱 단면도 실물 자료 전 실행용 표현으로 분리합니다.
보존 ZIP은 절대 수정하지 않습니다.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile

import yaml

from build_experimental_track import inspect_geometry
from build_runtime_world import build_runtime_world
from experimental_facilities import configure, replace_facilities, update_scene


REPO = Path(__file__).resolve().parents[1]
ARCHIVE = REPO / "assets/track/official/v2026.09.02/it_arena_track_v2026.09.02.zip"
PROFILE = REPO / "config/tracks/official_v2026.09.02.yaml"
DESTINATION = REPO / "src/arena_gazebo/worlds/it_arena_official"
VEHICLE = REPO / "src/arena_description/config/vehicle.yaml"
EXPECTED_ARCHIVE_SHA256 = "6f74322703554e2dbe87598ce00a85332e1c6227353e82b1255180d2b12e12cb"
EXPECTED_ARCHIVE_SIZE = 883_631
EXPECTED_MEMBER_COUNT = 24


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def verify_archive() -> dict:
    if not ARCHIVE.is_file():
        raise FileNotFoundError(f"공식 릴리스 자산이 없습니다: {ARCHIVE}")
    digest = sha256(ARCHIVE)
    if digest != EXPECTED_ARCHIVE_SHA256 or ARCHIVE.stat().st_size != EXPECTED_ARCHIVE_SIZE:
        raise ValueError("공식 릴리스 ZIP 해시 또는 크기가 기준과 다릅니다.")
    with ZipFile(ARCHIVE) as archive:
        names = archive.namelist()
        file_count = sum(not info.is_dir() for info in archive.infolist())
        if file_count != EXPECTED_MEMBER_COUNT:
            raise ValueError(f"공식 ZIP 파일 수가 {EXPECTED_MEMBER_COUNT}개가 아닙니다.")
        for name in names:
            target = Path(name)
            if target.is_absolute() or ".." in target.parts:
                raise ValueError(f"안전하지 않은 ZIP 경로: {name}")
    return {"size_bytes": ARCHIVE.stat().st_size, "sha256": digest, "members": EXPECTED_MEMBER_COUNT}


def extract_archive(destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    with ZipFile(ARCHIVE) as archive:
        archive.extractall(destination)


def load_generator(source: Path):
    spec = importlib.util.spec_from_file_location("official_track_generator", source / "track_gen.py")
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def _floats(text: str | None, default: str = "0 0 0 0 0 0") -> list[float]:
    return [float(value) for value in (text or default).split()]


def verify_official_markers(world_path: Path, source_scene: dict) -> list[dict]:
    """v2026.09.02 원본 마커의 pose·판·재질을 팀 변경 전에 대조합니다."""
    tree = ET.parse(world_path)
    model = tree.find("./world/model[@name='it_arena_track_static']")
    if model is None:
        raise ValueError("공식 SDF에서 트랙 모델을 찾지 못했습니다.")
    scene_markers = {int(item["id"]): item for item in source_scene["aruco_markers"]["markers"]}
    if set(scene_markers) != {0, 20, 30, 45}:
        raise ValueError("공식 scene.json의 코스 마커 ID가 예상과 다릅니다.")
    placements = []
    for marker_id, scene_marker in sorted(scene_markers.items()):
        scene_pose = scene_marker["pose"]
        link = model.find(f"link[@name='aruco_{marker_id}']")
        if link is None:
            raise ValueError(f"공식 SDF에서 ArUco ID {marker_id} 링크를 찾지 못했습니다.")
        link_pose = _floats(link.findtext("pose"))
        expected = [float(scene_pose["x"]), float(scene_pose["y"]), float(scene_pose["z"]) + .05,
                    0.0, 0.0, float(scene_pose["yaw_rad"])]
        if not all(math.isclose(a, b, abs_tol=1e-4) for a, b in zip(link_pose, expected)):
            raise ValueError(f"ArUco ID {marker_id}의 생성 결과와 SDF pose가 다릅니다.")
        visual = link.find("visual")
        collision = link.find("collision")
        if visual is None or collision is None:
            raise ValueError(f"ArUco ID {marker_id}에 visual/collision이 모두 필요합니다.")
        board = _floats(collision.findtext("geometry/box/size"), "0 0 0")
        if not all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(board, [.005, .10, .10])):
            raise ValueError(f"ArUco ID {marker_id} 판 크기가 공식 5×100×100 mm와 다릅니다.")
        material = visual.find("material")
        uri = material.findtext("pbr/metal/albedo_map") if material is not None else None
        if material is None or material.find("script") is not None or material.findtext("diffuse") != "1 1 1 1":
            raise ValueError(f"ArUco ID {marker_id} 재질 수정이 릴리스에 완전히 반영되지 않았습니다.")
        if uri != f"aruco/aruco_id{marker_id}.png" or not (world_path.parent / uri).is_file():
            raise ValueError(f"ArUco ID {marker_id} 텍스처 경로가 올바르지 않습니다.")
        placements.append({
            "id": marker_id,
            "official_scene_pose": scene_pose,
            "official_sdf_board_center_pose": {
                "x": link_pose[0], "y": link_pose[1], "z": link_pose[2], "yaw_rad": link_pose[5],
            },
            "official_input_verified_before_runtime_change": True,
            "runtime_representation": "official v2026.09.02 PNG/PBR textured plate on a provisional angled wall bracket",
        })
    return placements


def swept_footprint(vehicle: dict) -> dict:
    footprint, drive = vehicle["footprint"], vehicle["drivetrain"]
    angle = float(drive["max_steering_angle_rad"])
    radius, wheel_width = float(drive["wheel_radius_m"]), float(drive["wheel_width_m"])
    # 0..극한 각에서 바퀴 바운딩 박스의 최댓값은 닫힌 형으로 안정적으로 산출합니다.
    candidate = min(angle, math.atan2(wheel_width, 2 * radius))
    half_x = max(radius, radius * math.cos(candidate) + wheel_width / 2 * math.sin(candidate))
    candidate = min(angle, math.atan2(2 * radius, wheel_width))
    half_y = max(wheel_width / 2, radius * math.sin(candidate) + wheel_width / 2 * math.cos(candidate))
    return {
        "length_m": max(float(footprint["length_m"]), float(drive["wheelbase_m"]) + 2 * half_x),
        "width_m": max(float(footprint["width_m"]), float(drive["track_width_m"]) + 2 * half_y),
    }


def prepare(profile_path: Path, scratch: Path) -> tuple[dict, object, dict, dict, dict]:
    archive = verify_archive()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if profile.get("profile") != "official_v2026.09.02":
        raise ValueError("공식 프로필 이름이 올바르지 않습니다.")
    if profile["release_sha256"] != archive["sha256"]:
        raise ValueError("프로필의 릴리스 해시가 보존 ZIP과 다릅니다.")
    source = scratch / "release"
    extract_archive(source)
    generator = load_generator(source)
    design = json.loads((source / "design_final.json").read_text(encoding="utf-8"))
    result = generator.build_all_from_design(design, resolution=.01)
    raw = scratch / "runtime_source"
    shutil.copytree(source / "output_final", raw)
    source_scene = json.loads((raw / "scene.json").read_text(encoding="utf-8"))
    if not math.isclose(float(result["meta"]["Ltot"]), float(source_scene["track"]["lap_length_m"]), abs_tol=5e-4):
        raise ValueError("공식 설계의 재생성 길이가 릴리스 scene과 다릅니다.")
    if float(result["meta"]["track_w"]) != .45 or [float(item["width_m"]) for item in result["branches"]] != [.20, .20]:
        raise ValueError("공식 본선 45 cm·지름길 20 cm 조건을 만족하지 않습니다.")
    marker_placements = verify_official_markers(raw / "world.sdf", source_scene)
    facility_config = configure(result, profile, generator)
    start_finish = source_scene.get("start_finish")
    if not start_finish or not math.isclose(float(start_finish["s_m"]), 0.0, abs_tol=1e-9):
        raise ValueError("v2026.09.02 공식 출발/결승 기준 s=0이 scene.json에 없습니다.")
    if not math.isclose(float(design["features"]["start_line"]["s"]), 0.0, abs_tol=1e-9):
        raise ValueError("v2026.09.02 design_final.json의 출발/결승 기준이 s=0이 아닙니다.")
    if not math.isclose(float(source_scene["starting_grid"]["longitudinal_stagger_m"]), .20, abs_tol=1e-9):
        raise ValueError("v2026.09.02 그리드 엇갈림 메타데이터가 실제 적용값 0.20 m가 아닙니다.")
    replace_facilities(raw / "world.sdf", result, generator)
    runtime_by_id = {item["id"]: item for item in result.get("runtime_marker_placements", [])}
    if set(runtime_by_id) != {0, 20, 30, 45}:
        raise ValueError("팀 시험용 코스 마커 네 개의 기울임 결과가 완전하지 않습니다.")
    for placement in marker_placements:
        placement.update(runtime_by_id[placement["id"]])
    scene = copy.deepcopy(source_scene)
    update_scene(scene, result, generator)
    scene["course_name"] = "IT ARENA official release v2026.09.02 team-test runtime"
    scene["official_source"] = {
        "repository": profile["upstream_repository"], "commit": profile["upstream_commit"],
        "release": profile["upstream_release"], "asset": profile["release_asset"], **archive,
    }
    scene["runtime_corrections"] = {
        "official_geometry_unchanged": [
            "main and shortcut centerlines", "road/grass/wall geometry", "branch openings",
            "starting slot poses and filled white visuals",
            "course ArUco IDs, route s, board dimensions and official PNG/PBR rendering",
        ],
        "marker_placement": {
            "status": "team_provisional_angled_wall_bracket_pending_issue_12_meeting_decision",
            "placements": marker_placements,
        },
        "provisional_facilities": [
            "course-marker yaw and wall bracket chosen for a 1.2 m upstream viewing point",
            "camera-visible low traffic-light body and deterministic simulator sequence",
            "5 cm raised-cosine speed-bump cross-section pending MeKENic STL",
            "checker finish paint at the official s=0 start/finish pose",
        ],
        "friction": "official wall collisions retain mu=mu2=0.8; road, grass and bump have no injected coefficient",
    }
    write_json(raw / "scene.json", scene)
    shutil.copy2(source / "design_final.json", raw / "design.json")
    return profile, generator, result, design, {
        "archive": archive, "raw": raw, "marker_placements": marker_placements,
        "facility_config": facility_config,
    }


def build(profile_path: Path = PROFILE, destination: Path = DESTINATION) -> dict:
    scratch_root = REPO / "build/official_track_generation"
    scratch_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="v2026_09_02_", dir=scratch_root) as temporary:
        profile, generator, result, design, prepared = prepare(profile_path, Path(temporary))
        raw = prepared["raw"]
        vehicle = yaml.safe_load(VEHICLE.read_text(encoding="utf-8"))["vehicle"]
        body = {key: float(vehicle["footprint"][key]) for key in ("length_m", "width_m")}
        steered = swept_footprint(vehicle)
        body_geometry = inspect_geometry(raw / "world.sdf", result, body)
        steered_geometry = inspect_geometry(raw / "world.sdf", result, steered)
        if not body_geometry["static_footprint_checks_pass"] or not steered_geometry["static_footprint_checks_pass"]:
            raise ValueError("공식 실행본의 차량 정적 통과 검사가 실패했습니다.")
        runtime = build_runtime_world(raw, destination)
        if inspect_geometry(destination / "world.sdf", result, body) != body_geometry:
            raise ValueError("실행 월드 병합 전후의 충돌 형상이 다릅니다.")
        shutil.copy2(raw / "design.json", destination / "design.json")
    provenance = {
        "profile": profile["profile"], "status": profile["status"],
        "upstream": {
            "repository": profile["upstream_repository"], "commit": profile["upstream_commit"],
            "release": profile["upstream_release"], "asset": profile["release_asset"],
            **prepared["archive"],
        },
        "official_values": {
            "main_width_m": .45, "shortcut_widths_m": [.20, .20],
            "lap_length_m": round(float(result["meta"]["Ltot"]), 6),
            "vehicle_envelope_m": {"length": .20, "width": .15},
            "grid_slots": 6, "grid_slot_size_m": {"length": .25, "width": .17},
            "course_marker_dictionary": "DICT_4X4_50", "course_marker_ids": [0, 20, 30, 45],
            "course_marker_printed_board_m": .10, "course_marker_black_code_m": .07,
        },
        "runtime_corrections": {
            "marker_placement": {
                "status": "team_provisional_angled_wall_bracket_pending_issue_12_meeting_decision",
                "placements": prepared["marker_placements"],
            },
            "facilities": prepared["facility_config"],
        },
        "friction_scope": {
            "preserved_official_wall_mu": .8,
            "road": "unspecified_engine_default", "grass": "unspecified_engine_default",
            "speed_bump": "unspecified_engine_default", "vehicle_tire": "separate_provisional_vehicle_model",
        },
        "geometry_checks": {"body_20x15_cm": body_geometry, "steered_envelope": steered_geometry},
        "runtime_conversion": runtime,
        "input_sha256": {
            "profile": sha256(profile_path), "builder": sha256(Path(__file__)),
            "vehicle_config": sha256(VEHICLE),
            "geometry_inspector": sha256(REPO / "scripts/build_experimental_track.py"),
            "runtime_builder": sha256(REPO / "scripts/build_runtime_world.py"),
            "facility_builder": sha256(REPO / "scripts/experimental_facilities.py"),
        },
    }
    provenance["output_sha256"] = {
        path.relative_to(destination).as_posix(): sha256(path)
        for path in sorted(destination.rglob("*")) if path.is_file() and path.name != "provenance.json"
    }
    write_json(destination / "provenance.json", provenance)
    print(json.dumps(provenance, ensure_ascii=False, indent=2))
    return provenance


def check(destination: Path = DESTINATION, profile_path: Path = PROFILE) -> dict:
    archive = verify_archive()
    saved = json.loads((destination / "provenance.json").read_text(encoding="utf-8"))
    if saved["upstream"]["sha256"] != archive["sha256"]:
        raise ValueError("생성된 월드의 공식 입력 해시가 현재 ZIP과 다릅니다.")
    expected_inputs = {
        "profile": sha256(profile_path), "builder": sha256(Path(__file__)),
        "vehicle_config": sha256(VEHICLE),
        "geometry_inspector": sha256(REPO / "scripts/build_experimental_track.py"),
        "runtime_builder": sha256(REPO / "scripts/build_runtime_world.py"),
        "facility_builder": sha256(REPO / "scripts/experimental_facilities.py"),
    }
    if saved["input_sha256"] != expected_inputs:
        raise ValueError("공식 실행 월드를 재생성해야 하는 입력 변경이 있습니다.")
    for name, expected in saved["output_sha256"].items():
        path = (destination / name).resolve()
        if not path.is_relative_to(destination.resolve()) or sha256(path) != expected:
            raise ValueError(f"공식 실행 월드 해시 불일치: {name}")
    result = {"archive": archive, "outputs": len(saved["output_sha256"]), "status": "ok"}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--destination", type=Path, default=DESTINATION)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if args.check:
        check(args.destination, args.profile)
    else:
        build(args.profile, args.destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
