#!/usr/bin/env python3
"""공식 v2026.08.31 릴리스를 보존하며 Gazebo 실행 월드를 생성합니다.

노면·벽·중심선·분기 형상은 릴리스를 따릅니다. 현재 패키지의 ArUco 좌표가
‘벽 부착’ 안내와 다르고 ID 30이 지름길을 막는 문제는 벽 충돌체에 부착하는
실행용 보정으로 해결합니다. 보존 ZIP은 절대 수정하지 않습니다.
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
ARCHIVE = REPO / "assets/track/official/v2026.08.31/it_arena_track_v2026.08.31.zip"
PROFILE = REPO / "config/tracks/official_v2026.08.31.yaml"
DESTINATION = REPO / "src/arena_gazebo/worlds/it_arena_official"
VEHICLE = REPO / "src/arena_description/config/vehicle.yaml"
EXPECTED_ARCHIVE_SHA256 = "897183d2e2541458d190a0a1e3f76bff754c67fab894f2302c3110830a86149b"
EXPECTED_ARCHIVE_SIZE = 746_603
EXPECTED_MEMBER_COUNT = 23


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


def _closest_point_on_box(qx: float, qy: float, pose: list[float], size: list[float]) -> tuple[float, float]:
    """수평 회전한 벽 상자의 평면 외곽에서 q와 가장 가까운 점을 구합니다."""
    x, y, _, roll, pitch, yaw = pose
    if abs(roll) > 1e-9 or abs(pitch) > 1e-9:
        raise ValueError("기울어진 벽은 현재 마커 부착 검사가 지원하지 않습니다.")
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = qx - x, qy - y
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    half_x, half_y = size[0] / 2, size[1] / 2
    px = min(half_x, max(-half_x, local_x))
    py = min(half_y, max(-half_y, local_y))
    if abs(local_x) <= half_x and abs(local_y) <= half_y:
        gaps = [(half_x - local_x, half_x, local_y),
                (local_x + half_x, -half_x, local_y),
                (half_y - local_y, local_x, half_y),
                (local_y + half_y, local_x, -half_y)]
        _, px, py = min(gaps, key=lambda item: item[0])
    return x + c * px - s * py, y + s * px + c * py


def attach_markers_to_walls(world_path: Path, res: dict, design: dict, generator) -> list[dict]:
    """공식 README의 벽 부착 지침에 맞춰 실행본 마커를 가장 가까운 해당 벽면으로 옮깁니다."""
    tree = ET.parse(world_path)
    model = tree.find("./world/model[@name='it_arena_track_static']")
    if model is None:
        raise ValueError("공식 SDF에서 트랙 모델을 찾지 못했습니다.")
    walls = []
    for link in model.findall("link"):
        name = link.attrib.get("name", "")
        if not name.startswith("walls_"):
            continue
        link_pose = _floats(link.findtext("pose"))
        collision = link.find("collision")
        if collision is None:
            continue
        size = _floats(collision.findtext("geometry/box/size"), "0 0 0")
        walls.append((name, link_pose, size))
    if not walls:
        raise ValueError("마커를 부착할 벽 충돌체가 없습니다.")

    sides = {int(item["id"]): item.get("side", "left") for item in design["features"]["aruco"]}
    corrections = []
    for marker in res["markers"]:
        marker_id = int(marker["id"])
        qx, qy, tangent = generator.sample_at_s(res["arr"], marker["s"], res["meta"]["Ltot"])
        side = 1.0 if sides[marker_id] == "left" else -1.0
        normal = (-math.sin(tangent) * side, math.cos(tangent) * side)
        candidates = []
        for name, wall_pose, size in walls:
            projection = (wall_pose[0] - qx) * normal[0] + (wall_pose[1] - qy) * normal[1]
            if projection <= .05:
                continue
            wx, wy = _closest_point_on_box(qx, qy, wall_pose, size)
            distance = math.hypot(wx - qx, wy - qy)
            candidates.append((distance, name, wx, wy))
        if not candidates:
            raise ValueError(f"ArUco ID {marker_id}의 {sides[marker_id]} 벽을 찾지 못했습니다.")
        distance, wall_name, wx, wy = min(candidates)
        if not .20 < distance < .80:
            raise ValueError(f"ArUco ID {marker_id} 벽 검색 거리가 비정상입니다: {distance}")
        toward_track = ((qx - wx) / distance, (qy - wy) / distance)
        thickness = .009
        old = {"x": float(marker["x"]), "y": float(marker["y"]), "yaw_rad": float(marker["yaw"])}
        marker["x"] = float(wx + toward_track[0] * thickness / 2)
        marker["y"] = float(wy + toward_track[1] * thickness / 2)
        marker["yaw"] = float(math.atan2(toward_track[1], toward_track[0]))
        marker["approach_target"] = [float(qx), float(qy)]
        corrections.append({
            "id": marker_id,
            "side_from_design": sides[marker_id],
            "official_generated_pose": old,
            "runtime_wall_attached_pose": {
                "x": marker["x"], "y": marker["y"], "yaw_rad": marker["yaw"],
                "board_center_height_m": marker["center_height"],
            },
            "wall_link": wall_name,
            "route_center_to_wall_surface_m": distance,
            "reason": "track/README.md wall-attached guidance; generated poses float off-wall and ID30 obstructs shortcut 2",
        })
    return corrections


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
    if profile.get("profile") != "official_v2026.08.31":
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
    facility_config = configure(result, profile, generator)
    marker_corrections = attach_markers_to_walls(raw / "world.sdf", result, design, generator)
    replace_facilities(raw / "world.sdf", result, generator)
    scene = copy.deepcopy(source_scene)
    update_scene(scene, result, generator)
    scene["course_name"] = "IT ARENA official release v2026.08.31 runtime"
    scene["official_source"] = {
        "repository": profile["upstream_repository"], "commit": profile["upstream_commit"],
        "release": profile["upstream_release"], "asset": profile["release_asset"], **archive,
    }
    scene["runtime_corrections"] = {
        "official_geometry_unchanged": [
            "main and shortcut centerlines", "road/grass/wall geometry", "branch openings",
            "starting slot poses", "course ArUco IDs and s positions",
        ],
        "marker_wall_attachment": marker_corrections,
        "provisional_facilities": [
            "camera-visible low traffic-light body and deterministic simulator sequence",
            "5 cm raised-cosine speed-bump cross-section pending MeKENic STL",
            "grid U-lines/numbers and checker finish paint at design start_line.s (not ID0 s=0)",
        ],
        "friction": "official wall collisions retain mu=mu2=0.8; road, grass and bump have no injected coefficient",
    }
    write_json(raw / "scene.json", scene)
    shutil.copy2(source / "design_final.json", raw / "design.json")
    return profile, generator, result, design, {
        "archive": archive, "raw": raw, "marker_corrections": marker_corrections,
        "facility_config": facility_config,
    }


def build(profile_path: Path = PROFILE, destination: Path = DESTINATION) -> dict:
    scratch_root = REPO / "build/official_track_generation"
    scratch_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="v2026_08_31_", dir=scratch_root) as temporary:
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
            "marker_wall_attachment": prepared["marker_corrections"],
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
