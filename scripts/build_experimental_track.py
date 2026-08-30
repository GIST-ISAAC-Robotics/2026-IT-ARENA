#!/usr/bin/env python3
"""원본을 보존하면서 실험 트랙을 생성하거나 --check로 재현성을 검사합니다."""

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

import numpy as np
import yaml
from shapely.geometry import LineString, Polygon
from shapely.strtree import STRtree

from build_runtime_world import build_runtime_world
from experimental_facilities import configure, replace_facilities, update_scene
from validate_track import verify_preserved_sources


REPO = Path(__file__).resolve().parents[1]
SOURCE = REPO / "assets/track/source/it_arena_track"
DESTINATION = REPO / "src/arena_gazebo/worlds/it_arena_experimental"
PROFILE = REPO / "config/tracks/experimental.yaml"
VEHICLE = REPO / "src/arena_description/config/vehicle.yaml"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_generator():
    # 원본 디렉터리에 __pycache__도 만들지 않습니다. 상수 수정은 메모리 안에서만 합니다.
    spec = importlib.util.spec_from_file_location("organizer_track_generator", SOURCE / "track_gen.py")
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def derive_design(original: dict, profile: dict) -> dict:
    """중심선·분기 접속점·시설의 경로상 위치는 보존하고 폭·표지판 측면을 조정합니다."""
    main_width = float(profile["main_width_m"])
    branch_widths = [float(width) for width in profile["branch_widths_m"]]
    if not math.isfinite(main_width) or main_width <= 0:
        raise ValueError("본선 폭은 유한한 양수여야 합니다.")
    if len(branch_widths) != len(original.get("branches", [])):
        raise ValueError("지름길 폭의 개수가 원본 분기 개수와 다릅니다.")
    if any(not math.isfinite(width) or width <= 0 for width in branch_widths):
        raise ValueError("지름길 폭은 유한한 양수여야 합니다.")
    if not original.get("closed") or original.get("version", 0) < 3:
        raise ValueError("이 파생 지도는 원본의 닫힌 dense centerline 설계를 요구합니다.")
    if original.get("features", {}).get("narrow_zones"):
        raise ValueError("본선 협로가 있는 새 원본은 별도 검토 후 지원해야 합니다.")
    result = copy.deepcopy(original)
    result["track_width_m"] = main_width
    for branch, width in zip(result["branches"], branch_widths):
        branch["width_m"] = width
    markers = {int(marker["id"]): marker for marker in result["features"]["aruco"]}
    for marker_id, side in profile.get("marker_side_overrides", {}).items():
        if int(marker_id) not in markers or side not in ("left", "right"):
            raise ValueError("표지판 측면 변경은 기존 ID와 left/right만 지원합니다.")
        markers[int(marker_id)]["side"] = side
    return result


def rectangle(x, y, yaw, length, width) -> Polygon:
    c, s = math.cos(yaw), math.sin(yaw)
    return Polygon([
        (x + lx * c - ly * s, y + lx * s + ly * c)
        for lx, ly in [(-length / 2, -width / 2), (length / 2, -width / 2),
                       (length / 2, width / 2), (-length / 2, width / 2)]
    ])


def obstacle_polygons(world_path: Path) -> list[Polygon]:
    """실제 SDF 충돌체를 검사합니다. 노면·낮은 과속방지턱·상부 빔은 제외합니다."""
    model = ET.parse(world_path).find("./world/model[@name='it_arena_track_static']")
    obstacles = []
    for link in model.findall("link"):
        # 방지턱은 차량이 밟고 넘는 저층 노면입니다. 별도 곡면 높이·물리 검사로 검증합니다.
        if link.attrib.get("name", "").startswith("safety_bump_"):
            continue
        lp = [float(value) for value in link.findtext("pose", "0 0 0 0 0 0").split()]
        for collision in link.findall("collision"):
            size = collision.findtext("geometry/box/size")
            if size is None:
                continue
            sx, sy, sz = map(float, size.split())
            cp = [float(value) for value in collision.findtext("pose", "0 0 0 0 0 0").split()]
            if any(abs(value) > 1e-8 for value in (lp[3], lp[4], cp[3], cp[4])):
                raise ValueError("기울어진 충돌체는 현재 평면 검사기가 지원하지 않습니다.")
            z = lp[2] + cp[2]
            if z + sz / 2 <= 0.02 or z - sz / 2 >= 0.10:
                continue
            c, s = math.cos(lp[5]), math.sin(lp[5])
            obstacles.append(rectangle(lp[0] + cp[0] * c - cp[1] * s,
                                       lp[1] + cp[0] * s + cp[1] * c,
                                       lp[5] + cp[5], sx, sy))
    if not obstacles:
        raise ValueError("검사할 벽·시설 충돌체가 없습니다.")
    return obstacles


def inspect_geometry(world_path: Path, res: dict, footprint: dict) -> dict:
    obstacles = obstacle_polygons(world_path)
    spatial = STRtree(obstacles)
    length, width = footprint["length_m"], footprint["width_m"]

    def blocked(polygon):
        return any(polygon.intersection(obstacles[int(index)]).area > 1e-9
                   for index in spatial.query(polygon))

    slots = [rectangle(g["x"], g["y"], g["yaw"], length, width) for g in res["grid_slots"]]
    blocked_slots = [index for index, polygon in enumerate(slots) if blocked(polygon)]
    slot_pairs = [(i, j) for i in range(len(slots)) for j in range(i + 1, len(slots))
                  if slots[i].intersection(slots[j]).area > 1e-9]
    routes = [("main", res["arr"])] + [(f"branch_{i}", br["arr"])
                                                 for i, br in enumerate(res["branches"])]
    route_checks = {}
    for name, points in routes:
        collisions = [index for index, (x, y, yaw, _) in enumerate(points)
                      if blocked(rectangle(x, y, yaw, length, width))]
        route_checks[name] = {"samples": len(points), "blocked_samples": collisions}
    main_line = LineString(np.vstack([res["arr"][:, :2], res["arr"][0, :2]]))
    centerline_simple = bool(main_line.is_simple)
    branch_simple = [bool(LineString(br["arr"][:, :2]).is_simple) for br in res["branches"]]
    return {
        "obstacle_boxes": len(obstacles), "grid_slots_checked": len(slots),
        "blocked_grid_slots": blocked_slots, "overlapping_grid_pairs": slot_pairs,
        "centerline_simple": centerline_simple, "branch_centerlines_simple": branch_simple,
        "routes": route_checks,
        "static_footprint_checks_pass": not blocked_slots and not slot_pairs and centerline_simple
        and all(branch_simple) and all(not check["blocked_samples"] for check in route_checks.values()),
        "scope": "실제 SDF의 2D 충돌체와 직진 자세 20x15 cm 외형을 경로 표본마다 대조. 조향 궤적·완주 증명 아님.",
    }


def render_preview(res: dict, boxes: dict, target: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.patches import Polygon as Patch
    korean_font = None
    for font_path in (Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
                      Path("/mnt/c/Windows/Fonts/malgun.ttf")):
        if font_path.is_file():
            korean_font = font_manager.FontProperties(fname=str(font_path))
            break
    fig, axis = plt.subplots(figsize=(8, 10))
    for key, group in boxes.items():
        color = "#23829b" if "branch" in key else "#4c5664"
        if key.startswith("grass_"):
            color = "#b7cfad"
        if key.startswith("walls_"):
            color = "#1e2630"
        for box in group:
            axis.add_patch(Patch(np.asarray(rectangle(box["x"], box["y"], box["yaw"],
                                                      box["length"], box["width"]).exterior.coords),
                                 facecolor=color, edgecolor="none"))
    for grid in res["grid_slots"]:
        axis.text(grid["x"], grid["y"], str(grid.get("painted_number", grid["index"])), color="#f6c85f", fontsize=8,
                  ha="center", va="center")
    for bump in res["bumps"]:
        axis.add_patch(Patch(np.asarray(rectangle(bump["x"], bump["y"], bump["yaw"], bump["length"],
                                                  bump["width"]).exterior.coords), facecolor="#edc531", edgecolor="none"))
    if "finish_line" in res:
        finish = res["finish_line"]
        axis.add_patch(Patch(np.asarray(rectangle(finish["x"], finish["y"], finish["yaw_rad"], finish["depth_m"],
                                                  finish["width_m"]).exterior.coords), facecolor="white", edgecolor="black", linewidth=.3))
    for marker in res["markers"]:
        axis.plot(marker["x"], marker["y"], "s", color="#d83b49", markersize=4)
        axis.annotate(f"ID {marker['id']}", (marker["x"], marker["y"]),
                      xytext=(7, 0), textcoords="offset points", fontsize=7)
    widths = ", ".join(f"{br['width_m'] * 100:g}" for br in res["branches"])
    if korean_font:
        axis.set_title(f"실험용 트랙 · 공식 규격 아님\n"
                       f"본선 {res['meta']['track_w'] * 100:g} cm | 지름길 {widths} cm\n"
                       "원본 중심선과 경로 길이 유지", fontproperties=korean_font)
    else:
        axis.set_title(f"EXPERIMENTAL - not an official course\n"
                       f"Main {res['meta']['track_w'] * 100:g} cm | Shortcuts {widths} cm")
    axis.set(xlim=(0, 11), ylim=(0, 14.5), xlabel="x [m]", ylabel="y [m]", aspect="equal")
    axis.grid(alpha=0.15)
    fig.tight_layout()
    fig.savefig(target, dpi=150)
    plt.close(fig)


def prepare(profile_path: Path):
    preservation = verify_preserved_sources(REPO)
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if profile.get("profile") != "experimental" or profile.get("status") != "experimental_not_official":
        raise ValueError("실험용 상태 표시가 올바르지 않습니다.")
    resolution = float(profile["resolution_m"])
    if not math.isfinite(resolution) or not 0.002 <= resolution <= 0.05:
        raise ValueError("지도 해상도는 0.002~0.05 m/px 범위여야 합니다.")
    original = json.loads((SOURCE / "design_final.json").read_text(encoding="utf-8"))
    design = derive_design(original, profile)
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding="utf-8"))["vehicle"]
    footprint = {key: float(vehicle["footprint"][key]) for key in ("length_m", "width_m")}
    if min(profile["branch_widths_m"]) <= footprint["width_m"]:
        raise ValueError("실험 지름길은 차량 폭보다 넓어야 합니다.")
    generator = load_generator()
    generator.VEHICLE_L = generator.GRID_SLOT_L = footprint["length_m"]
    generator.VEHICLE_W = generator.GRID_SLOT_W = footprint["width_m"]
    result = generator.build_all_from_design(design, resolution=resolution)
    source_result = generator.build_all_from_design(original, resolution=resolution)
    np.testing.assert_array_equal(result["arr"], source_result["arr"])
    for branch, source_branch in zip(result["branches"], source_result["branches"]):
        np.testing.assert_array_equal(branch["arr"], source_branch["arr"])
    original_scene = json.loads((SOURCE / "output_final/scene.json").read_text(encoding="utf-8"))
    if round(result["meta"]["Ltot"], 4) != original_scene["track"]["lap_length_m"]:
        raise ValueError("원본 출력물과 재생성한 중심선의 한 바퀴 길이가 다릅니다.")
    facility_config = configure(result, profile, generator)
    provenance = {
        "profile": "experimental", "status": "experimental_not_official",
        "meeting_url": profile["meeting_url"], "meeting_date": profile["meeting_date"],
        "source_preservation": preservation,
        "source_design_sha256": sha256(SOURCE / "design_final.json"),
        "source_generator_sha256": sha256(SOURCE / "track_gen.py"),
        "derivation_script_sha256": sha256(Path(__file__)),
        "runtime_builder_sha256": sha256(REPO / "scripts/build_runtime_world.py"),
        "facility_builder_sha256": sha256(REPO / "scripts/experimental_facilities.py"),
        "profile_sha256": sha256(profile_path),
        "main_width_m": profile["main_width_m"], "branch_widths_m": profile["branch_widths_m"],
        "marker_side_overrides": {str(key): value for key, value in profile.get("marker_side_overrides", {}).items()},
        "vehicle_footprint_m": footprint, "centerlines_unchanged": True,
        "facilities": facility_config,
        "lap_length_m": original_scene["track"]["lap_length_m"],
        "changes": ["본선·분기 폭", "폭에 종속된 노면·벽·잔디·분기 입구",
                    "폭에 종속된 마커 횡방향 위치·신호등 지지대", "ID 30 표지판을 통로 반대편으로 이동",
                    "차량 메타데이터·노면 위 출발 6칸과 피니시 표시",
                    "도로 횡단 방향 신호등·낮은 등 높이·초기 빨강 단독 점등",
                    "진행 방향 20 cm 코사인 곡면 방지턱·같은 표본을 잇는 분할 물리 충돌면",
                    "진입 차량을 향하는 독립 ArUco 표지판·흰 여백·SDF 셀 형상",
                    "마커 10 cm의 기준을 검은 테두리 바깥 변으로 명시(원본 PNG 전체 크기와 구분)"],
        "not_changed": ["중심선 좌표", "길이 배율 1.0", "분기 접속 위치", "출발 위치",
                        "마커 ID", "벽 높이·두께", "잔디 폭·과속방지턱 높이"],
    }
    return generator, result, design, footprint, provenance


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--check", action="store_true", help="파일을 쓰지 않고 생성 결과·무결성·형상 검사")
    args = parser.parse_args()
    generator, res, design, footprint, provenance = prepare(args.profile)
    if args.check:
        saved = json.loads((DESTINATION / "provenance.json").read_text(encoding="utf-8"))
        for key, value in provenance.items():
            if saved.get(key) != value:
                raise ValueError(f"재생성이 필요한 입력 변경: {key}")
        for name, expected in saved["output_sha256"].items():
            path = (DESTINATION / name).resolve()
            if not path.is_relative_to(DESTINATION.resolve()) or sha256(path) != expected:
                raise ValueError(f"실험 출력 파일 해시 불일치: {name}")
        geometry = inspect_geometry(DESTINATION / "world.sdf", res, footprint)
        if not geometry["static_footprint_checks_pass"]:
            raise ValueError(f"정적 통과 공간 검사 실패: {geometry}")
        print(json.dumps({"preservation": provenance["source_preservation"], "geometry": geometry},
                         ensure_ascii=False, indent=2))
        return 0

    # 일회용 생성 위치는 build 아래에만 둡니다. 원본·원본 실행 월드를 출력으로 사용하지 않습니다.
    scratch_root = REPO / "build/track_generation"
    scratch_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="experimental_", dir=scratch_root) as temporary:
        raw = Path(temporary)
        boxes = generator.build_geometry_boxes(res)
        generator.write_csvs(res, str(raw))
        generator.write_map(res, boxes, str(raw), res["meta"]["resolution"])
        generator.write_sdf(res, boxes, str(raw))
        replace_facilities(raw / "world.sdf", res, generator)
        checks = generator.run_checks(res)
        scene = generator.write_scene_json(res, str(raw), checks)
        update_scene(scene, res, generator)
        scene["course_name"] = "IT ARENA 실험 코스 — 공식 규격 아님"
        scene["experimental_profile"] = provenance
        write_json(raw / "scene.json", scene)
        texture_dir = raw / "aruco"
        texture_dir.mkdir()
        for marker in res["markers"]:
            name = f"aruco_id{marker['id']}.png"
            shutil.copy2(SOURCE / "output_final/aruco" / name, texture_dir / name)
        geometry = inspect_geometry(raw / "world.sdf", res, footprint)
        if not geometry["static_footprint_checks_pass"]:
            raise ValueError(f"정적 통과 공간 검사 실패. 출력은 교체하지 않았습니다: {geometry}")
        render_preview(res, boxes, raw / "preview.png")
        runtime = build_runtime_world(raw, DESTINATION)
        if inspect_geometry(DESTINATION / "world.sdf", res, footprint) != geometry:
            raise ValueError("실행 월드 변환 전후 충돌 형상이 달라졌습니다.")
    write_json(DESTINATION / "design.json", design)
    provenance["geometry_checks"] = geometry
    provenance["organizer_checks"] = checks
    provenance["runtime_conversion"] = runtime
    provenance["output_sha256"] = {
        path.relative_to(DESTINATION).as_posix(): sha256(path)
        for path in sorted(DESTINATION.rglob("*")) if path.is_file() and path.name != "provenance.json"
    }
    write_json(DESTINATION / "provenance.json", provenance)
    verify_preserved_sources(REPO)
    print(json.dumps(provenance, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
