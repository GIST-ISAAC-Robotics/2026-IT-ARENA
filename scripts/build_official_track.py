#!/usr/bin/env python3
"""고정 공식 릴리스를 보존하며 Gazebo 실행 월드를 생성합니다.

노면·잔디·벽·중심선·분기·그리드 슬롯은 릴리스를 따릅니다. 코스 ArUco의
SDF 링크 전체·위치·방향·텍스처와 scene 메타데이터를 보존합니다.
신호등은 팀 임시 표현을 유지합니다. 방지턱은 공개 STL 시각과
20구간 충돌 근사를 구분하며, 미확정 길이 완화안은 적용하지 않습니다.
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
import struct
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
ARCHIVE = REPO / "assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip"
PROFILE = REPO / "config/tracks/official_v2026.10.24.yaml"
DESTINATION = REPO / "src/arena_gazebo/worlds/it_arena_official"
VEHICLE = REPO / "src/arena_description/config/vehicle.yaml"
EXPECTED_ARCHIVE_SHA256 = "16b2f078609abea45ac3f351215df6576ced02bc6c40e933998ab91578de8a79"
EXPECTED_ARCHIVE_SIZE = 812_042
EXPECTED_MEMBER_COUNT = 23
RELEASES = {
    'v2026.09.14': ('a4ff2ea0ab99c217803fff799189d7e24794f1944faf21db56215517c4c5d814',805_607),
    'v2026.10.24': (EXPECTED_ARCHIVE_SHA256,EXPECTED_ARCHIVE_SIZE),
}


def profile_archive(profile):
    release=profile['upstream_release']
    if release not in RELEASES or profile.get('profile')!='official_'+release:
        raise ValueError('지원하지 않는 공식 릴리스 프로필입니다.')
    expected='it_arena_track_'+release+'.zip'
    if profile['release_asset']!=expected:
        raise ValueError('공식 자산 이름이 릴리스와 다릅니다.')
    return REPO/'assets/track/official'/release/expected


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def verify_archive(archive_path: Path = ARCHIVE) -> dict:
    if not archive_path.is_file():
        raise FileNotFoundError(f"공식 릴리스 자산이 없습니다: {archive_path}")
    expected_hash,expected_size=RELEASES[archive_path.parent.name]
    digest = sha256(archive_path)
    if digest != expected_hash or archive_path.stat().st_size != expected_size:
        raise ValueError("공식 릴리스 ZIP 해시 또는 크기가 기준과 다릅니다.")
    with ZipFile(archive_path) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)):
            raise ValueError("중복 ZIP 경로가 있습니다.")
        file_count = sum(not info.is_dir() for info in archive.infolist())
        if file_count != EXPECTED_MEMBER_COUNT:
            raise ValueError(f"공식 ZIP 파일 수가 {EXPECTED_MEMBER_COUNT}개가 아닙니다.")
        for name in names:
            target = Path(name)
            if target.is_absolute() or ".." in target.parts:
                raise ValueError(f"안전하지 않은 ZIP 경로: {name}")
    return {"size_bytes": archive_path.stat().st_size, "sha256": digest, "members": EXPECTED_MEMBER_COUNT}


def extract_archive(destination: Path, archive_path: Path = ARCHIVE) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    with ZipFile(archive_path) as archive:
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


def verify_official_markers(world_path: Path, source_scene: dict, release='v2026.09.14') -> list[dict]:
    """공식 원본 마커의 pose·판·재질을 실행 변환 전에 대조합니다."""
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
            "runtime_representation": f"unchanged official {release} gantry PNG/PBR plate",
            "placement_changed": False,
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


PUBLISHED_BUMP_PARTS = (
    ('speed_bump_225mm_h10_1_of_2.stl','cbdb4e19942d02683fcd27fdfa95b9f0ec03d1cd1e55ba9b1976d0d6772a61db',231.,-.225),
    ('speed_bump_225mm_h10_2_of_2.stl','4863f2861d87eb75f13e1e8a04dda42f92452c8b06c0e1a87c6317e71e001467',225.,0.),
)
PUBLISHED_BUMP_ROOT = REPO/'assets/track/official/v2026.10.24/upstream/parts/speed_bump'


def published_bump_inputs(profile):
    mode=profile['facilities']['speed_bump'].get('published_visual')
    if mode is None:return {}
    if mode!='official_two_piece_225mm_h10':raise ValueError('지원하지 않는 공개 방지턱 모델입니다.')
    return {'published_bump/'+name:sha256(PUBLISHED_BUMP_ROOT/name) for name,_,_,_ in PUBLISHED_BUMP_PARTS}


def apply_published_bump_visual(raw,profile):
    """공개 mm STL 두 조각만 시각에 이관한다. 기존 20구간 충돌 근사와 원본은 보존한다."""
    inputs=published_bump_inputs(profile)
    if not inputs:return None
    cfg=profile['facilities']['speed_bump']
    if cfg['length_m']!=.05 or cfg['height_m']!=.01:
        raise ValueError('공개 STL은 길이50 mm·높이10 mm이며 임의 완화 형상과 혼용할 수 없습니다.')
    tree=ET.parse(raw/'world.sdf')
    link=tree.find("./world/model[@name='it_arena_track_static']/link[@name='safety_bump_0']")
    if link is None or len(link.findall('collision'))!=cfg['subdivisions']:
        raise ValueError('방지턱 충돌 근사가 없습니다.')
    for visual in list(link.findall('visual')):link.remove(visual)
    details=[]
    for i,(name,expected,width,y) in enumerate(PUBLISHED_BUMP_PARTS,1):
        source=PUBLISHED_BUMP_ROOT/name
        if inputs['published_bump/'+name]!=expected:raise ValueError('공개 STL 해시가 다릅니다.')
        data=source.read_bytes();count=struct.unpack_from('<I',data,80)[0]
        if len(data)!=84+50*count or count<4:raise ValueError('binary STL 크기가 다릅니다.')
        vertices=[struct.unpack_from('<3f',data,84+50*j+12+12*k) for j in range(count) for k in range(3)]
        bounds=[min(v[k] for v in vertices) for k in range(3)]+[max(v[k] for v in vertices) for k in range(3)]
        if not all(math.isfinite(v) and math.isclose(v,e,abs_tol=1e-5) for v,e in zip(bounds,[0,0,0,50,width,10])):
            raise ValueError('공개 STL 좌표계/외형이 등록 치수와 다릅니다.')
        shutil.copy2(source,raw/'meshes'/name)
        visual=ET.SubElement(link,'visual',name=f'published_piece_{i}')
        ET.SubElement(visual,'pose').text=f'-0.025 {y} 0 0 0 0'
        mesh=ET.SubElement(ET.SubElement(visual,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text='meshes/'+name
        ET.SubElement(mesh,'scale').text='0.001 0.001 0.001'
        mat=ET.SubElement(visual,'material')
        for tag in ('ambient','diffuse'):ET.SubElement(mat,tag).text='0.9 0.6 0.1 1'
        ET.SubElement(mat,'specular').text='0 0 0 1'
        details.append(dict(source=source.relative_to(REPO).as_posix(),sha256=expected,
                            bounds_mm=bounds,translation_m=[-.025,y,0],scale=.001))
    ET.indent(tree,space='  ');tree.write(raw/'world.sdf',encoding='utf-8',xml_declaration=True)
    return dict(parts=details,rgba=[.9,.6,.1,1],assembly_size_m=[.05,.45,.01],
        collision='existing 20-strip cosine approximation; joints, compliance and adhesive not simulated',
        status='published 50mm model; proposed 100/150mm lengths are unconfirmed')


def prepare(profile_path: Path, scratch: Path) -> tuple[dict, object, dict, dict, dict]:
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    archive_path=profile_archive(profile)
    archive = verify_archive(archive_path)
    release=profile['upstream_release']
    if profile["release_sha256"] != archive["sha256"]:
        raise ValueError("프로필의 릴리스 해시가 보존 ZIP과 다릅니다.")
    source = scratch / "release"
    extract_archive(source,archive_path)
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
    marker_placements = verify_official_markers(raw / "world.sdf", source_scene,release)
    facility_config = configure(result, profile, generator)
    start_finish = source_scene.get("start_finish")
    if not start_finish or not math.isclose(float(start_finish["s_m"]), 0.0, abs_tol=1e-9):
        raise ValueError("공식 출발/결승 기준 s=0이 scene.json에 없습니다.")
    if not math.isclose(float(design["features"]["start_line"]["s"]), 0.0, abs_tol=1e-9):
        raise ValueError("공식 design_final.json의 출발/결승 기준이 s=0이 아닙니다.")
    if not math.isclose(float(source_scene["starting_grid"]["longitudinal_stagger_m"]), .20, abs_tol=1e-9):
        raise ValueError("공식 그리드 엇갈림 메타데이터가 실제 적용값 0.20 m가 아닙니다.")
    replace_facilities(raw / "world.sdf", result, generator)
    published_bump=apply_published_bump_visual(raw,profile)
    verify_preserved_markers(source / "output_final/world.sdf", raw / "world.sdf")
    scene = copy.deepcopy(source_scene)
    update_scene(scene, result, generator)
    if scene["aruco_markers"] != source_scene["aruco_markers"]:
        raise ValueError("공식 마커 scene 메타데이터가 변경되었습니다.")
    scene["starting_grid"]["paint"]["source"] = f"official {release} world.sdf grid_slot_0..5 visuals"
    scene["course_name"] = f"IT ARENA official release {release} team-test runtime"
    if published_bump:
        scene['speed_bumps']['published_visual']=published_bump
    scene["official_source"] = {
        "repository": profile["upstream_repository"], "commit": profile["upstream_commit"],
        "release": profile["upstream_release"], "asset": profile["release_asset"], **archive,
    }
    scene["runtime_corrections"] = {
        "official_geometry_unchanged": [
            "main and shortcut centerlines", "road/grass/wall geometry", "branch openings",
            "starting slot poses and filled white visuals",
            "complete course ArUco SDF links, poses, materials, PNGs and scene metadata",
        ],
        "marker_placement": {
            "status": "official_gantry_unchanged",
            "placements": marker_placements,
        },
        "provisional_facilities": [
            "camera-visible low traffic-light body and deterministic simulator sequence",
            ("published 50 mm x 10 mm orange STL visual; 20-strip collision approximation; length revision unresolved"
             if published_bump else "legacy provisional 5 cm raised-cosine striped speed bump"),
            "checker finish paint at the official s=0 start/finish pose",
            ("separate provisional gantry posts and overhead beams; official marker links unchanged"
             if facility_config.get("gantry_supports", {}).get("enabled", False)
             else "no added marker supports pending real construction photos; official marker links unchanged"),
            ("provisional traffic-light ground post retained for legacy stress comparison"
             if facility_config["traffic_light"].get("ground_post_enabled", True)
             else "provisional traffic-light ground post omitted; lamp and overhead assembly unchanged"),
        ],
        "friction": "official wall collisions retain mu=mu2=0.8; road, grass and bump have no injected coefficient",
    }
    write_json(raw / "scene.json", scene)
    shutil.copy2(source / "design_final.json", raw / "design.json")
    return profile, generator, result, design, {
        "archive": archive, "raw": raw, "marker_placements": marker_placements,
        "facility_config": facility_config,
    }


def verify_preserved_markers(source: Path, runtime: Path) -> None:
    """공백 정렬 외의 링크 변경과 PNG 변경을 거부합니다."""
    def canonical(link):
        return ET.canonicalize(ET.tostring(link, encoding="unicode"), strip_text=True)
    left, right = ET.parse(source), ET.parse(runtime)
    for marker_id in (0, 20, 30, 45):
        query = f"./world/model[@name='it_arena_track_static']/link[@name='aruco_{marker_id}']"
        a, b = left.find(query), right.find(query)
        if a is None or b is None or canonical(a) != canonical(b):
            raise ValueError(f"공식 마커 {marker_id} 링크가 변경되었습니다.")
        image = f"aruco/aruco_id{marker_id}.png"
        if sha256(source.parent / image) != sha256(runtime.parent / image):
            raise ValueError(f"공식 마커 {marker_id} PNG가 변경되었습니다.")


def build(profile_path: Path = PROFILE, destination: Path = DESTINATION) -> dict:
    scratch_root = REPO / "build/official_track_generation"
    scratch_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="official_", dir=scratch_root) as temporary:
        profile, generator, result, design, prepared = prepare(profile_path, Path(temporary))
        raw = prepared["raw"]
        vehicle = yaml.safe_load(VEHICLE.read_text(encoding="utf-8"))["vehicle"]
        body = {key: float(vehicle["footprint"][key]) for key in ("length_m", "width_m")}
        steered = swept_footprint(vehicle)
        body_geometry = inspect_geometry(raw / "world.sdf", result, body)
        steered_geometry = inspect_geometry(raw / "world.sdf", result, steered)
        if not body_geometry["static_footprint_checks_pass"] or not steered_geometry["static_footprint_checks_pass"]:
            print(json.dumps({"body": body_geometry, "steered": steered_geometry}, ensure_ascii=False, indent=2))
            raise ValueError("공식 실행본의 차량 정적 통과 검사가 실패했습니다.")
        runtime = build_runtime_world(raw, destination)
        verify_preserved_markers(raw / "world.sdf", destination / "world.sdf")
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
                "status": "official_gantry_unchanged",
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
            **published_bump_inputs(profile),
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
    profile=yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    archive = verify_archive(profile_archive(profile))
    saved = json.loads((destination / "provenance.json").read_text(encoding="utf-8"))
    if saved["upstream"]["sha256"] != archive["sha256"]:
        raise ValueError("생성된 월드의 공식 입력 해시가 현재 ZIP과 다릅니다.")
    expected_inputs = {
        "profile": sha256(profile_path), "builder": sha256(Path(__file__)),
        "vehicle_config": sha256(VEHICLE),
        "geometry_inspector": sha256(REPO / "scripts/build_experimental_track.py"),
        "runtime_builder": sha256(REPO / "scripts/build_runtime_world.py"),
        "facility_builder": sha256(REPO / "scripts/experimental_facilities.py"),
        **published_bump_inputs(profile),
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
