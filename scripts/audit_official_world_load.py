#!/usr/bin/env python3
"""보존 ZIP에서 단일 변경 사본을 만들고 Gazebo 로드를 격리 재현합니다.

원본/운영 월드/사용자 Gazebo 설정은 수정하지 않습니다. 결과 디렉터리가
이미 있으면 덮어쓰지 않고 중단합니다. Linux(WSL)의 Gazebo CLI용입니다.
"""

from __future__ import annotations

import argparse
import copy
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
import zipfile


REPO = Path(__file__).resolve().parents[1]
ZIP = REPO / "assets/track/official/v2026.08.31/it_arena_track_v2026.08.31.zip"
ZIP_SHA256 = "897183d2e2541458d190a0a1e3f76bff754c67fab894f2302c3110830a86149b"
CASES = ("raw", "paths_only", "remove_script", "script_and_paths", "merge_static", "merge_only", "ready")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def geometry_signature(xml: str) -> str:
    """재질을 제외한 위치·접촉 설정이 병합 전 사본들에서 같은지 확인합니다."""
    root = ET.fromstring(xml)
    for visual in root.findall(".//visual"):
        for material in list(visual.findall("material")):
            visual.remove(material)
    # XML 정렬 차이로 해시가 바뀌지 않도록 공백뿐인 텍스트는 제외합니다.
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        element.tail = None
    return sha256(ET.tostring(root))


def flattened_geometry_signature(xml: str) -> str:
    """이번 평탄한 정적 모델에서 링크 병합 전후의 개별 형상을 대조합니다."""
    root = ET.fromstring(xml)
    records = []
    for link in root.findall("world/model/link"):
        link_pose = [float(v) for v in link.findtext("pose", "0 0 0 0 0 0").split()]
        for tag in ("visual", "collision"):
            for child in link.findall(tag):
                child_pose = [float(v) for v in child.findtext("pose", "0 0 0 0 0 0").split()]
                assert not (any(link_pose) and any(child_pose)), "일반 자세 합성은 이 진단 도구의 범위 밖입니다."
                item = copy.deepcopy(child)
                name = item.attrib["name"]
                if link.attrib["name"] == "audit_merged_static_geometry":
                    link_name, name = name.split("__", 1)
                    item.set("name", name)
                else:
                    link_name = link.attrib["name"]
                for field in ("pose", "material"):
                    for element in list(item.findall(field)):
                        item.remove(element)
                for element in item.iter():
                    element.tail = None
                    if element.text:
                        element.text = element.text.strip() or None
                records.append([link_name, tag, name, child_pose if any(child_pose) else link_pose,
                                ET.tostring(item, encoding="unicode")])
    return sha256(json.dumps(sorted(records), ensure_ascii=False).encode())


def merge_static(xml: str) -> str:
    """정적 링크만 병합하며 시스템·센서·시설 위치 보정은 추가하지 않습니다."""
    root = ET.fromstring(xml)
    model = root.find("world/model")
    assert model is not None
    merged = ET.Element("link", name="audit_merged_static_geometry")
    prefixes = ("ground_plane_", "surface_", "grass_", "walls_", "bump_", "grid_")
    for link in list(model.findall("link")):
        name = link.attrib["name"]
        if not name.startswith(prefixes):
            continue
        pose = link.findtext("pose", "0 0 0 0 0 0")
        for tag in ("collision", "visual"):
            for original in link.findall(tag):
                assert original.find("pose") is None
                child = copy.deepcopy(original)
                child.set("name", name + "__" + child.attrib["name"])
                ET.SubElement(child, "pose").text = pose
                merged.append(child)
        model.remove(link)
    model.insert(1, merged)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def white_marker_diffuse(xml: str) -> str:
    """기존 마커의 선택 재질 색만 명시하고 패턴/위치/조명은 유지합니다."""
    root = ET.fromstring(xml)
    materials = root.findall(".//visual/material[pbr]")
    assert len(materials) == 4
    for material in materials:
        assert material.find("diffuse") is None
        ET.SubElement(material, "diffuse").text = "1 1 1 1"
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def prepare(base: Path, sequential: bool = False) -> dict:
    if base.exists():
        raise FileExistsError(f"기존 감사 결과를 덮어쓰지 않습니다: {base}")
    archive_bytes = ZIP.read_bytes()
    assert sha256(archive_bytes) == ZIP_SHA256
    with zipfile.ZipFile(ZIP) as archive:
        entries = {info.filename: archive.read(info) for info in archive.infolist() if not info.is_dir()}
    world_names = [name for name in entries if name.endswith("output_final/world.sdf")]
    assert len(world_names) == 1
    world_name = world_names[0]
    raw = entries[world_name].decode("utf-8")
    assert raw.count("<script>") == 4
    assert raw.count("../aruco/") == 8
    without_script, count = re.subn(r"<script>.*?</script>", "", raw, flags=re.DOTALL)
    assert count == 4
    corrected = without_script.replace("../aruco/", "aruco/")
    variants = {
        "raw": raw,
        "paths_only": raw.replace("../aruco/", "aruco/"),
        "remove_script": without_script,
        "script_and_paths": corrected,
        "merge_static": merge_static(corrected),
    }
    if sequential:
        merged = merge_static(without_script)
        merged_paths = merged.replace("../aruco/", "aruco/")
        variants = {"raw": raw, "remove_script": without_script, "merge_only": merged,
                    "merge_static": merged_paths, "ready": white_marker_diffuse(merged_paths)}
    base.mkdir(parents=True)
    summary = {
        "source_zip": ZIP.relative_to(REPO).as_posix(),
        "source_zip_sha256": ZIP_SHA256,
        "source_world_sha256": sha256(entries[world_name]),
        "world_relative_path": world_name,
        "file_count": len(entries),
        "sequential_workflow": sequential,
        "all_source_files": {name: sha256(data) for name, data in entries.items()},
        "cases": {},
    }
    for case, xml in variants.items():
        directory = base / "fixtures" / case
        for name, data in entries.items():
            target = (directory / name).resolve()
            if not target.is_relative_to(directory.resolve()):
                raise ValueError(f"압축 항목 경로가 작업 디렉터리를 벗어납니다: {name}")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        world = directory / world_name
        if case != "raw":
            world.write_bytes(xml.encode("utf-8"))
        untouched = all((directory / name).read_bytes() == data for name, data in entries.items() if name != world_name)
        assert untouched
        root = ET.fromstring(xml)
        references = []
        for element in root.findall(".//albedo_map"):
            relative = element.text.strip()
            references.append({"uri": relative, "exists_relative_to_sdf": (world.parent / relative).is_file()})
        signature = geometry_signature(xml)
        flat_signature = flattened_geometry_signature(xml)
        assert flat_signature == flattened_geometry_signature(raw)
        if case in ("raw", "paths_only", "remove_script", "script_and_paths"):
            assert signature == geometry_signature(raw)
        (directory / "world_changes.diff").write_text(
            "".join(difflib.unified_diff(raw.splitlines(True), xml.splitlines(True), fromfile="release/world.sdf", tofile=f"{case}/world.sdf")),
            encoding="utf-8",
        )
        summary["cases"][case] = {
            "world": world.relative_to(base).as_posix(),
            "sha256": sha256(world.read_bytes()),
            "links": len(root.findall(".//link")),
            "collisions": len(root.findall(".//collision")),
            "visuals": len(root.findall(".//visual")),
            "explicit_plugins": len(root.findall(".//plugin")),
            "other_release_files_unchanged": untouched,
            "non_material_xml_signature": signature,
            "flattened_geometry_signature": flat_signature,
            "albedo_references": references,
        }
    assert (base / "fixtures/raw" / world_name).read_bytes() == entries[world_name]
    write_json(base / "fixtures.json", summary)
    return summary


def descendants_alive(pgid: int) -> list[int]:
    result = []
    for directory in Path("/proc").iterdir():
        if not directory.name.isdigit():
            continue
        try:
            pid = int(directory.name)
            if os.getpgid(pid) == pgid:
                # Zombies are reaped by their parent; they cannot execute.
                state = (directory / "stat").read_text().rsplit(")", 1)[1].split()[0]
                if state != "Z":
                    result.append(pid)
        except (ProcessLookupError, PermissionError, FileNotFoundError):
            pass
    return result


def sanitize_evidence_text(text: str) -> str:
    """완전한 ANSI 색 코드는 제거하고 중단된 ESC는 읽을 수 있게 남깁니다."""
    text = re.sub(r"\x1b\[[0-9;]*m", "", text)
    return (text.replace(str(REPO), "<WORKSPACE>").replace(str(Path.home()), "<WSL_HOME>")
            .replace("\b", "<0x08>").replace("\x1b", "<ESC>"))


def export_evidence(base: Path, destination: Path) -> dict:
    """관측 로그·진단 영상만 복사하고 개인 경로와 ANSI 제어문자를 제거합니다."""
    destination = destination.resolve()
    if not destination.is_relative_to((REPO / "artifacts/validation").resolve()):
        raise ValueError("공개용 근거는 이 저장소 artifacts/validation 아래에 저장합니다.")
    destination.mkdir(parents=True, exist_ok=False)
    fixtures = json.loads((base / "fixtures.json").read_text(encoding="utf-8"))
    raw_xml = (base / fixtures["cases"]["raw"]["world"]).read_text(encoding="utf-8")
    flat_signature = flattened_geometry_signature(raw_xml)
    for case in fixtures["cases"].values():
        actual = base / case["world"]
        assert sha256(actual.read_bytes()) == case["sha256"]
        assert flattened_geometry_signature(actual.read_text(encoding="utf-8")) == flat_signature
    sanitized = sanitize_evidence_text
    records = []
    for file in sorted((base / "runs").glob("*/result.json")):
        raw_result = json.loads(file.read_text(encoding="utf-8"))
        original_log = file.parent / "console.log"
        log = sanitized(original_log.read_text(encoding="utf-8", errors="replace"))
        target = destination / "logs" / (file.parent.name + ".log")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(log, encoding="utf-8")
        records.append({
            **{key: raw_result[key] for key in (
                "case", "mode", "command", "source_world_sha256", "cwd", "timeout_seconds",
                "observation_timeout_reached", "return_code", "wall_seconds_including_cleanup",
                "cleanup_signals", "remaining_test_pids")},
            "run_id": file.parent.name,
            "server_config_override": sanitized(raw_result.get("server_config_override") or "") or None,
            "error_codes": {str(code): log.count(f"Error Code {code}:") for code in (8, 9, 14)},
            "world_initialized_message_present": "World [it_arena_track] initialized" in log,
            "physics_loaded_message_present": "Loaded system [gz::sim::systems::Physics]" in log,
            "log": target.relative_to(destination).as_posix(),
            "unmodified_raw_log_sha256": sha256(original_log.read_bytes()),
        })
    render_reports = {}
    import shutil
    for source in sorted((base / "render").glob("*")):
        case = source.name
        if not (source / "result.json").exists():
            continue
        target = destination / "render" / case
        target.mkdir(parents=True)
        report = json.loads((source / "result.json").read_text(encoding="utf-8"))
        (target / "result.json").write_text(sanitized(json.dumps(report, ensure_ascii=False, indent=2)) + "\n", encoding="utf-8")
        for image_file in source.glob("marker_*.png"):
            expected = report["frames"][image_file.stem.split("_")[-1]]["sha256"]
            assert sha256(image_file.read_bytes()) == expected
            shutil.copy2(image_file, target / image_file.name)
        for log in source.glob("*.log"):
            (target / log.name).write_text(sanitized(log.read_text(encoding="utf-8", errors="replace")), encoding="utf-8")
        render_reports[case] = {"report": f"render/{case}/result.json", "capture_complete": report["capture_complete"],
                                "source_case": report["source_case"],
                                "physics_enabled": report.get("physics_enabled", False),
                                "detected_ids_by_target": {key: value["detected_ids"] for key, value in report["frames"].items()}}
    patches = destination / "patches"
    patches.mkdir()
    if fixtures.get("sequential_workflow"):
        for before_case, after_case, name in (
            ("raw", "remove_script", "01_remove_invalid_script.diff"),
            ("merge_only", "merge_static", "03_fix_texture_paths.diff"),
            ("merge_static", "ready", "04_set_diffuse_white.diff"),
        ):
            before_path = base / fixtures["cases"][before_case]["world"]
            after_path = base / fixtures["cases"][after_case]["world"]
            before = before_path.read_text(encoding="utf-8")
            after = after_path.read_text(encoding="utf-8")
            (patches / name).write_text("".join(difflib.unified_diff(
                before.splitlines(True), after.splitlines(True), fromfile=f"{before_case}/world.sdf",
                tofile=f"{after_case}/world.sdf")), encoding="utf-8")
    else:
        for case in (key for key in fixtures["cases"] if key != "raw"):
            shutil.copy2(base / "fixtures" / case / "world_changes.diff", patches / f"{case}.diff")
    pair = ((base / "fixtures/merge_static/output_final/world.sdf", base / "fixtures/ready/output_final/world.sdf")
            if fixtures.get("sequential_workflow") else
            (base / "render/path_fixed/world.sdf", base / "render/diffuse_white/world.sdf"))
    if all(path.exists() for path in pair):
        before, after = (path.read_text(encoding="utf-8") for path in pair)
        (patches / "diffuse_only.diff").write_text("".join(difflib.unified_diff(
            before.splitlines(True), after.splitlines(True), fromfile="path_fixed/world.sdf", tofile="diffuse_white/world.sdf")), encoding="utf-8")
    write_json(destination / "fixtures.json", fixtures)
    result = {
        "source_release": "v2026.08.31", "source_commit": "d61c5db9252cedfbc163cd044a47671df91e1660",
        "source_zip_sha256": ZIP_SHA256, "source_world_sha256": fixtures["source_world_sha256"],
        "source_file_count": fixtures["file_count"],
        "sequential_workflow": fixtures.get("sequential_workflow", False),
        "source_zip_unchanged": sha256(ZIP.read_bytes()) == ZIP_SHA256,
        "geometry_equal_across_all_five_fixtures": True, "flattened_geometry_signature": flat_signature,
        "environment": {"ubuntu": "24.04.4 LTS", "ros": "Jazzy", "gazebo_sim": "8.11.0", "sdformat": "14.9.0",
                        "physics_plugin": "gz::physics::dartsim::Plugin", "render_engine": "ogre2",
                        "default_server_systems": ["Physics", "UserCommands", "SceneBroadcaster"]},
        "scope": "배포 원본 파싱, 격리 수정별 첫 갱신, 정지 진단 RGB. 주행·실물·다른 OS/엔진 검증 아님.",
        "public_logs": "원시 로그에서 <WORKSPACE>/<WSL_HOME> 치환 및 ANSI 제어문자 제거. 원시는 build에 별도 보존.",
        "initial_helper_flag_correction": "초기 loaded_physics_log는 검색 문자열 불일치가 있었음. 이 보고서는 실제 로그 문구를 재대조. 원시 초기 JSON은 보존.",
        "runs": records, "render": render_reports,
        "upstream_writes": 0, "team_repository_push": False,
    }
    write_json(destination / "summary.json", result)
    return result


def run_case(base: Path, case: str, mode: str, timeout: float, run_id: str,
             server_config: Path | None = None, line_buffered: bool = False,
             iterations: int = 1) -> dict:
    summary = json.loads((base / "fixtures.json").read_text(encoding="utf-8"))
    world = base / summary["cases"][case]["world"]
    assert sha256(world.read_bytes()) == summary["cases"][case]["sha256"]
    destination = base / "runs" / f"{case}_{mode}_{run_id}"
    destination.mkdir(parents=True, exist_ok=False)
    partition = "arena_raw_audit_" + uuid.uuid4().hex[:12]
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    if server_config is not None:
        env["GZ_SIM_SERVER_CONFIG_PATH"] = str(server_config.resolve())
    if mode == "check":
        command = ["gz", "sdf", "-k", "world.sdf"]
    elif mode == "server":
        command = ["gz", "sim", "-s", "-r", "-v", "4", "--iterations", str(iterations), "world.sdf"]
    elif mode == "gui":
        command = ["gz", "sim", "-v", "4", "world.sdf"]
    else:
        raise ValueError(mode)
    if line_buffered:
        command = ["stdbuf", "-oL", "-eL", *command]
    log_path = destination / "console.log"
    start = time.monotonic()
    signals = []
    expired = False
    print(f"START {case}/{mode}: {' '.join(command)} timeout={timeout}s", flush=True)
    with log_path.open("wb") as log:
        process = subprocess.Popen(command, cwd=world.parent, env=env, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        write_json(destination / "started.json", {
            "pid": process.pid, "partition": partition, "case": case, "mode": mode,
            "world": str(world), "command": command, "timeout_seconds": timeout,
        })
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            expired = True
        finally:
            # Only this test's freshly created process group is eligible for cleanup.
            for sig, grace in ((signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
                alive = descendants_alive(process.pid)
                if not alive:
                    break
                os.killpg(process.pid, sig)
                signals.append(sig.name)
                deadline = time.monotonic() + grace
                while descendants_alive(process.pid) and time.monotonic() < deadline:
                    time.sleep(0.1)
            process.wait(timeout=2)
    duration = time.monotonic() - start
    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    clean = re.sub(r"\x1b\[[0-9;]*m", "", log_text)
    error_lines = [line for line in clean.splitlines() if any(s in line for s in ("Error", "[Err]", "[Wrn]", "Exception", "Unable to", "Failed"))]
    result = {
        "case": case, "mode": mode, "command": command,
        "source_world_sha256": summary["cases"][case]["sha256"],
        "cwd": world.parent.relative_to(REPO).as_posix(),
        "partition": partition, "timeout_seconds": timeout,
        "observation_timeout_reached": expired,
        "return_code": process.returncode, "wall_seconds_including_cleanup": round(duration, 4),
        "cleanup_signals": signals, "remaining_test_pids": descendants_alive(process.pid),
        "server_config_override": str(server_config) if server_config else None,
        "requested_iterations": iterations if mode == "server" else None,
        "errors_warnings": error_lines,
        "world_initialized_log": "Serving world controls" in clean,
        "loaded_physics_log": "Loaded system [gz::sim::systems::Physics]" in clean,
        "output_log": "console.log",
    }
    write_json(destination / "result.json", result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    assert not result["remaining_test_pids"], "감사 프로세스가 남았습니다."
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workdir", type=Path, default=REPO / "build/official_load_audit_20260831")
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--sequential", action="store_true", help="script 제거→링크 병합→경로→diffuse의 누적 수정 사본 생성")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=["raw"])
    parser.add_argument("--modes", nargs="+", choices=("check", "server", "gui"), default=["check", "server"])
    parser.add_argument("--timeout", type=float, default=45)
    parser.add_argument("--run-id", default="01")
    parser.add_argument("--server-config", type=Path)
    parser.add_argument("--line-buffered", action="store_true")
    parser.add_argument("--iterations", type=int, default=1)
    parser.add_argument("--export", type=Path, help="기존 실행 결과를 새 공개용 근거 폴더에 정리")
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("iterations는 1 이상이어야 합니다.")
    base = args.workdir.resolve()
    if not base.is_relative_to((REPO / "build").resolve()):
        parser.error("감사 작업 폴더는 이 저장소의 build 아래여야 합니다.")
    if args.export:
        summary = export_evidence(base, args.export)
        print(json.dumps({"export": str(args.export), "run_count": len(summary["runs"]),
                          "geometry_equal": summary["geometry_equal_across_all_five_fixtures"],
                          "render": summary["render"]}, ensure_ascii=False, indent=2))
        return 0
    if args.prepare:
        summary = prepare(base, sequential=args.sequential)
        print(json.dumps({k: v for k, v in summary.items() if k != "all_source_files"}, ensure_ascii=False, indent=2))
    for case in args.cases:
        for mode in args.modes:
            run_case(base, case, mode, args.timeout, args.run_id,
                     args.server_config, args.line_buffered, args.iterations)
    assert sha256(ZIP.read_bytes()) == ZIP_SHA256
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
