#!/usr/bin/env python3
"""현재 UNO Q 소스·cross-build·오프라인 의존성을 해시 고정한 설치 묶음 생성."""
import argparse
import ast
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

from deploy_uno_q_bundle import SCHEMA, digest, safe_name, verify_directory

ROOT = Path(__file__).resolve().parents[1]
SKIP_PARTS = {"__pycache__", ".pytest_cache", ".git", "node_modules", "build", "install", "log"}
SOURCE_SUFFIXES = {".py", ".hpp", ".h", ".c", ".cpp", ".txt", ".conf", ".overlay", ".yaml", ".yml",
                   ".json", ".xml", ".md", ".sh", ".ps1", ".toml", ".csv", ".dts", ".rviz", ".xacro", ".urdf"}


def script_dependencies(root, entries):
    """Resolve local imports; external ROS/Python dependencies stay explicit."""
    pending, seen = list(entries), set()
    while pending:
        name = pending.pop()
        safe_name(name)
        path = root / name
        if name in seen:
            continue
        if not path.is_file():
            raise ValueError("required script missing: " + name)
        seen.add(name)
        if path.suffix != ".py":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=name)
        for node in ast.walk(tree):
            names = [alias.name for alias in node.names] if isinstance(node, ast.Import) else [node.module] if isinstance(node, ast.ImportFrom) and node.module else []
            for module in names:
                # Tests also import local fixtures, sometimes inside a test body.
                # Preserve the same-directory module before the scripts fallback.
                filename = module.split(".")[0] + ".py"
                for directory in (Path(name).parent, Path("scripts")):
                    dependency = (directory / filename).as_posix()
                    if (root / dependency).is_file():
                        pending.append(dependency)
                        break
    return seen


def collect_sources(root, entries):
    selected = script_dependencies(root, entries)
    for directory in ("src/arena_vehicle_interface", "src/arena_autonomy", "config/hardware"):
        base = root / directory
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            rel = path.relative_to(root)
            if path.is_file() and not (set(rel.parts) & SKIP_PARTS) and path.suffix in SOURCE_SUFFIXES:
                # Historical ESP32 programs are not a second install target.
                if any(part.startswith("firmware_") and part != "firmware_uno_q" for part in rel.parts):
                    continue
                selected.add(rel.as_posix())
            elif path.is_file() and (path.name in ("package.xml", "CMakeLists.txt", "setup.cfg") or "resource" in rel.parts):
                selected.add(rel.as_posix())
    return selected


def firmware_sources(root, firmware_dir):
    manifest = json.loads((firmware_dir / "firmware_manifest.json").read_text(encoding="utf-8"))
    sources = manifest.get("source_files")
    if not isinstance(sources, dict) or not sources:
        raise ValueError("firmware manifest must record source_files with SHA256 values")
    for relative, expected in sources.items():
        safe_name(relative)
        if digest(root / relative) != expected:
            raise ValueError("firmware source has changed after build: " + relative)
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict) or not all(name in artifacts for name in ("zephyr.bin", "zephyr.elf")):
        raise ValueError("firmware manifest must record zephyr.bin and zephyr.elf")
    for name, expected in artifacts.items():
        safe_name(name)
        if digest(firmware_dir / name) != expected:
            raise ValueError("firmware artifact hash mismatch: " + name)
    return manifest


def build_bundle(root, firmware_dir, wheelhouse, output, extra_entries=(), bench_firmware_dir=None):
    root, firmware_dir, wheelhouse, output = map(Path, (root, firmware_dir, wheelhouse, output))
    if output.exists():
        raise ValueError("output already exists; choose a fresh directory")
    required_entries = ["scripts/uno_q_host.py", "scripts/deploy_uno_q_bundle.py", "scripts/check_uno_q_jetson.py",
                        "scripts/prepare_uno_q_bundle.py", "scripts/runtime_installed_profile.py", *extra_entries]
    selected = collect_sources(root, required_entries)
    documentation = ["docs/hardware/UNO_Q_DEPLOYMENT_2026_11_14.md", "docs/hardware/schematics/uno_q_v1.json",
                     "docs/hardware/schematics/uno_q_pin_table_v1.md", "docs/hardware/schematics/uno_q_all_v1.svg"]
    selected.update(documentation)
    firmware_directories = {"unconfigured": firmware_dir}
    if bench_firmware_dir is not None:
        firmware_directories["bench"] = Path(bench_firmware_dir)
    firmware_manifests = {}
    for profile_name, directory in firmware_directories.items():
        firmware_manifests[profile_name] = firmware_sources(root, directory)
        selected.update(firmware_manifests[profile_name]["source_files"])
    wheels = sorted(wheelhouse.glob("*.whl"))
    if not any(path.name.startswith("pyserial-3.5-") for path in wheels):
        raise ValueError("offline pyserial 3.5 wheel required")
    for name in selected:
        path = root / name
        if not path.is_file() or path.is_symlink() or not path.resolve().is_relative_to(root.resolve()):
            raise ValueError("source missing, symlinked or outside repository: " + name)
    output.mkdir(parents=True)
    release = output / "bundle"
    release.mkdir()
    for name in sorted(selected):
        destination = release / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / name, destination)
    for profile_name, directory in firmware_directories.items():
        for name in (*firmware_manifests[profile_name]["artifacts"], "firmware_manifest.json"):
            target = release / "firmware" / profile_name / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(directory / name, target)
    (release / "wheelhouse").mkdir()
    for wheel in wheels:
        shutil.copyfile(wheel, release / "wheelhouse" / wheel.name)
    (release / "requirements-jetson.txt").write_text("pyserial==3.5\n", encoding="utf-8")
    try:
        git_head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        git_head = None
    inventory = {p.relative_to(release).as_posix(): {"size": p.stat().st_size, "sha256": digest(p)}
                 for p in sorted(release.rglob("*")) if p.is_file()}
    manifest = {"schema": SCHEMA, "created_utc": datetime.now(timezone.utc).isoformat(),
                "git_head": git_head, "source_identity": "files, including working-tree changes; git_head alone is insufficient",
                "scope": "UNO Q MCU/host and current autonomy source with optional quick sensor setup; not a full-course validated release",
                "physical_stop_included": False, "hardware_verified": False, "auto_arm": False,
                "firmware_profiles": list(firmware_directories),
                "files": inventory}
    (release / "bundle_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    verify_directory(release)
    archive = output / "uno_q_deployment.zip"
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as package:
        for path in sorted(release.rglob("*")):
            if path.is_file():
                info = zipfile.ZipInfo(path.relative_to(release).as_posix(), date_time=(2026, 11, 14, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                package.writestr(info, path.read_bytes())
    result = {"archive": str(archive), "sha256": digest(archive), "file_count": len(inventory),
              "hardware_access": False, "flash_executed": False}
    (output / "bundle_result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-dir", required=True, type=Path)
    parser.add_argument("--bench-firmware-dir", type=Path)
    parser.add_argument("--wheelhouse", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--entry", action="append", default=[], help="추가할 저장소 상대 script 경로")
    args = parser.parse_args()
    print(json.dumps(build_bundle(ROOT, args.firmware_dir, args.wheelhouse, args.output, args.entry, args.bench_firmware_dir), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
