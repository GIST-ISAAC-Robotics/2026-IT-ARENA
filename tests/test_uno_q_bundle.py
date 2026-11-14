"""배포 파일 무결성·기존 설치 보존·장치 없는 준비 경계 검사."""
import importlib.util
import json
from pathlib import Path
import stat
import sys
import zipfile

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import deploy_uno_q_bundle as deploy
import prepare_uno_q_bundle as prepare
import check_uno_q_jetson as preflight


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def fixture_bundle(tmp_path):
    root, firmware, wheels = (tmp_path / name for name in ("repo", "firmware", "wheels"))
    root.mkdir()
    firmware.mkdir()
    wheels.mkdir()
    for name in ("uno_q_host", "deploy_uno_q_bundle", "check_uno_q_jetson", "prepare_uno_q_bundle", "runtime_installed_profile"):
        write(root / f"scripts/{name}.py", "# fixture\n")
    write(root / "scripts/uno_q_host.py", "from local_dependency import value\n")
    write(root / "scripts/local_dependency.py", "value = 1\n")
    source = "src/arena_vehicle_interface/firmware_uno_q/src/main.cpp"
    write(root / source, "// test source snapshot\n")
    for name in ("UNO_Q_DEPLOYMENT_2026_11_14.md", "schematics/uno_q_v1.json", "schematics/uno_q_pin_table_v1.md",
                 "schematics/uno_q_all_v1.svg", "schematics/uno_q_wiring_v1.html"):
        write(root / "docs/hardware" / name, "fixture\n")
    write(firmware / "zephyr.bin", "test bin\n")
    write(firmware / "zephyr.elf", "test elf\n")
    manifest = {"source_files": {source: deploy.digest(root / source)},
                "artifacts": {name: deploy.digest(firmware / name) for name in ("zephyr.bin", "zephyr.elf")}}
    write(firmware / "firmware_manifest.json", json.dumps(manifest))
    write(wheels / "pyserial-3.5-py2.py3-none-any.whl", "test wheel fixture\n")
    return root, firmware, wheels


def test_bundle_roundtrip_and_existing_destination_preserved(tmp_path):
    root, firmware, wheels = fixture_bundle(tmp_path)
    result = prepare.build_bundle(root, firmware, wheels, tmp_path / "output")
    unpacked = tmp_path / "unpacked"
    unpacked.mkdir()
    deploy.extract_archive(result["archive"], unpacked)
    manifest = deploy.verify_directory(unpacked)
    assert "scripts/local_dependency.py" in manifest["files"]
    assert manifest["auto_arm"] is False
    assert manifest["hardware_verified"] is False
    destination = tmp_path / "installed"
    deploy.install_directory(unpacked, destination)
    before = (destination / "bundle_manifest.json").read_bytes()
    with pytest.raises(ValueError, match="already exist"):
        deploy.install_directory(unpacked, destination)
    assert before == (destination / "bundle_manifest.json").read_bytes()


def test_local_test_fixtures_inside_functions_are_packaged_recursively(tmp_path):
    root, firmware, wheels = fixture_bundle(tmp_path)
    write(root / "tests/test_entry.py", "def test_it():\n    from test_helper import value\n")
    write(root / "tests/test_helper.py", "from test_leaf import value\n")
    write(root / "tests/test_leaf.py", "value = 1\n")
    result = prepare.build_bundle(root, firmware, wheels, tmp_path / "output",
                                  ["tests/test_entry.py"])
    manifest = deploy.verify_directory(Path(result["archive"]).parent / "bundle")
    assert {"tests/test_entry.py", "tests/test_helper.py", "tests/test_leaf.py"} <= set(manifest["files"])


@pytest.mark.parametrize("mode", ["changed", "missing", "extra"])
def test_bundle_rejects_modified_inventory(tmp_path, mode):
    root, firmware, wheels = fixture_bundle(tmp_path)
    prepare.build_bundle(root, firmware, wheels, tmp_path / "output")
    bundle = tmp_path / "output/bundle"
    target = bundle / "scripts/uno_q_host.py"
    if mode == "changed":
        target.write_bytes(b"x" * target.stat().st_size)
    elif mode == "missing":
        target.unlink()
    else:
        write(bundle / "extra.py", "unexpected")
    with pytest.raises(ValueError):
        deploy.verify_directory(bundle)


@pytest.mark.parametrize("name", ["../escape", "/tmp/escape", "C:/escape", "folder\\escape", "folder/../escape", "./escape"])
def test_archive_rejects_unsafe_paths_before_extract(tmp_path, name):
    archive = tmp_path / "bad.zip"
    with zipfile.ZipFile(archive, "w") as package:
        package.writestr("valid.txt", "must not extract")
        package.writestr(name, "bad")
    target = tmp_path / "out"
    target.mkdir()
    with pytest.raises(ValueError):
        deploy.extract_archive(archive, target)
    assert list(target.iterdir()) == []


def test_archive_rejects_symlink(tmp_path):
    archive = tmp_path / "bad.zip"
    with zipfile.ZipFile(archive, "w") as package:
        info = zipfile.ZipInfo("link")
        info.external_attr = (stat.S_IFLNK | 0o777) << 16
        package.writestr(info, "../elsewhere")
    with pytest.raises(ValueError, match="non-regular"):
        deploy.extract_archive(archive, tmp_path / "out")


def test_changed_source_after_firmware_build_is_rejected(tmp_path):
    root, firmware, wheels = fixture_bundle(tmp_path)
    write(root / "src/arena_vehicle_interface/firmware_uno_q/src/main.cpp", "new source")
    with pytest.raises(ValueError, match="source has changed"):
        prepare.build_bundle(root, firmware, wheels, tmp_path / "output")
    assert not (tmp_path / "output").exists()


def test_changed_binary_after_manifest_is_rejected(tmp_path):
    root, firmware, wheels = fixture_bundle(tmp_path)
    write(firmware / "zephyr.bin", "different binary")
    with pytest.raises(ValueError, match="artifact hash"):
        prepare.build_bundle(root, firmware, wheels, tmp_path / "output")


def test_multiple_firmware_profiles_kept_distinct(tmp_path):
    root, firmware, wheels = fixture_bundle(tmp_path)
    import shutil
    bench = tmp_path / "bench"
    shutil.copytree(firmware, bench)
    write(bench / "zephyr.bin", "different explicit bench image")
    manifest = json.loads((bench / "firmware_manifest.json").read_text())
    manifest["artifacts"]["zephyr.bin"] = deploy.digest(bench / "zephyr.bin")
    write(bench / "firmware_manifest.json", json.dumps(manifest))
    prepare.build_bundle(root, firmware, wheels, tmp_path / "output", bench_firmware_dir=bench)
    release = tmp_path / "output/bundle"
    assert (release / "firmware/unconfigured/zephyr.bin").read_bytes() != (release / "firmware/bench/zephyr.bin").read_bytes()
    assert deploy.verify_directory(release)["firmware_profiles"] == ["unconfigured", "bench"]


def test_offline_dependency_required(tmp_path):
    root, firmware, wheels = fixture_bundle(tmp_path)
    next(wheels.iterdir()).unlink()
    with pytest.raises(ValueError, match="offline pyserial"):
        prepare.build_bundle(root, firmware, wheels, tmp_path / "output")


def test_readonly_preflight_never_opens_serial_or_changes_power(monkeypatch):
    calls = []
    monkeypatch.setattr(preflight, "query", lambda command: calls.append(command) or {"fixture": True})
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    report = preflight.inspect("/nonexistent/uart", "/nonexistent/servo", power_status=True)
    assert report["hardware_ports_opened"] is False
    assert report["outputs_enabled"] is False
    assert report["pinmux_changed"] is False
    assert report["pinmux_verified"] is False
    assert calls == [["nvpmodel", "-q", "--verbose"], ["jetson_clocks", "--show"]]
