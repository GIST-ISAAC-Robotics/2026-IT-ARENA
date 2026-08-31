"""감사 사본의 최소 변경·원본 보존·병합 전후 형상 비교 검사."""

import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from audit_official_world_load import (  # noqa: E402
    ZIP, ZIP_SHA256, flattened_geometry_signature, geometry_signature, prepare, sanitize_evidence_text, sha256,
)


@pytest.fixture(scope="module")
def fixtures(tmp_path_factory):
    base = tmp_path_factory.mktemp("raw_audit") / "new_run"
    return base, prepare(base)


def xml(fixtures, case):
    base, report = fixtures
    return (base / report["cases"][case]["world"]).read_text(encoding="utf-8")


def test_raw_zip_bytes_and_all_files_are_preserved(fixtures):
    _, report = fixtures
    assert sha256(ZIP.read_bytes()) == ZIP_SHA256
    assert report["file_count"] == 23
    assert report["cases"]["raw"]["sha256"] == report["source_world_sha256"]
    assert all(case["other_release_files_unchanged"] for case in report["cases"].values())


def test_path_change_does_not_hide_required_name_failure(fixtures):
    root = ET.fromstring(xml(fixtures, "paths_only"))
    scripts = root.findall(".//material/script")
    assert len(scripts) == 4
    assert all(script.find("name") is None for script in scripts)
    assert "../aruco/" not in xml(fixtures, "paths_only")


def test_script_removal_does_not_fix_the_pbr_path(fixtures):
    root = ET.fromstring(xml(fixtures, "remove_script"))
    assert not root.findall(".//script")
    assert all(element.text.startswith("../aruco/") for element in root.findall(".//albedo_map"))


def test_material_controls_keep_every_non_material_field(fixtures):
    original = geometry_signature(xml(fixtures, "raw"))
    for case in ("paths_only", "remove_script", "script_and_paths"):
        assert geometry_signature(xml(fixtures, case)) == original


def test_link_merge_changes_no_geometry_or_contact_surface(fixtures):
    original = flattened_geometry_signature(xml(fixtures, "raw"))
    assert flattened_geometry_signature(xml(fixtures, "merge_static")) == original
    _, report = fixtures
    assert report["cases"]["raw"]["links"] == 1113
    assert report["cases"]["merge_static"]["links"] == 10
    assert all(case["collisions"] == 1107 and case["visuals"] == 1113 and case["explicit_plugins"] == 0
               for case in report["cases"].values())


def test_preparation_will_not_overwrite_an_existing_run(fixtures):
    base, _ = fixtures
    before = (base / "fixtures.json").read_bytes()
    with pytest.raises(FileExistsError):
        prepare(base)
    assert (base / "fixtures.json").read_bytes() == before


def test_prepared_report_is_machine_readable(fixtures):
    base, report = fixtures
    assert json.loads((base / "fixtures.json").read_text(encoding="utf-8")) == report
    assert {ref["exists_relative_to_sdf"] for ref in report["cases"]["script_and_paths"]["albedo_references"]} == {True}


def test_log_sanitizing_keeps_truncated_escape_visible_and_hides_personal_paths():
    text = "\x1b[1;32m" + str(REPO) + "\x1b[0m\n" + str(Path.home()) + "\b\x1b[1;32"
    sanitized = sanitize_evidence_text(text)
    assert sanitized == "<WORKSPACE>\n<WSL_HOME><0x08><ESC>[1;32"
    assert "\x1b" not in sanitized


def test_sequential_fix_keeps_geometry_and_changes_one_cause_at_a_time(tmp_path):
    base = tmp_path / "sequence"
    report = prepare(base, sequential=True)
    assert list(report["cases"]) == ["raw", "remove_script", "merge_only", "merge_static", "ready"]
    roots = {key: ET.parse(base / case["world"]).getroot() for key, case in report["cases"].items()}
    assert len(roots["remove_script"].findall(".//link")) == 1113
    assert len(roots["merge_only"].findall(".//link")) == 10
    assert all(tag.text.startswith("../aruco/") for tag in roots["merge_only"].findall(".//albedo_map"))
    assert all(tag.text.startswith("aruco/") for tag in roots["merge_static"].findall(".//albedo_map"))
    assert not roots["merge_static"].findall(".//material[pbr]/diffuse")
    assert len(roots["ready"].findall(".//material[pbr]/diffuse")) == 4
    assert all(tag.text == "1 1 1 1" for tag in roots["ready"].findall(".//material[pbr]/diffuse"))
    original = (base / report["cases"]["raw"]["world"]).read_text()
    for case in report["cases"].values():
        assert flattened_geometry_signature((base / case["world"]).read_text()) == flattened_geometry_signature(original)
        assert case["collisions"] == 1107 and case["visuals"] == 1113
