"""Windows에서 r553 레시피만 복사한다. 빌드/동결/ROS 실행 권한은 만들지 않는다."""
import ast
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = A.parent / "measured_steering_radial_lap_r553_v1"
CHANGES = {
    "src/arena_autonomy/arena_autonomy/bypass_entry_trial.py",
    "src/arena_autonomy/arena_autonomy/current_wall_reacquire.py",
    "src/arena_autonomy/arena_autonomy/current_path_recheck/_geometry.py",
    "src/arena_autonomy/arena_autonomy/race_runtime_policy/retained_recheck/_geometry.py",
}


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    if path.exists():
        assert read(path) == value, path
        return
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def main():
    assert not any((A / name).exists() for name in ("sources.zip", "prepared.json", "registered.json", "run1"))
    preserved_names = (
        "prepared.json", "sources.zip", "run1/report.json", "integrity.json",
        "execution_v1/owned_processes_after.json", "RESULT.md",
        "stall_extract/first_zero_publication_call_call.json",
    )
    preserved = {name: sha(BASE / name) for name in preserved_names}
    old = read(BASE / "prepared.json")
    assert preserved["sources.zip"] == old["source_archive_sha256"]
    assert read(BASE / "run1/report.json")["shutdown_clean"]
    assert not read(BASE / "execution_v1/owned_processes_after.json")["remaining_owned_work"]
    previous = read(BASE / "source_selection.json")
    selected = {path: sha(ROOT / path) for path in previous["required_sources"]}
    delta = [dict(path=path, before_sha256=before, after_sha256=selected[path])
             for path, before in previous["required_sources"].items() if before != selected[path]]
    assert {row["path"] for row in delta} == CHANGES, delta

    # 실행 산출물/동결 archive는 복제하지 않는다. 기존 평가/STOP 구현을 그대로 쓴다.
    names = [row["path"] for row in read(BASE / "recipe_provenance.json")["files"]]
    names += ["arguments.json"]
    copied = []
    prefix = BASE.relative_to(ROOT).as_posix() + "/"
    for name in names:
        source, dest = BASE / name, A / name
        before = sha(source)
        assert before == old["frozen_inputs"][prefix + name], name
        data = source.read_bytes()
        connection_change = False
        if dest.suffix in (".py", ".json", ".sh", ".bash"):
            text = data.decode("utf-8").replace(BASE.name, A.name)
            if name == "prepare.py":
                old_base = "BASE=A.parent/'measured_steering_reacquire_lap_r550_v1'"
                assert old_base in text
                text = text.replace(old_base, "BASE=A.parent/'" + BASE.name + "'")
                connection_change = True
            if name == "preflight.py":
                needle = "'bypass_wall_fit','current_wall_reacquire'"
                assert needle in text
                text = text.replace(needle, "'bypass_wall_fit','bypass_entry_trial','current_wall_reacquire'")
                connection_change = True
            if name.startswith("execution_v1/") and dest.suffix in (".sh", ".bash"):
                text = text.replace("r553 ", "r558 ")
            data = text.encode("utf-8")
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest.exists():
            assert dest.read_bytes() == data, name
        else:
            with dest.open("xb") as stream:
                stream.write(data)
        copied.append(dict(path=name, baseline_sha256=before, sha256=sha(dest),
                           path_relocated=True, source_connection_change=connection_change))

    write(A / "source_selection.json", dict(
        freeze_authorized=False, wsl_exclusive_owner=False, drive_authorized=False,
        final_production_source_selection_pending=True, native_rebuild_authorized=False,
        source_baseline=BASE.relative_to(ROOT).as_posix(), required_sources=selected,
        source_delta=delta, status_detail="full",
        status="WINDOWS_RECIPE_ONLY_WAIT_ROOT_GO_AFTER_R560",
        source_hash_scope="preparation-time proposal; root must confirm selection and exclusive ownership after r560",
        validation_references=[
            "artifacts/validation/2026-11-14/r557_current_radial_stall_v1/INTEGRATION.md",
            "artifacts/validation/2026-11-14/r557_current_radial_stall_v1/integrated_manifest.json",
            "artifacts/validation/2026-11-14/ray_order_followup_r554_v1/common_transfer",
        ],
    ))
    write(A / "observational_baseline.json", read(BASE / "observational_baseline.json"))
    write(A / "recipe_provenance.json", dict(
        baseline=BASE.relative_to(ROOT).as_posix(), preserved_evidence=preserved, files=copied,
        world_sha256=sha(A / "field_world.sdf"), expected_header_sha256=sha(A / "expected_header.json"),
        seven_angle_audit_reused=True, original_acceptance_speed_stop_unchanged=True,
        copied_geometry_receipts_scope="inherited r553 static evidence only; no new geometry or dynamic verification",
    ))
    commands = json.loads((BASE / "commands.json").read_text(encoding="utf-8").replace(BASE.name, A.name))
    commands.update(status="RECIPE_READY_WAIT_ROOT_GO_AFTER_R560", baseline_artifact=BASE.relative_to(ROOT).as_posix(),
                    source_selection="source_selection.json: four changed sources proposed; all authorizations false",
                    planned_scripts_exist=True, source_archive_created=False, frozen=False,
                    after_execution="r560_sparse_actor_drive_v1: owned shutdown/PID0 and explicit WSL handover first")
    commands["steps"][1] = dict(stage="root_select_after_go", operation=(
        "Verify preparation-time source hashes and r557/r554 adoption after r560; root records explicit source/build/freeze/drive "
        "authorization and WSL ownership. After freeze, create fresh root_execution_review.json bound to this prepared.json/archive; never reuse r553 receipt."))
    write(A / "commands.json", commands)
    expected_args = [value.replace(BASE.name, A.name) for value in read(BASE / "arguments.json")]
    assert read(A / "arguments.json") == expected_args
    config = read(A / "config.json")
    assert config == read(A / "expected_config.json")
    assert json.loads(json.dumps(config).replace(A.name, BASE.name)) == read(BASE / "config.json")
    assert config["launch"]["grid_slot"] == 3
    unchanged = {name: sha(A / name) == sha(BASE / name) for name in
                 ("field_world.sdf", "facilities_only.sdf", "expected_header.json", "solo_evaluation.py", "solo_finalize.py",
                  "audit_runtime.py", "steering_runtime_audit.py")}
    assert all(unchanged.values()), unchanged
    assert (A / "stop_owned.py").read_text(encoding="utf-8").replace(A.name, BASE.name) == (BASE / "stop_owned.py").read_text(encoding="utf-8")
    compiled = []
    for path in A.rglob("*.py"):
        compile(path.read_bytes(), str(path), "exec")
        compiled.append(path.relative_to(A).as_posix())
    assert all(sha(BASE / name) == before for name, before in preserved.items())
    assert not any((A / name).exists() for name in ("prepared.json", "sources.zip", "registered.json", "run1", "build_result.json", "root_execution_review.json"))
    write(A / "preparation_check.json", dict(
        passed=True, scope="Windows file/JSON/hash/syntax checks only", actual_runs=0, WSL_used=False,
        build_executed=False, source_archive_created=False, root_go_received=False,
        actor_count=0, grid_slot=3, facilities=3, max_sim_seconds=80, no_progress_stop_sim_seconds=8,
        original_config_equal_after_path_relocation=True, original_arguments_equal_after_path_relocation=True,
        unchanged_bytewise=unchanged, source_delta=delta, required_source_count=len(selected),
        stop_owned_equal_after_path_relocation=True,
        original_r553_evidence_unchanged=True, recipe_files=len(copied), python_compiled=compiled,
        world_sha256=sha(A / "field_world.sdf"),
        measured_vehicle_sha256=selected["src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml"],
        central_steering_limit_rad=0.28475436568260193,
    ))
    print(json.dumps(dict(recipe_ready=True, required_sources=len(selected), source_delta_files=len(delta),
                         copied_files=len(copied), syntax_checked=len(compiled), actual_runs=0)))


if __name__ == "__main__":
    main()
