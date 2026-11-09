"""r52 후속 출처 계약: HEAD(815932fe) 바이트를 before.zip에 보존하고 현재 바이트를 after로 등록한다.

Windows/WSL python 모두 가능(git 필요). 이미 있는 계약/스냅샷은 덮어쓰지 않는다.
"""
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = "config/tests/p22_exit_alignment_r51_sha256.json"
CONTRACT = "config/tests/signal_posts_markers_r52_sha256.json"
REASON = {
    "src/arena_bringup/launch/simulation.launch.py":
        "Default-none v11.06 runtime-copy signal posts (STL sections, 610 mm spacing) and blank_20_30 evaluation marker mode",
    "src/arena_bringup/launch/demo.launch.py": "Accept the blank_20_30 evaluation marker mode",
    "src/arena_bringup/launch/g1_follow.launch.py": "Default-off narrow occluder bridge and junction continuity parameters",
    "src/arena_autonomy/arena_autonomy/local_follow.py":
        "Default-off narrow occluder bridge passthrough and junction continuity (opposite wall / recently expired path agreement)",
    "src/arena_autonomy/arena_autonomy/local_path.py": "Default-off narrow occluder bridge in wall chain extraction",
    "src/arena_autonomy/arena_autonomy/local_pursuit.py": "Declare the two default-off candidate parameters",
}
EVALUATION = {
    "scripts/validate_basic_autonomy.py":
        "Signal post obstacles/clearance evidence, blank_20_30 route evidence and candidate flags",
    "scripts/run_new_course_solo.py": "Optional signal post, marker blank and candidate flags appended to the r25 preset",
}
sha = lambda data: hashlib.sha256(data).hexdigest()


def main():
    contract, snapshot = ROOT / CONTRACT, HERE / "before.zip"
    assert not contract.exists() and not snapshot.exists()
    files = {**REASON, **EVALUATION}
    head = {f: subprocess.check_output(["git", "show", "HEAD:" + f], cwd=ROOT) for f in files}
    with zipfile.ZipFile(snapshot, "x", zipfile.ZIP_DEFLATED) as archive:
        for f, data in head.items():
            archive.writestr(f, data)
    row = lambda f, reasons: dict(before=sha(head[f]), after=sha((ROOT / f).read_bytes()), reason=reasons[f])
    data = dict(
        schema_version=1,
        scope="11/9 signal posts (runtime facility, default none) and junction marker loss candidates (default OFF)",
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT / PARENT).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes={f: row(f, REASON) for f in REASON},
        approved_evaluation_scripts={f: row(f, EVALUATION) for f in EVALUATION},
        new_dependencies={})
    assert all(v["before"] != v["after"] for group in ("approved_changes", "approved_evaluation_scripts")
               for v in data[group].values())
    contract.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({k: (len(v) if isinstance(v, dict) else v) for k, v in data.items()}, ensure_ascii=False))


if __name__ == "__main__":
    main()
