"""Compare the frozen ESP32 final source ZIP with the source ZIPs of earlier simulation runs.

Read-only: opens ZIPs, hashes members in memory, writes one JSON summary.
Usage: python compare_sources.py <repo_root> <out_json>
"""
import hashlib
import json
import sys
import zipfile
from pathlib import Path

FINAL = "artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/selected_v2/runtime_handover_sources.zip"
FINAL_SHA = "92aa7039b188cf16ee31c3ecabe941793837ca9a157b9de9533198807feff9aa"
RUNS = {
    "r475_solo_full_lap": "artifacts/validation/2026-11-14/solo_full_lap_r475_v1/sources.zip",
    "r492_solo_guard_recovery": "artifacts/validation/2026-11-14/solo_guard_recovery_r492_v1/execution_sources.zip",
    "r496_traffic_current_geometry": "artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1/sources.zip",
    "r539_field_wall_guard_relevance": "artifacts/validation/2026-11-14/field_wall_guard_relevance_drive_r539_v1/sources.zip",
    "r544_attempt2_measured_steering": "artifacts/validation/2026-11-14/measured_steering_lap_r544_attempt2_v1/sources.zip",
    "r550_reacquire": "artifacts/validation/2026-11-14/measured_steering_reacquire_lap_r550_v1/sources.zip",
    "r553_radial": "artifacts/validation/2026-11-14/measured_steering_radial_lap_r553_v1/sources.zip",
    "r560_sparse_actor": "artifacts/validation/2026-11-14/r560_sparse_actor_drive_v1/sources.zip",
    # 기록 영상 원 실행(부분 스냅샷일 수 있음: 공통 경로만 비교)
    "r15_curved_course_registered_untracked": "artifacts/validation/2026-11-05/solo_straight_closed_loop_v1/registered_untracked_sources.zip",
    "r25_right_angle_snapshot": "artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/source_snapshot.zip",
    "r40_stop_restart_snapshot": "artifacts/validation/2026-11-08/entry_width_transition_v1/source_snapshot.zip",
    "r15_curved_course_run_snapshot": "artifacts/validation/2026-11-05/solo_straight_closed_loop_v1/run1/source_snapshot.zip",
    "r13_overtake_snapshot": "artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/source_snapshot.zip",
    "r18_bypass_snapshot": "artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/source_snapshot.zip",
    "r15m_medium_pass_snapshot": "artifacts/validation/2026-11-08/medium_pass_connected_v1/straight_moving_lead/source_snapshot.zip",
    "r38_sensor_noise_delay_snapshot": "artifacts/validation/2026-11-08/masked_motion_speed_v1/source_snapshot.zip",
}
# Paths whose content decides runtime driving behaviour in simulation.
RUNTIME_PREFIXES = ("src/", "config/")


def sha(b):
    return hashlib.sha256(b).hexdigest()


def members(path):
    out = {}
    with zipfile.ZipFile(path) as z:
        for info in z.infolist():
            if info.is_dir():
                continue
            out[info.filename] = sha(z.read(info))
    return out


def main():
    root = Path(sys.argv[1])
    out_path = Path(sys.argv[2])
    final_bytes = (root / FINAL).read_bytes()
    final_sha = sha(final_bytes)
    final = members(root / FINAL)
    result = {
        "final_zip": FINAL,
        "final_zip_sha256": final_sha,
        "final_zip_sha256_matches_release": final_sha == FINAL_SHA,
        "final_member_count": len(final),
        "runs": {},
    }
    final_rt = {k: v for k, v in final.items() if k.startswith(RUNTIME_PREFIXES)}
    for name, rel in RUNS.items():
        p = root / rel
        if not p.exists():
            result["runs"][name] = {"zip": rel, "missing": True}
            continue
        run = members(p)
        run_rt = {k: v for k, v in run.items() if k.startswith(RUNTIME_PREFIXES)}
        common = sorted(set(final_rt) & set(run_rt))
        same = [k for k in common if final_rt[k] == run_rt[k]]
        diff = [k for k in common if final_rt[k] != run_rt[k]]
        result["runs"][name] = {
            "zip": rel,
            "zip_sha256": sha(p.read_bytes()),
            "member_count": len(run),
            "runtime_paths_final": len(final_rt),
            "runtime_paths_run": len(run_rt),
            "common_runtime_paths": len(common),
            "identical": len(same),
            "different": len(diff),
            "different_paths": diff,
            "only_in_final": sorted(set(final_rt) - set(run_rt)),
            "only_in_run_count": len(set(run_rt) - set(final_rt)),
        }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    for name, r in result["runs"].items():
        if r.get("missing"):
            print(name, "MISSING")
            continue
        print(f"{name}: common={r['common_runtime_paths']} same={r['identical']} diff={r['different']} "
              f"only_final={len(r['only_in_final'])} only_run={r['only_in_run_count']}")
    print("final sha ok:", result["final_zip_sha256_matches_release"])


if __name__ == "__main__":
    main()
