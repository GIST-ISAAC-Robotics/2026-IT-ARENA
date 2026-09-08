from pathlib import Path
import hashlib, json, re, sys
root = Path(__file__).resolve().parent
repo = root.parents[3]
sys.path.insert(0, str(repo / "scripts"))
from lidar_shutdown import audit_shutdown
summary = {"native": {}, "ros": {}, "current_source_sha256": {}}
for case in ("static_full", "physics_only", "no_physics", "empty_physics", "detector_ode", "detector_bullet", "detector_fcl", "detector_dart"):
    content = (root / f"{case}.log").read_text(errors="replace")
    match = re.search(r"PROFILE (\{.*\})", content)
    summary["native"][case] = json.loads(match.group(1)) if match else {"completed": False}
for case in ("baseline", "bullet", "bullet_no_scene", "gpu_bullet_no_scene", "gpu_grace", "gpu_grace_repeat"):
    reports = list((root / case).glob("*.json"))
    if not reports: continue
    report = json.loads(reports[0].read_text())
    log = reports[0].with_suffix(".log").read_text()
    summary["ros"][case] = {"report": report, "shutdown_audit": audit_shutdown(log, report["launch_exit_code"])}
for name in ("scripts/smoke_simulation.py", "src/arena_bringup/launch/simulation.launch.py", "src/arena_bringup/launch/demo.launch.py"):
    summary["current_source_sha256"][name] = hashlib.sha256((repo / name).read_bytes()).hexdigest()
(root / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
for case, row in summary["ros"].items():
    report = row["report"]
    print(case, report.get("camera_validation", {}).get("rgb", {}).get("real_time_factor"), report["passed"], row["shutdown_audit"]["clean"])
