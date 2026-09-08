"""CPU/GPU의 같은 짧은 실행 결과와 자식 프로세스 종료를 재감사합니다."""
from pathlib import Path
import hashlib
import json
import re
import sys

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[3]
sys.path.insert(0, str(REPO / "scripts"))
from lidar_shutdown import audit_shutdown

runs = {}
for case in ("cpu_01", "gpu_01", "gpu_final_01", "gpu_final_02"):
    if not (ROOT / case / "official_low_load_30_smoke.json").exists():
        continue
    report = json.loads((ROOT / case / "official_low_load_30_smoke.json").read_text())
    log = (ROOT / case / "official_low_load_30_smoke.log").read_text()
    renderer_log = (ROOT / f"{case}_ogre2.log").read_text()
    shutdown = audit_shutdown(log, report["launch_exit_code"])
    runs[case] = {"passed": report["passed"], "elapsed_wall_s": report["elapsed_wall_s"],
                  "renderer": re.search(r"GL_RENDERER = (.+)", renderer_log).group(1),
                  "camera": report["camera_validation"], "shutdown": shutdown,
                  "vehicle_config_sha256": report["vehicle_config_sha256"],
                  "depth_valid_range_observed_m": report.get("depth_valid_range_observed_m"),
                  "straight_distance_m": report.get("straight_distance_m"),
                  "turn_yaw_change_rad": report.get("turn_yaw_change_rad")}
summary = {"runs": runs,
           "rgb_rtf_ratio": runs["gpu_01"]["camera"]["rgb"]["real_time_factor"] / runs["cpu_01"]["camera"]["rgb"]["real_time_factor"],
           "same_vehicle_config": runs["cpu_01"]["vehicle_config_sha256"] == runs["gpu_01"]["vehicle_config_sha256"],
           "current_source_sha256": {name: hashlib.sha256((REPO / name).read_bytes()).hexdigest() for name in
                             ("scripts/smoke_simulation.py", "src/arena_bringup/launch/simulation.launch.py",
                              "src/arena_bringup/launch/demo.launch.py", "src/arena_gazebo/src/wsl_d3d12_lifetime.cpp")}}
(ROOT / "comparison.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
print(json.dumps({"rtf_ratio": summary["rgb_rtf_ratio"], "runs": {
    name: {key: row[key] for key in ("passed", "elapsed_wall_s", "renderer", "shutdown")} for name, row in runs.items()}}, indent=2))
