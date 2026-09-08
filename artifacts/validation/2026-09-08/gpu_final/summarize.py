from pathlib import Path
import hashlib, json, re, statistics

root = Path(__file__).resolve().parent
repo = root.parents[3]
rows = json.loads((root / "results.json").read_text())
micro = {}
for mode in ("rgbd", "lidar500", "tof6"):
    means = {}
    for backend in ("cpu", "gpu"):
        matches = [row for row in rows if re.fullmatch(f"{backend}_{mode}_[12]", row["name"])]
        assert len(matches) == 2 and all(row["clean_exit"] and row["profile"]["valid"] for row in matches)
        assert all(("llvmpipe" if backend == "cpu" else "NVIDIA") in row["renderer"] for row in matches)
        means[backend] = statistics.mean(row["profile"]["wall_s"] for row in matches)
    micro[mode] = {**means, "time_reduction_percent": 100*(1-means["gpu"]/means["cpu"])}
trials = []
for name in ("sequential_software", "sequential_wsl_nvidia"):
    folder = root / name
    report = json.loads((folder / "report.json").read_text())
    trace = [json.loads(line) for line in (folder / "trace.jsonl").read_text().splitlines()]
    acq = json.loads((folder / "acquisition.json").read_text())
    ogre = (folder / "ogre2.log").read_text(errors="replace")
    trials.append({"name": name, "passed": report["passed"],
                   "elapsed_wall_s": report["elapsed_wall_s"],
                   "trace_sim_span_s": trace[-1]["sim_time_s"]-trace[0]["sim_time_s"],
                   "trace_final_elapsed_s": trace[-1]["elapsed_s"],
                   "renderer": re.findall(r"GL_RENDERER\s*=\s*(.+)", ogre)[-1],
                   "acquisition_verified": report["acquisition_verified"],
                   "max_capture_time_error_s": max(row["max_capture_time_error_s"] for row in acq),
                   "metrics": report["metrics"], "shutdown_audit": report["shutdown_audit"]})
sources = [root/"sensor_profile.cpp", root/"run.py", root/"run_sequential.sh",
           repo/"scripts/validate_lidar_control_lab.py", repo/"src/arena_bringup/launch/simulation.launch.py",
           repo/"src/arena_description/models/arena_car/model.sdf.xacro",
           repo/"src/arena_gazebo/worlds/it_arena_official/world.sdf"]
result = {"microbench": micro, "sequential_trials": trials,
          "input_sha256": {str(path.relative_to(repo)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}}
(root/"summary.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
