#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_level_surface_v1
test ! -e "$base/run3"
test ! -e "$base/run3_trial.log"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run3_preexisting_processes.txt" || true
if [ -s "$base/run3_preexisting_processes.txt" ]; then echo 'preexisting simulation processes; not starting' > "$base/run3_ABORTED.txt"; exit 92; fi
python3 -B -m pytest -q -p no:cacheprovider "$base/test_level_surface.py" > "$base/run3_tests.log" 2>&1
python3 -B "$base/run.py" --output "$base/run3" --prepare-only > "$base/run3_prepare.log" 2>&1
python3 -B "$base/preflight.py" run3 > "$base/run3_preflight.log" 2>&1
set +e
python3 -B "$base/run.py" --output "$base/run3" > "$base/run3_trial.log" 2>&1
trial_exit=$?
if [ ! -f "$base/run3/report.json" ] || [ ! -f "$base/run3/follow_decisions.jsonl" ]; then
    printf 'setup_or_capture_failure=%s\n' "$trial_exit" > "$base/run3_exit.txt"
    ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run3_residual_processes.txt" || true
    exit 1
fi
python3 -B scripts/replay_follow_trace.py "$base/run3/follow_decisions.jsonl" --output "$base/run3/follow_trace_replay.json" > "$base/run3_replay.log" 2>&1
replay_exit=$?
python3 -B "$base/compare.py" --run "$base/run3" --output "$base/run3_comparison.json" > "$base/run3_evaluation.log" 2>&1
evaluation_exit=$?
python3 -B artifacts/validation/2026-10-28/bump_traffic_v1/analyze.py "$base/run3" --output "$base/run3_bump_analysis.json" > "$base/run3_bump_analysis.log" 2>&1
analysis_exit=$?
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run3_residual_processes.txt" || true
printf 'original_g1_trial=%s\nreplay=%s\nhandoff_evaluation=%s\nbump_analysis=%s\n' "$trial_exit" "$replay_exit" "$evaluation_exit" "$analysis_exit" > "$base/run3_exit.txt"
exit "$evaluation_exit"
