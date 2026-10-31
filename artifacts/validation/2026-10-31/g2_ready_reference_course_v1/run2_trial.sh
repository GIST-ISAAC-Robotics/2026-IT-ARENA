#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_ready_reference_course_v1
test ! -e "$base/run2"
test ! -e "$base/run2_trial.log"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run2_preexisting_processes.txt" || true
if [ -s "$base/run2_preexisting_processes.txt" ]; then echo 'preexisting simulation processes; not starting' > "$base/run2_ABORTED.txt"; exit 92; fi
python3 -B -m pytest -q -p no:cacheprovider "$base/test_evaluate.py" > "$base/run2_tests.log" 2>&1
python3 -B "$base/run.py" --output "$base/run2" --prepare-only > "$base/run2_prepare.log" 2>&1
python3 -B "$base/preflight.py" run2 > "$base/run2_preflight.log" 2>&1
set +e
python3 -B "$base/run.py" --output "$base/run2" > "$base/run2_trial.log" 2>&1
trial_exit=$?
if [ ! -f "$base/run2/report.json" ] || [ ! -f "$base/run2/follow_decisions.jsonl" ]; then
    printf 'setup_or_capture_failure=%s\n' "$trial_exit" > "$base/run2_exit.txt"
    ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run2_residual_processes.txt" || true
    exit 1
fi
python3 -B scripts/replay_follow_trace.py "$base/run2/follow_decisions.jsonl" --output "$base/run2/follow_trace_replay.json" > "$base/run2_replay.log" 2>&1
replay_exit=$?
python3 -B "$base/evaluate.py" "$base/run2" --output "$base/run2_evaluation.json" > "$base/run2_evaluation.log" 2>&1
evaluation_exit=$?
python3 -B artifacts/validation/2026-10-28/bump_traffic_v1/analyze.py "$base/run2" --output "$base/run2_bump_analysis.json" > "$base/run2_bump_analysis.log" 2>&1
analysis_exit=$?
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run2_residual_processes.txt" || true
printf 'original_g1_trial=%s\nreplay=%s\nhandoff_evaluation=%s\nbump_analysis=%s\n' "$trial_exit" "$replay_exit" "$evaluation_exit" "$analysis_exit" > "$base/run2_exit.txt"
exit "$evaluation_exit"
