#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_stress_v1
test ! -e "$base/run2_course"
test ! -e "$base/run2_trial.log"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run2_preexisting_processes.txt" || true
if [ -s "$base/run2_preexisting_processes.txt" ]; then echo 'preexisting simulation processes; not starting' > "$base/run2_ABORTED.txt"; exit 92; fi
/usr/bin/python3 -B "$base/run.py" --output "$base/run2_course" --prepare-only > "$base/run2_prepare.log" 2>&1
set +e
/usr/bin/python3 -B "$base/run.py" --output "$base/run2_course" > "$base/run2_trial.log" 2>&1
trial_exit=$?
/usr/bin/python3 -B artifacts/validation/2026-10-27/partial_observation_v1/audit_wheel_centers.py "$base/run2_course" --output "$base/run2_wheel_centers.json" > "$base/run2_wheel_centers.log" 2>&1
wheel_exit=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py "$base/run2_course/follow_decisions.jsonl" --output "$base/run2_course/follow_trace_replay.json" > "$base/run2_replay.log" 2>&1
replay_exit=$?
/usr/bin/python3 -B "$base/summarize_run.py" run2_course > "$base/run2_summary.log" 2>&1
summary_exit=$?
sleep 3
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run2_residual_processes.txt" || true
printf 'trial=%s\nwheel=%s\nreplay=%s\nsummary=%s\n' "$trial_exit" "$wheel_exit" "$replay_exit" "$summary_exit" > "$base/run2_exit.txt"
exit "$trial_exit"
