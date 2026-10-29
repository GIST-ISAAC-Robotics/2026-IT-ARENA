#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_stress_v1
test ! -e "$base/run3_course"
test ! -e "$base/run3_trial.log"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run3_preexisting_processes.txt" || true
if [ -s "$base/run3_preexisting_processes.txt" ]; then echo 'preexisting simulation processes; not starting' > "$base/run3_ABORTED.txt"; exit 92; fi
/usr/bin/python3 -B "$base/run.py" --output "$base/run3_course" --prepare-only > "$base/run3_prepare.log" 2>&1
set +e
/usr/bin/python3 -B "$base/run.py" --output "$base/run3_course" > "$base/run3_trial.log" 2>&1
trial_exit=$?
/usr/bin/python3 -B artifacts/validation/2026-10-27/partial_observation_v1/audit_wheel_centers.py "$base/run3_course" --output "$base/run3_wheel_centers.json" > "$base/run3_wheel_centers.log" 2>&1
wheel_exit=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py "$base/run3_course/follow_decisions.jsonl" --output "$base/run3_course/follow_trace_replay.json" > "$base/run3_replay.log" 2>&1
replay_exit=$?
/usr/bin/python3 -B "$base/summarize_run.py" run3_course > "$base/run3_summary.log" 2>&1
summary_exit=$?
sleep 3
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run3_residual_processes.txt" || true
printf 'trial=%s\nwheel=%s\nreplay=%s\nsummary=%s\n' "$trial_exit" "$wheel_exit" "$replay_exit" "$summary_exit" > "$base/run3_exit.txt"
exit "$trial_exit"
