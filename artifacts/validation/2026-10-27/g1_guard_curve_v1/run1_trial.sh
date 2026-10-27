#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_guard_curve_v1
test ! -e "$base/run1_corner"
test ! -e "$base/run1_trial.log"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run1_preexisting_processes.txt" || true
if [ -s "$base/run1_preexisting_processes.txt" ]; then echo "preexisting simulation processes; not starting" > "$base/run1_ABORTED.txt"; exit 92; fi
/usr/bin/python3 -B "$base/run1_snapshot.py" --check > "$base/run1_pre_drive_source.log" 2>&1
set +e
/usr/bin/python3 -B scripts/validate_g1_traffic.py --config config/tests/g1_corner_after_threat_v1.json --case stop_restart_first_curve --seed 0 --mode g1 --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --partial-observation-policy bounded_local_v4 --parallel-hold-trial --pass-release-trial --guard-curve-trial --actor-overlay config/tests/e3_stop_restart_corner_v1.json --output "$base/run1_corner" > "$base/run1_trial.log" 2>&1
trial_exit=$?
/usr/bin/python3 -B "$base/run1_snapshot.py" --check > "$base/run1_source_check.log" 2>&1
source_exit=$?
/usr/bin/python3 -B artifacts/validation/2026-10-27/partial_observation_v1/audit_wheel_centers.py "$base/run1_corner" --output "$base/run1_wheel_centers.json" > "$base/run1_wheel_centers.log" 2>&1
wheel_exit=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py "$base/run1_corner/follow_decisions.jsonl" --output "$base/run1_corner/follow_trace_replay.json" > "$base/run1_replay.log" 2>&1
replay_exit=$?
/usr/bin/python3 -B "$base/g1_window_eval.py" "$base/run1_corner" "$base/run1_window_eval.json" > "$base/run1_window_eval.log" 2>&1
window_exit=$?
sleep 3
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2 |parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$base/run1_residual_processes.txt" || true
set -e
printf 'trial=%s\nsource=%s\nwheel=%s\nreplay=%s\nwindow=%s\n' "$trial_exit" "$source_exit" "$wheel_exit" "$replay_exit" "$window_exit" > "$base/run1_exit.txt"
