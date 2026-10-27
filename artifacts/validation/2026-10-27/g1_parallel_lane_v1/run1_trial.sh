#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_parallel_lane_v1
test ! -e "$base/run1_tests.log"
/usr/bin/python3 -B "$base/run1_snapshot.py" > "$base/run1_snapshot.log" 2>&1
/usr/bin/python3 -B -m pytest -q tests/test_parallel_lane_trial.py tests/test_g1_partial_traffic.py tests/test_g1_partial_policy.py tests/test_local_follow.py tests/test_follow_trace.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_source_provenance_r0_plus_b3.py tests/test_g1_traffic_eval.py tests/test_g1_traffic_extensions.py > "$base/run1_tests.log" 2>&1
tail -3 "$base/run1_tests.log"
/usr/bin/python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-27/g1_partial_traffic_v1/run1_stop_restart/follow_decisions.jsonl --output "$base/run1_off_replay.json" > "$base/run1_off_replay.log" 2>&1
colcon build --symlink-install --packages-select arena_autonomy arena_bringup > "$base/run1_build.log" 2>&1
/usr/bin/python3 -B "$base/run1_snapshot.py" --check > "$base/run1_pre_drive_source.log" 2>&1
set +e
/usr/bin/python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode g1 --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --partial-observation-policy bounded_local_v4 --parallel-hold-trial --actor-overlay config/tests/e3_onroad_encounter_v1.json --output "$base/run1_left" > "$base/run1_trial.log" 2>&1
trial_exit=$?
/usr/bin/python3 -B "$base/run1_snapshot.py" --check > "$base/run1_source_check.log" 2>&1
source_exit=$?
/usr/bin/python3 -B artifacts/validation/2026-10-27/partial_observation_v1/audit_wheel_centers.py "$base/run1_left" --output "$base/run1_wheel_centers.json" > "$base/run1_wheel_centers.log" 2>&1
wheel_exit=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py "$base/run1_left/follow_decisions.jsonl" --output "$base/run1_left/follow_trace_replay.json" > "$base/run1_replay.log" 2>&1
replay_exit=$?
set -e
printf 'trial=%s\nsource=%s\nwheel=%s\nreplay=%s\n' "$trial_exit" "$source_exit" "$wheel_exit" "$replay_exit" > "$base/run1_exit.txt"
if (( trial_exit != 0 || source_exit != 0 || wheel_exit != 0 || replay_exit != 0 )); then exit 1; fi
