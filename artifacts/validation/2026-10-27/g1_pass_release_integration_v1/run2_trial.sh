#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_pass_release_integration_v1
test ! -e "$base/run2_left"
test ! -e "$base/run2_trial.log"
/usr/bin/python3 -B "$base/run2_snapshot.py" --check > "$base/run2_pre_drive_source.log" 2>&1
set +e
/usr/bin/python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode g1 --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --partial-observation-policy bounded_local_v4 --parallel-hold-trial --pass-release-trial --actor-overlay config/tests/e3_onroad_encounter_v1.json --output "$base/run2_left" > "$base/run2_trial.log" 2>&1
trial_exit=$?
/usr/bin/python3 -B "$base/run2_snapshot.py" --check > "$base/run2_source_check.log" 2>&1
source_exit=$?
/usr/bin/python3 -B artifacts/validation/2026-10-27/partial_observation_v1/audit_wheel_centers.py "$base/run2_left" --output "$base/run2_wheel_centers.json" > "$base/run2_wheel_centers.log" 2>&1
wheel_exit=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py "$base/run2_left/follow_decisions.jsonl" --output "$base/run2_left/follow_trace_replay.json" > "$base/run2_replay.log" 2>&1
replay_exit=$?
set -e
printf 'trial=%s\nsource=%s\nwheel=%s\nreplay=%s\n' "$trial_exit" "$source_exit" "$wheel_exit" "$replay_exit" > "$base/run2_exit.txt"
if (( trial_exit != 0 || source_exit != 0 || wheel_exit != 0 || replay_exit != 0 )); then exit 1; fi
