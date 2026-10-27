#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
base=artifacts/validation/2026-10-27/g1_pass_release_operation_v1
out=$base/new_source_v1
test ! -e "$out"
mkdir "$out"
sha256sum src/arena_autonomy/arena_autonomy/pass_release_trial.py > "$out/pass_release_trial.sha256"
v27=artifacts/validation/2026-10-27
m3=artifacts/validation/2026-10-25/m3_execution_review_v1
cfg=$v27/g1_pass_release_integration_v1/run4_left/follow_decisions.jsonl
set +e
declare -A rc
/usr/bin/python3 -B scripts/replay_follow_trace.py $v27/g1_pass_release_integration_v1/run4_left/follow_decisions.jsonl --output "$out/replay_run4_left.json" > "$out/replay_run4_left.log" 2>&1; rc[run4]=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py $v27/g1_pass_release_integration_v1/run5_right/follow_decisions.jsonl --output "$out/replay_run5_right.json" > "$out/replay_run5_right.log" 2>&1; rc[run5]=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py $v27/g1_parallel_lane_v1/run1_left/follow_decisions.jsonl --output "$out/replay_off_parallel_left.json" > "$out/replay_off_parallel_left.log" 2>&1; rc[off_left]=$?
/usr/bin/python3 -B scripts/replay_follow_trace.py $v27/g1_partial_traffic_v1/run1_stop_restart/follow_decisions.jsonl --output "$out/replay_off_stop_restart.json" > "$out/replay_off_stop_restart.log" 2>&1; rc[off_stop]=$?
/usr/bin/python3 -B "$base/compare_saved_selector.py" $v27/g1_partial_traffic_v1/run1_stop_restart/follow_decisions.jsonl --output "$out/saved_stop_restart_on.json" > "$out/saved_stop_restart_on.log" 2>&1; rc[saved_stop]=$?
/usr/bin/python3 -B "$base/compare_cross_inputs.py" "$m3/run51_curve_stop_restart/run/follow_decisions.jsonl" --config-from "$cfg" --output "$out/cross_run51.json" > "$out/cross_run51.log" 2>&1; rc[cross51]=$?
/usr/bin/python3 -B "$base/compare_cross_inputs.py" "$m3/run32_corner_after_threat/run/follow_decisions.jsonl" --config-from "$cfg" --output "$out/cross_run32.json" > "$out/cross_run32.log" 2>&1; rc[cross32]=$?
/usr/bin/python3 -B "$base/compare_saved_selector.py" $v27/g1_partial_integration_v1/run11_lap/follow_decisions.jsonl --output "$out/saved_run11_lap_on.json" > "$out/saved_run11_lap_on.log" 2>&1; rc[saved_lap]=$?
sha256sum src/arena_autonomy/arena_autonomy/pass_release_trial.py > "$out/pass_release_trial_after.sha256"
set -e
for k in "${!rc[@]}"; do echo "$k=${rc[$k]}"; done | sort > "$out/exit_codes.txt"
