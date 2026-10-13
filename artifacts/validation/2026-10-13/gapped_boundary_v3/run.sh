#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/gapped_boundary_v3
test ! -e "$base/fragments.png"
python3 -B scripts/plot_gapped_boundary.py artifacts/validation/2026-10-13/follow_overlap_v1/side_left_s0/follow_decisions.jsonl --output "$base/fragments.png" > "$base/fragments.json" 2>&1
test ! -e "$base/build.log"
colcon --log-base "$base/build_logs" build --base-paths src --packages-select arena_autonomy arena_bringup --symlink-install > "$base/build.log" 2>&1
tail -10 "$base/build.log"
test ! -e "$base/side_left_s0"
test ! -e "$base/side_left_s0.runner.log"
set +e
python3 -B scripts/validate_g1_traffic.py --case side_left --seed 0 --mode g1 --config config/tests/g1_side_occlusion_v1.json --output "$base/side_left_s0" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate > "$base/side_left_s0.runner.log" 2>&1
run_exit=$?
set -e
echo "G1 validator exit=$run_exit"
tail -40 "$base/side_left_s0.runner.log"
test -f "$base/side_left_s0/follow_decisions.jsonl"
python3 -B scripts/replay_follow_trace.py "$base/side_left_s0/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
tail -10 "$base/replay.log"
exit "$run_exit"
