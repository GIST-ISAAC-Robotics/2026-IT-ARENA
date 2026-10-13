#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/follow_trace_v1
test ! -e "$base/build2.log"
# 보존 소스 사본을 패키지로 탐색하지 않는다. 원본 사본에는 손대지 않는다.
colcon --log-base "$base/build_logs2" build --base-paths src --packages-select arena_autonomy arena_bringup --symlink-install > "$base/build2.log" 2>&1
tail -12 "$base/build2.log"
test ! -e "$base/preflight.log"
python3 -B -m pytest -q tests/test_follow_trace.py tests/test_local_follow.py tests/test_g1_side_occlusion.py tests/test_g1_traffic_eval.py tests/test_local_follow_validation.py > "$base/preflight.log" 2>&1
tail -6 "$base/preflight.log"
test ! -e "$base/side_left_s0"
test ! -e "$base/side_left_s0.runner.log"
set +e
python3 -B scripts/validate_g1_traffic.py --case side_left --seed 0 --mode g1 --config config/tests/g1_side_occlusion_v1.json --output "$base/side_left_s0" --record-braking-diagnostics --record-follow-decisions > "$base/side_left_s0.runner.log" 2>&1
run_exit=$?
set -e
echo "G1 validator exit=$run_exit"
tail -50 "$base/side_left_s0.runner.log"
test -f "$base/side_left_s0/follow_decisions.jsonl"
python3 -B scripts/replay_follow_trace.py "$base/side_left_s0/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
tail -10 "$base/replay.log"
