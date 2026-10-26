#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D="$A/run52_queue_preflight"
[ ! -e "$D/exit_code.txt" ] || exit 91
cp "$A/queue_preflight.py" "$D/preflight_source.py"
cp config/tests/m2_queue_staggered_v1.json "$D/overlay.json"
cp scripts/m2_queue_eval.py "$D/evaluator_source.py"
cp tests/test_m2_queue_eval.py "$D/test_source.py"
python3 -B "$D/preflight_source.py" "$D/result.json" > "$D/preflight_stdout.txt" 2> "$D/preflight_stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/preflight_exit_code.txt"
[ "$code" = 0 ] || exit "$code"
python3 -B -m pytest -q tests/test_m2_queue_eval.py tests/test_traffic_actors.py tests/test_traffic_actor_runtime.py tests/test_g1_traffic_eval.py > "$D/stdout.txt" 2> "$D/stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/exit_code.txt"
exit "$code"
