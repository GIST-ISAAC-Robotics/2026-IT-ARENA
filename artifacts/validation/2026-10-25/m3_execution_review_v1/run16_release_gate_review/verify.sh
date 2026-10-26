#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
out=artifacts/validation/2026-10-25/m3_execution_review_v1/run16_release_gate_review/verified
base=artifacts/validation/2026-10-25/m3_execution_review_v1
mkdir "$out" || exit 91
date -Is > "$out/start.txt"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
files=(src/arena_autonomy/arena_autonomy/*.py scripts/validate_g1_traffic.py tests/test_m3*.py tests/test_source_provenance_r0_plus_b3.py config/tests/m3*.json "$base/compare_release_gate.py")
sha256sum "${files[@]}" > "$out/before.sha256"
tar -cf "$out/sources.tar" "${files[@]}"
python3 -B -m pytest -q -p no:cacheprovider tests/test_m3_release_gate_review.py tests/test_m3_baseline_release.py tests/test_m3_execution_trial.py tests/test_m3_command_selection.py tests/test_m3_execution_integration.py tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py --junitxml="$out/pytest.xml" > "$out/stdout.txt" 2> "$out/stderr.txt"
rc=$?
echo "$rc" > "$out/exitcode.txt"
if [ "$rc" -eq 0 ]; then
 python3 -B "$base/compare_release_gate.py" "$base/run13_selection_closed_loop/run/follow_decisions.jsonl" "$base/run9_closed_loop_left/run/follow_decisions.jsonl" --output "$out/replay" --old-summary "$base/run14_baseline_release/release_comparison/summary.json" > "$out/replay_stdout.txt" 2> "$out/replay_stderr.txt"
 echo "$?" > "$out/replay_exitcode.txt"
fi
sha256sum -c "$out/before.sha256" > "$out/source_check.txt"
date -Is > "$out/end.txt"
cat "$out/stdout.txt"
exit "$rc"
