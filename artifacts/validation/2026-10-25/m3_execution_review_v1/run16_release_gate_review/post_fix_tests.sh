#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
out=artifacts/validation/2026-10-25/m3_execution_review_v1/run16_release_gate_review/post_fix_tests
mkdir "$out" || exit 91
date -Is > "$out/start.txt"
sha256sum src/arena_autonomy/arena_autonomy/m3_execution_trial.py tests/test_m3_release_gate_review.py tests/test_m3_baseline_release.py > "$out/before.sha256"
export PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts:tests
python3 -B -m pytest -q -p no:cacheprovider tests/test_m3_release_gate_review.py tests/test_m3_baseline_release.py tests/test_m3_execution_trial.py tests/test_m3_command_selection.py --junitxml="$out/pytest.xml" > "$out/stdout.txt" 2> "$out/stderr.txt"
rc=$?
echo "$rc" > "$out/exitcode.txt"
sha256sum -c "$out/before.sha256" > "$out/source_check.txt"
date -Is > "$out/end.txt"
cat "$out/stdout.txt"
exit "$rc"
