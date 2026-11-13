#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-13/solo_full_lap_r465_v1'
receipt="$artifact/execution_v1"
test ! -e "$artifact/registered.json"
test ! -e "$artifact/run1"
test ! -e "$receipt/run.pid"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B - "$artifact" <<'PY'
import hashlib,json,sys
from pathlib import Path
a=Path(sys.argv[1]);r=json.loads((a/'root_execution_review.json').read_bytes())
assert r['root_reviewed'] and r['execution_limit']==1 and r['repeat_on_failure'] is False
assert hashlib.sha256((a/'prepared.json').read_bytes()).hexdigest()==r['prepared_sha256']
assert hashlib.sha256((a/'sources.zip').read_bytes()).hexdigest()==r['source_archive_sha256']
assert json.loads((a/'recipe_manifest.json').read_bytes())['recipe_verified']
build=json.loads((a/'build_v1/receipt.json').read_bytes())
assert build['build_exitcode']==build['import_cli_exitcode']==0 and build['native_import_verified']
assert json.loads((a/'prepared.json').read_bytes())['source_policy']=='r457_plus_exact_root_reviewed_delta'
PY
python3 -B "$artifact/prepare.py" --check > "$receipt/check_before_run.json"
date --utc --iso-8601=seconds > "$receipt/run_started_utc.txt"
python3 "$artifact/run.py" > "$receipt/runner.log" 2>&1 &
run_pid=$!
printf '%s\n' "$run_pid" > "$receipt/run.pid"
printf 'r465 one owned run started, pid=%s\n' "$run_pid"
set +e
wait "$run_pid"
run_code=$?
set -e
printf '%s\n' "$run_code" > "$receipt/run.exit"
date --utc --iso-8601=seconds > "$receipt/run_finished_utc.txt"
python3 -B "$artifact/prepare.py" --check > "$receipt/check_after_run.json"
printf 'r465 run exited: %s\n' "$run_code"
exit "$run_code"
