#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-13/crop_reach_drive_r431_v1'
receipt="$artifact/execution_v1"
test ! -e "$artifact/preflight_v1"
test ! -e "$artifact/registered.json"
test ! -e "$artifact/run1"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B - "$artifact" <<'PY'
import hashlib,json,sys
from pathlib import Path
a=Path(sys.argv[1])
assert hashlib.sha256((a/'prepared.json').read_bytes()).hexdigest()=='365c29b824c366377bc05f076b937c10393ed421570b6e06ba30723bbfb3d95e'
assert len(json.loads((a/'prepared.json').read_bytes())['frozen_inputs'])==1337
PY
mkdir "$artifact/preflight_v1"
python3 -B "$artifact/prepare.py" --check | tee "$artifact/preflight_v1/check_before.json"
set +e
timeout --signal=TERM --kill-after=10s 180s python3 -B -m pytest -q -p no:cacheprovider \
  "$artifact/test_preflight.py" "$artifact/test_launch_selection.py" \
  --junitxml="$artifact/preflight_v1/junit.xml" 2>&1 | tee "$artifact/preflight_v1/pytest.log"
preflight_code=${PIPESTATUS[0]}
set -e
printf '%s\n' "$preflight_code" > "$artifact/preflight_v1/pytest.exit"
test "$preflight_code" -eq 0
python3 -B - "$artifact/preflight_v1/junit.xml" <<'PY'
import sys,xml.etree.ElementTree as ET
root=ET.parse(sys.argv[1]).getroot()
suites=[root] if root.tag=='testsuite' else list(root)
assert sum(int(s.attrib['tests']) for s in suites)==16
assert all(int(s.attrib.get(k,0))==0 for s in suites for k in ('failures','errors','skipped'))
PY
python3 -B "$artifact/prepare.py" --check | tee "$artifact/preflight_v1/check_after.json"
date --utc --iso-8601=seconds > "$receipt/run_started_utc.txt"
# stop_owned.py의 동결된 /proc 소유 확인과 일치시킨다. bytecode는 환경으로 차단한다.
python3 "$artifact/run.py" > "$receipt/runner.log" 2>&1 &
run_pid=$!
printf '%s\n' "$run_pid" > "$receipt/run.pid"
printf 'r431 preflight passed; one owned run started, pid=%s\n' "$run_pid"
set +e
wait "$run_pid"
run_code=$?
set -e
printf '%s\n' "$run_code" > "$receipt/run.exit"
date --utc --iso-8601=seconds > "$receipt/run_finished_utc.txt"
python3 -B "$artifact/prepare.py" --check | tee "$receipt/check_after_run.json"
printf 'r431 run exited: %s\n' "$run_code"
exit "$run_code"
