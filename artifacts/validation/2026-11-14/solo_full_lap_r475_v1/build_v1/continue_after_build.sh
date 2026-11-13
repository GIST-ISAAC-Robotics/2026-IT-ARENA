#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/solo_full_lap_r475_v1'
b="$artifact/build_v1"
test "$(cat "$b/build.exit")" = '0'
test ! -e "$b/session_v2_started_utc.txt"
test ! -e "$artifact/prepared.json"
date -u +%FT%TZ > "$b/session_v2_started_utc.txt"
finish() {
  rc=$?
  trap - EXIT
  export R475_SESSION_EXIT="$rc"
  export R475_AUDIT_PATH="$PWD/$b/session_v2_cleanup.json"
  printf '%s\n' "$rc" > "$b/session_v2.exit"
  python3 -B "$b/session_audit.py" > "$b/session_v2_audit.log" 2>&1
  audit_rc=$?
  if [ "$audit_rc" -ne 0 ]; then exit "$audit_rc"; fi
  exit "$rc"
}
trap finish EXIT
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$artifact/prepare.py" --verify-source > "$b/source_v2_before.json"
set +e
python3 -B "$b/check_import_cli.py" > "$b/import_cli_v2.log" 2>&1
check_rc=$?
set -e
printf '%s\n' "$check_rc" > "$b/import_cli_v2.exit"
if [ "$check_rc" -ne 0 ]; then exit "$check_rc"; fi
set +e
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -B -m pytest -q -p no:cacheprovider \
  --junitxml="$b/native_current_mask_v2.xml" tests/test_current_mask_native.py > "$b/native_current_mask_v2.log" 2>&1
native_test_rc=$?
set -e
printf '%s\n' "$native_test_rc" > "$b/native_current_mask_v2.exit"
if [ "$native_test_rc" -ne 0 ]; then exit "$native_test_rc"; fi
python3 -B - "$b" <<'PY'
import hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
b=Path(sys.argv[1]);report=b/'native_current_mask_v2.xml'
tests=ET.parse(report).findall('.//testcase')
assert len(tests)==10 and not any(t.findall('failure')+t.findall('error')+t.findall('skipped') for t in tests)
value=dict(tests=10,passed=10,failed=0,skipped=0,report_sha256=hashlib.sha256(report.read_bytes()).hexdigest())
(b/'native_current_mask_v2.json').write_text(json.dumps(value,indent=2)+'\n')
receipt=json.loads((b/'receipt.json').read_bytes());receipt['native_entrypoint_pytest']=value
(b/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
PY
python3 -B "$artifact/prepare.py" --freeze > "$b/freeze.json"
python3 -B "$artifact/prepare.py" --check > "$b/check_after_freeze.json"
