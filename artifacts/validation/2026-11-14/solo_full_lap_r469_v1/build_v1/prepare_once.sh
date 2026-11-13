#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/solo_full_lap_r469_v1'
b="$artifact/build_v1"
test ! -e "$b/session_started_utc.txt"
test ! -e "$artifact/prepared.json"
date -u +%FT%TZ > "$b/session_started_utc.txt"
finish() {
  rc=$?
  trap - EXIT
  export R469_SESSION_EXIT="$rc"
  export R469_AUDIT_PATH="$PWD/$b/session_cleanup.json"
  printf '%s\n' "$rc" > "$b/session.exit"
  python3 -B "$b/session_audit.py" > "$b/session_audit.log" 2>&1
  audit_rc=$?
  if [ "$audit_rc" -ne 0 ]; then exit "$audit_rc"; fi
  exit "$rc"
}
trap finish EXIT
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 -B "$artifact/prepare.py" --verify-source > "$b/source_before.json"
set +e
colcon build --symlink-install --packages-select arena_autonomy > "$b/build.log" 2>&1
build_rc=$?
set -e
printf '%s\n' "$build_rc" > "$b/build.exit"
if [ "$build_rc" -ne 0 ]; then exit "$build_rc"; fi
source install/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
set +e
python3 -B "$b/check_import_cli.py" > "$b/import_cli.log" 2>&1
check_rc=$?
set -e
printf '%s\n' "$check_rc" > "$b/import_cli.exit"
if [ "$check_rc" -ne 0 ]; then exit "$check_rc"; fi
python3 -B "$artifact/prepare.py" --freeze > "$b/freeze.json"
python3 -B "$artifact/prepare.py" --check > "$b/check_after_freeze.json"
