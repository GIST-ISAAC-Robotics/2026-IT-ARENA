#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-13/solo_full_lap_r457_v1'
b="$artifact/build_v1"
test ! -e "$b/session_started_utc.txt"
date -u +%FT%TZ > "$b/session_started_utc.txt"
finish() {
  rc=$?
  trap - EXIT
  export R457_SESSION_EXIT="$rc"
  printf '%s\n' "$rc" > "$b/session.exit"
  date -u +%FT%TZ > "$b/session_finished_utc.txt"
  python3 -B "$b/session_audit.py" > "$b/session_audit.log" 2>&1
  audit_rc=$?
  cat "$b/session_audit.log"
  if [ "$audit_rc" -ne 0 ]; then exit "$audit_rc"; fi
  exit "$rc"
}
trap finish EXIT
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$artifact/prepare.py" > "$b/source_before.json"
printf '%s\n' 'colcon build --symlink-install --packages-select arena_autonomy' > "$b/command.txt"
set +e
colcon build --symlink-install --packages-select arena_autonomy > "$b/build.log" 2>&1
build_rc=$?
set -e
printf '%s\n' "$build_rc" > "$b/build.exit"
if [ "$build_rc" -ne 0 ]; then cat "$b/build.log"; exit "$build_rc"; fi
source install/setup.bash
set +e
python3 -B "$b/check_import_cli.py" > "$b/import_cli.log" 2>&1
check_rc=$?
set -e
printf '%s\n' "$check_rc" > "$b/import_cli.exit"
cat "$b/import_cli.log"
if [ "$check_rc" -ne 0 ]; then exit "$check_rc"; fi
python3 -B "$artifact/prepare.py" --freeze > "$b/freeze.json"
python3 -B "$artifact/prepare.py" --check > "$b/check_after_freeze.json"
cat "$b/freeze.json"
