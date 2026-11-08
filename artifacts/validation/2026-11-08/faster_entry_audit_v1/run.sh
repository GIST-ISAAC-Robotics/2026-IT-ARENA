#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts"
python3 -B artifacts/validation/2026-11-08/faster_entry_audit_v1/audit.py > artifacts/validation/2026-11-08/faster_entry_audit_v1/audit.log 2>&1
rc=$?
printf '%s\n' "$rc" > artifacts/validation/2026-11-08/faster_entry_audit_v1/audit.exit
exit "$rc"
