#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +e
python3 -B artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/audit_transition.py artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/drive_on > artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/audit_on.log 2>&1
result=$?
printf '%s\n' "$result" > artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/audit_on.exit
exit "$result"