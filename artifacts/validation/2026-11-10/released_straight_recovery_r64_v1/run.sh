#!/usr/bin/env bash
set -u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-10/released_straight_recovery_r64_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$A/run1" || -e "$A/registered.json" ]]; then exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight_processes.txt"; then exit 3; fi
bash "$ENV" /usr/bin/python3 -B "$A/run.py" > "$A/console.log" 2>&1
harness=$?
surface=-1; assessment=-1; continuity=-1
if [[ -f "$A/run1/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$A/run1" --output "$A/surface_run1.json" > "$A/surface_run1.log" 2>&1
  surface=$?
  bash "$ENV" /usr/bin/python3 -B artifacts/validation/2026-11-10/facility_traffic_stability_v1/analyze_continuity.py "$A/run1" --output "$A/continuity.json" > "$A/continuity.log" 2>&1
  continuity=$?
fi
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/postflight_processes_run1.txt"
leftover=$?
if [[ -f "$A/surface_run1.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B "$A/assess.py" > "$A/assessment_run1.log" 2>&1
  assessment=$?
fi
printf 'HARNESS_EXIT=%s\nSURFACE_EXIT=%s\nASSESSMENT_EXIT=%s\nCONTINUITY_EXIT=%s\nLEFTOVER_PGREP=%s\n' "$harness" "$surface" "$assessment" "$continuity" "$leftover" > "$A/run1.exit"
exit "$harness"
