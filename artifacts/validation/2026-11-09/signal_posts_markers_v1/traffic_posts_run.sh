#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-09/signal_posts_markers_v1
D=$A/traffic_all_posts
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$D" ]]; then echo 'existing output refused'; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'; then exit 3; fi
bash "$ENV" /usr/bin/python3 -B "$A/traffic_posts_run.py" > "$A/traffic_posts_console.log" 2>&1
harness=$?
surface=-1
if [[ -f "$D/run1/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run1" --output "$D/surface.json" > "$D/surface.log" 2>&1
  surface=$?
fi
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight_processes.txt"
leftover=$?
assessment=-1
if [[ -f "$D/surface.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B "$A/traffic_posts_assess.py" > "$D/assessment.log" 2>&1
  assessment=$?
fi
printf 'HARNESS_EXIT=%s\nSURFACE_EXIT=%s\nASSESSMENT_EXIT=%s\nLEFTOVER_PGREP=%s\n' "$harness" "$surface" "$assessment" "$leftover" > "$D/run1.exit"
exit "$harness"
