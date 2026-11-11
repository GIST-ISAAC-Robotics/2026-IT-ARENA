#!/usr/bin/env bash
set -u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-10/facility_traffic_stability_v1
P=artifacts/validation/2026-11-09/signal_posts_markers_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
NAME=${1:?output identifier required}
if [[ ! "$NAME" =~ ^[a-zA-Z][a-zA-Z0-9_]*$ ]]; then exit 2; fi
D="$A/$NAME"
if [[ -e "$D" ]]; then echo 'output exists'; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'; then exit 3; fi
bash "$ENV" /usr/bin/python3 -B "$A/stationary_run.py" "$@" > "$A/${NAME}_console.log" 2>&1
harness=$?
if [[ ! -d "$D" ]]; then exit "$harness"; fi
surface=-1; goal=-1; posts=-1; assessment=-1; continuity=-1
if [[ -f "$D/run1/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run1" --output "$D/surface_run1.json" > "$D/surface_run1.log" 2>&1
  surface=$?
  bash "$ENV" /usr/bin/python3 -B "$P/stationary_goal_eval.py" "$D/run1" src/arena_gazebo/worlds/it_arena_official_v2026_11_06 > "$D/goal.log" 2>&1
  goal=$?
  bash "$ENV" /usr/bin/python3 -B "$P/evaluate_traffic_posts.py" "$D/run1" > "$D/posts.log" 2>&1
  posts=$?
  bash "$ENV" /usr/bin/python3 -B "$A/analyze_continuity.py" "$D/run1" --output "$D/continuity.json" > "$D/continuity.log" 2>&1
  continuity=$?
fi
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight_processes_run1.txt"
leftover=$?
if [[ -f "$D/run1/dense_post_clearance.json" && -f "$D/run1_goal_surface_corrected.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B "$P/stationary_posts_assess.py" "$D/run1" > "$D/assessment.log" 2>&1
  assessment=$?
fi
printf 'HARNESS_EXIT=%s\nSURFACE_EXIT=%s\nGOAL_EXIT=%s\nPOSTS_EXIT=%s\nASSESSMENT_EXIT=%s\nCONTINUITY_EXIT=%s\nLEFTOVER_PGREP=%s\n' "$harness" "$surface" "$goal" "$posts" "$assessment" "$continuity" "$leftover" > "$D/run1.exit"
exit "$harness"
