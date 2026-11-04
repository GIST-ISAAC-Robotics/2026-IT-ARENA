#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-03/post_free_race_distance_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight.log"; then
  echo 'ACTUAL_EXIT_CODE=3 (another simulation is running)' > "$A/runtime.exit"
  exit 3
fi
bash "$ENV" /usr/bin/python3 -B "$A/run.py" > "$A/runtime.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/runtime.exit"
if [ ! -f "$A/run1/report.json" ]; then exit "$code"; fi
bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$A/run1" --output "$A/run1_surface.json" > "$A/surface.log" 2>&1
surface=$?
echo "SURFACE_EXIT=$surface" > "$A/surface.exit"
if [ "$surface" -ne 0 ]; then exit "$surface"; fi
bash "$ENV" /usr/bin/python3 -B "$A/review.py" > "$A/review.log" 2>&1
review=$?
echo "REVIEW_EXIT=$review" > "$A/review.exit"
exit "$code"
