#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-03/right_reapproach_v3
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/run1_preflight.log"; then
  echo 'ACTUAL_EXIT_CODE=3 (another simulation is running)' > "$A/run1.exit"
  exit 3
fi
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B scripts/validate_g1_traffic.py --mode g1g2 --handoff-reserve paced --handoff-yield-progress \
  --handoff-rear-sweep --official-signal-course --config "$PWD/$A/scenario.json" --case right_reapproach --seed 0 \
  --record-follow-decisions --handoff-observed-motion --side-departure-trial --current-lane-preference-trial --output "$PWD/$A/run1" > "$A/run1_console.log" 2>&1
code=$?
bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$A/run1" --output "$A/run1_surface.json" > "$A/run1_post.log" 2>&1
surface=$?
bash "$ENV" /usr/bin/python3 -B "$A/evaluate.py" >> "$A/run1_post.log" 2>&1
analysis=$?
echo "ACTUAL_EXIT_CODE=$code SURFACE_EXIT=$surface ANALYSIS_EXIT=$analysis" > "$A/run1.exit"
exit "$code"
