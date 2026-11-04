#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-03/side_departure_runtime_v1
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/run2_preflight.log"; then
  echo 'ACTUAL_EXIT_CODE=3 (another simulation is running)' > "$A/run2.exit"
  exit 3
fi
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B scripts/validate_g1_traffic.py --mode g1g2 --handoff-reserve paced --handoff-yield-progress \
  --handoff-rear-sweep --official-signal-course --config "$PWD/$A/scenario.json" --case side_departure_static --seed 0 \
  --record-follow-decisions --handoff-observed-motion --side-departure-trial --output "$PWD/$A/run2" > "$A/run2_console.log" 2>&1
code=$?
bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$A/run2" --output "$A/run2_surface.json" > "$A/run2_post.log" 2>&1
surface=$?
bash "$ENV" /usr/bin/python3 -B "$A/evaluate2.py" >> "$A/run2_post.log" 2>&1
analysis=$?
echo "ACTUAL_EXIT_CODE=$code SURFACE_EXIT=$surface ANALYSIS_EXIT=$analysis" > "$A/run2.exit"
exit "$code"
