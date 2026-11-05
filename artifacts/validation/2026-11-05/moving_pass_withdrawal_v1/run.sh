#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-05/moving_pass_withdrawal_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
N=${1:?new registered case};D="$A/$N"
if [[ ! -f "$D/registered.json" || -e "$D/run1.exit" || -e "$D/run1" ]]; then exit 2; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/preflight.log"; then exit 3; fi
start=$(date +%s)
bash "$ENV" /usr/bin/python3 -B "$A/run.py" "$N" > "$D/run1.log" 2>&1; harness=$?
surface=-1;assessment=-1
if [[ -f "$D/run1/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run1" --output "$D/surface.json" > "$D/surface.log" 2>&1; surface=$?
  if [[ "$surface" == 0 ]]; then
    /usr/bin/python3 -B "$A/assess.py" "$N" > "$D/assessment.log" 2>&1; assessment=$?
  fi
fi
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight.log"; leftover=$?
echo "HARNESS_EXIT=$harness SURFACE=$surface ASSESSMENT=$assessment LEFTOVER_PGREP=$leftover WALL_S=$(( $(date +%s)-start ))" > "$D/run1.exit"
exit "$assessment"
