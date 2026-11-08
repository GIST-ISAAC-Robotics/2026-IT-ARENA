#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/solo_contact_diagnosis_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [ -e "$A/run1" ]; then exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight.txt"; then exit 3; fi
bash "$ENV" python3 -B "$A/register.py" > "$A/register.log" 2>&1
status=$?;printf '%s\n' "$status" > "$A/register.exit";[ "$status" -eq 0 ] || exit "$status"
bash "$ENV" python3 -B "$A/run.py" > "$A/run.log" 2>&1
status=$?;printf '%s\n' "$status" > "$A/run.exit"
if [ -f "$A/run1/report.json" ];then
    bash "$ENV" python3 -B scripts/surface_contact_eval.py "$A/run1" --output "$A/surface.json" > "$A/surface.log" 2>&1
    surface=$?;printf '%s\n' "$surface" > "$A/surface.exit"
fi
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/postflight.txt"
exit "$status"
