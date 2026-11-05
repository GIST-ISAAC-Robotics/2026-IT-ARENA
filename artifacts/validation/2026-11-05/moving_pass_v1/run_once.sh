#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-05/moving_pass_v1
N=${1:?new registered folder required}
D="$A/$N"
if [[ ! -f "$D/registered.json" || -e "$D/run1" || -e "$D/run1.exit" ]]; then exit 2; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/preflight.log"; then exit 3; fi
start=$(date +%s)
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B "$A/run_once.py" run "$N" > "$D/run1.log" 2>&1; result=$?
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
surface=-1;review=-1
if [[ -f "$D/run1/report.json" ]]; then
  /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run1" --output "$D/surface.json" > "$D/surface.log" 2>&1; surface=$?
  /usr/bin/python3 -B "$A/review.py" "$N" > "$D/review.log" 2>&1; review=$?
fi
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight.log"; leftover=$?
echo "HARNESS_EXIT=$result SURFACE=$surface REVIEW=$review LEFTOVER_PGREP=$leftover WALL_S=$(( $(date +%s) - start ))" > "$D/run1.exit"
exit "$review"
