#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_forward_pace_v3
OLD=artifacts/validation/2026-11-02/official_forward_v1
test ! -e "$OLD/a4"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$A/build_a4.log" 2>&1
source install/local_setup.bash
set +e
bash "$OLD/scene.sh" moving_stop_restart a4 --handoff-observed-motion
run_code=$?
set -e
printf 'RUN_EXIT_CODE=%s\n' "$run_code" > "$A/a4.exit"
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -I -B "$OLD/analyze.py" a4 > "$A/a4_analysis.log" 2>&1
python3 -B scripts/surface_contact_eval.py "$OLD/a4" --output "$A/a4_surface.json" > "$A/a4_surface.log" 2>&1
exit "$run_code"
