#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_shadow_v1
for name in motion15 combined16; do
 test ! -e "$base/${name}_wall_diagnosis.log" || exit 90
 /usr/bin/python3 -B "$base/diagnose_motion_wall.py" "$base/$name" > "$base/${name}_wall_diagnosis.log" 2>&1
 result=$?
 printf '{"diagnosis":%s}\n' "$result" > "$base/${name}_wall_diagnosis_exit.json"
 if test "$result" -ne 0; then exit "$result"; fi
done
for name in range13 motion15 combined16; do
 /usr/bin/python3 -B "$base/plot_sequence_full.py" "$base/$name" || exit $?
done
