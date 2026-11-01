#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path 1.5 > artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_metrics.json 2> artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_metrics.stderr
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" > artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_metrics.exit
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path --output artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_surface.json > artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_surface.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_surface.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/audit_motor_encoder_runtime.py artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path --output artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_motor_audit.json > artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_motor_audit.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_race_path_motor_audit.log
exit "$result"