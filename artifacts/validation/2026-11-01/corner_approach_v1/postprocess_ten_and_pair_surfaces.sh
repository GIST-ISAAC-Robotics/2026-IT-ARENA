#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh 2.7777777777777777 > artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_metrics.json 2> artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_metrics.stderr
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" > artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_metrics.exit
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh --output artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_surface.json > artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_surface.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_surface.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/audit_motor_encoder_runtime.py artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh --output artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_motor_audit.json > artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_motor_audit.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/corner_approach_v1/solo_10kmh_motor_audit.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-01/race_follow_pair_v1/fixed --output artifacts/validation/2026-11-01/race_follow_pair_v1/fixed_surface.json > artifacts/validation/2026-11-01/race_follow_pair_v1/fixed_surface.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_pair_v1/fixed_surface.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-01/race_follow_pair_v1/moving --output artifacts/validation/2026-11-01/race_follow_pair_v1/moving_surface.json > artifacts/validation/2026-11-01/race_follow_pair_v1/moving_surface.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_pair_v1/moving_surface.log
exit "$result"