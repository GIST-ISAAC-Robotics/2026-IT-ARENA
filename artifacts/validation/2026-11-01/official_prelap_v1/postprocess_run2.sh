#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/retime_autonomy_video.py artifacts/validation/2026-11-01/official_prelap_v1/run2 > artifacts/validation/2026-11-01/official_prelap_v1/retime_run2.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/official_prelap_v1/retime_run2.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/inspect_autonomy_video.py artifacts/validation/2026-11-01/official_prelap_v1/run2/lidar_third_person_simtime_1x.mp4 --output artifacts/validation/2026-11-01/official_prelap_v1/run2_video_review --times 0 2 4 8 12 24 36 46 > artifacts/validation/2026-11-01/official_prelap_v1/inspect_run2_video.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/official_prelap_v1/inspect_run2_video.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py artifacts/validation/2026-11-01/official_prelap_v1/run2 2.0 > artifacts/validation/2026-11-01/official_prelap_v1/run2_metrics.json 2> artifacts/validation/2026-11-01/official_prelap_v1/run2_metrics.stderr
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" > artifacts/validation/2026-11-01/official_prelap_v1/run2_metrics.exit
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/surface_contact_eval.py artifacts/validation/2026-11-01/official_prelap_v1/run2 --output artifacts/validation/2026-11-01/official_prelap_v1/run2_surface.json > artifacts/validation/2026-11-01/official_prelap_v1/run2_surface.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/official_prelap_v1/run2_surface.log
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B scripts/audit_motor_encoder_runtime.py artifacts/validation/2026-11-01/official_prelap_v1/run2 --output artifacts/validation/2026-11-01/official_prelap_v1/run2_motor_audit.json > artifacts/validation/2026-11-01/official_prelap_v1/run2_motor_audit.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/official_prelap_v1/run2_motor_audit.log
exit "$result"