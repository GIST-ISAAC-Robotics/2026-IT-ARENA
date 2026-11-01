#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/audit_motor_encoder_runtime.py artifacts/validation/2026-11-01/race_continuity_v1/solo_2p0_corrected --output artifacts/validation/2026-11-01/race_continuity_v1/solo_2p0_corrected_motor_audit.json > artifacts/validation/2026-11-01/race_continuity_v1/solo_2p0_corrected_motor_audit.log 2>&1
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_continuity_v1/solo_2p0_corrected_motor_audit.log
exit "$result"
