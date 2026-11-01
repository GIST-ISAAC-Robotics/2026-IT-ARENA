#!/usr/bin/env bash
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
cmd1='python3 -B scripts/audit_motor_encoder_runtime.py artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps --output artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps_motor_audit.json'
printf '$ %s\n' "$cmd1"
eval "$cmd1" 2>&1 | tee artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps_motor_audit_v2.log
r1=${PIPESTATUS[0]}; printf 'ACTUAL_EXIT_CODE=%s\n' "$r1" | tee -a artifacts/validation/2026-11-01/race_continuity_v1/solo_1mps_motor_audit_v2.log
cmd2='python3 -B scripts/audit_motor_encoder_runtime.py artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5 --output artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_motor_audit.json'
printf '$ %s\n' "$cmd2"
eval "$cmd2" 2>&1 | tee artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_motor_audit.log
r2=${PIPESTATUS[0]}; printf 'ACTUAL_EXIT_CODE=%s\n' "$r2" | tee -a artifacts/validation/2026-11-01/race_continuity_v1/solo_1p5_motor_audit.log
if [ "$r1" -ne 0 ]; then exit "$r1"; else exit "$r2"; fi
