#!/usr/bin/env bash
# 등록된 한 쪽만 실행하고 사후 평가를 남긴다. 사용: run_fast_case.sh closing|low_motion
set +u
arm="$1"
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
B=artifacts/validation/2026-11-01/follow_continuity_v1
N="fast_stop_$arm"
python3 -B "$B/run_fast_stop.py" "$arm"
run_result=$?
python3 -B scripts/surface_contact_eval.py "$B/$N" --output "$B/${N}_surface.json" > "$B/${N}_surface.log" 2>&1
printf 'ACTUAL_EXIT_CODE=%s\n' "$?" >> "$B/${N}_surface.log"
python3 -B scripts/audit_motor_encoder_runtime.py "$B/$N" --output "$B/${N}_motor_audit.json" > "$B/${N}_motor_audit.log" 2>&1
printf 'ACTUAL_EXIT_CODE=%s\n' "$?" >> "$B/${N}_motor_audit.log"
printf 'RUN_EXIT_CODE=%s\n' "$run_result" > "$B/$N.done"
