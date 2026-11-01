#!/usr/bin/env bash
# 실행 결과 회수. 한 단계가 실패해도 나머지 근거를 남긴다. 판정선은 바꾸지 않는다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-01/solo_preview_gate_v1
R=$A/run1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
worst=0
step() { name=$1; shift; "$@" > "$A/post_$name.log" 2>&1; code=$?; echo "ACTUAL_EXIT_CODE=$code" >> "$A/post_$name.log"; echo "$name=$code"; [ "$code" -ne 0 ] && worst=1; }
step retime python3 -B scripts/retime_autonomy_video.py "$R"
step video python3 -B scripts/inspect_autonomy_video.py "$R/lidar_third_person_simtime_1x.mp4" --output "${R}_video_review"
step metrics bash -c "python3 -B artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py '$R' 2.7777777778 > '${R}_metrics.json'"
step surface python3 -B scripts/surface_contact_eval.py "$R" --output "${R}_surface.json"
step motor python3 -B scripts/audit_motor_encoder_runtime.py "$R" --output "${R}_motor_audit.json"
step usage python3 -B "$A/usage.py"
step online python3 -B "$A/online_cost.py"
step response python3 -B "$A/response_check.py" "$PWD/$A" "$PWD/$A/response_run.json"
step compare python3 -B "$A/lap_compare.py"
echo "ACTUAL_EXIT_CODE=$worst" > "$A/postprocess.exit"
exit "$worst"
