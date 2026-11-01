#!/usr/bin/env bash
# 등록된 대표 코스를 한 번 실행하고 원시 결과를 덮어쓰지 않은 채 사후 평가와 영상 회수를 남긴다.
set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
B=artifacts/validation/2026-11-01/course_compare_v1
R=$B/run_paced
python3 -B "$B/run_course.py"
run_result=$?
step() {
  name="$1"; shift
  "$@" > "$B/${name}.log" 2>&1
  printf 'ACTUAL_EXIT_CODE=%s\n' "$?" >> "$B/${name}.log"
}
step surface python3 -B scripts/surface_contact_eval.py "$R" --output "$B/run_paced_surface.json"
step motor_audit python3 -B scripts/audit_motor_encoder_runtime.py "$R" --output "$B/run_paced_motor_audit.json"
step measure python3 -B artifacts/validation/2026-11-01/mixed_course_v1/measure_course.py "$R" "$B/run_paced_summary.json"
step post python3 -B "$B/post_course.py"
step retime python3 -B scripts/retime_autonomy_video.py "$R/video"
step inspect python3 -B scripts/inspect_autonomy_video.py "$R/video/lidar_third_person_simtime_1x.mp4" --output "$R/video_inspection"
printf 'RUN_EXIT_CODE=%s\n' "$run_result" > "$B/run_paced.done"
