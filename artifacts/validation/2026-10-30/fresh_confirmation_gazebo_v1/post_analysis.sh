#!/usr/bin/env bash
# 주행 뒤 사후 평가만 수행한다(제어/주행 재실행 없음). 각 단계의 종료 코드를 따로 남긴다.
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-30/fresh_confirmation_gazebo_v1
run=drive_right_on
test -d "$base/$run"
test ! -e "$base/post_analysis_exit.txt"
/usr/bin/python3 -B artifacts/validation/2026-10-29/rear_reapproach_gazebo_v1/summarize_runs.py "$base/$run" --output "$base/registered_summary.json" > "$base/registered_summary.log" 2>&1
summary=$?
/usr/bin/python3 -B scripts/surface_contact_eval.py "$base/$run" --output "$base/${run}_surface_contact.json" > "$base/${run}_surface_contact.log" 2>&1
surface=$?
/usr/bin/python3 -B "$base/analyze_preference.py" "$base/$run" --output "$base/preference_analysis.json" > "$base/preference_analysis.log" 2>&1
preference=$?
/usr/bin/python3 -B "$base/signed_lateral.py" "$base/$run" artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1/drive_on artifacts/validation/2026-10-29/rear_lane_preference_gazebo_v1/drive_on --output "$base/signed_lateral.json" > "$base/signed_lateral.log" 2>&1
signed=$?
printf 'summary=%s\nsurface=%s\npreference=%s\nsigned=%s\n' "$summary" "$surface" "$preference" "$signed" > "$base/post_analysis_exit.txt"
