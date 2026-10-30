#!/usr/bin/env bash
# 저장 호출 재계산·분해만 수행한다(주행/운영 코드 변경 없음). 합성 v4 3건 + Gazebo 좌우 2건.
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-30/rear_right_combined_stall_v1
v4=artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4
test ! -e "$base/analyze_all_exit.txt"
run() {
  local name=$1 trace=$2 first=$3 last=$4
  /usr/bin/python3 -B "$base/analyze_stall.py" "$trace" --output "$base/$name.json" --first "$first" --last "$last" > "$base/$name.log" 2>&1
  printf '%s=%s\n' "$name" "$?" >> "$base/analyze_all_exit.partial"
}
run synthetic_combined_right "$v4/comparison_combined/B_gazebo_right_trace.jsonl" 0.5 3.0
run synthetic_combined_left "$v4/comparison_combined/B_gazebo_left_trace.jsonl" 0.5 3.0
run synthetic_ideal_right "$v4/comparison_ideal/B_gazebo_right_trace.jsonl" 0.5 3.0
# Gazebo: 초록 관측 시각부터 4.5 s(오른쪽 7.689, 왼쪽 7.697).
run gazebo_right_1030 artifacts/validation/2026-10-30/rear_lane_measured_right_gazebo_v1/drive_right_on/follow_decisions.jsonl 7.689 12.189
run gazebo_left_1030 artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1/drive_on/follow_decisions.jsonl 7.697 12.197
mv "$base/analyze_all_exit.partial" "$base/analyze_all_exit.txt"
