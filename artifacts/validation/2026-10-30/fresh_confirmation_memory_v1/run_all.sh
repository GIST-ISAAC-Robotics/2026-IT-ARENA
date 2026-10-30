#!/usr/bin/env bash
# fresh 확인 기억 후보의 오프라인 인과 비교: 후보 생성 → 국소 반례 → 저장 입력 3건 → 합성 폐루프(런타임/후보).
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/fresh_confirmation_memory_v1
export PYTHONPATH="$PWD/$base:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/run_all_exit.txt"
step() {
  local name=$1; shift
  "$@" > "$base/$name.log" 2>&1
  local rc=$?
  printf '%s=%s\n' "$name" "$rc" >> "$base/run_all_exit.partial"
  return "$rc"
}
finish() { mv "$base/run_all_exit.partial" "$base/run_all_exit.txt"; exit "$1"; }
step make /usr/bin/python3 -B "$base/make_candidate.py" || finish 1
step tests /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_candidate_memory.py" || finish 1
step saved_gazebo_right /usr/bin/python3 -B "$base/shadow_saved.py" artifacts/validation/2026-10-30/rear_lane_measured_right_gazebo_v1/drive_right_on/follow_decisions.jsonl --output "$base/saved_gazebo_right.json"
step saved_gazebo_left /usr/bin/python3 -B "$base/shadow_saved.py" artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1/drive_on/follow_decisions.jsonl --output "$base/saved_gazebo_left.json"
step saved_synthetic_combined_right /usr/bin/python3 -B "$base/shadow_saved.py" artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4/comparison_combined/B_gazebo_right_trace.jsonl --output "$base/saved_synthetic_combined_right.json"
for policy in runtime memory; do
  step "probe_${policy}_ideal" /usr/bin/python3 -B "$base/probe_memory.py" --policy "$policy" --sensor-profile ideal \
    --output "$base/probe_${policy}_ideal" --trace --cases B_gazebo_left B_gazebo_right
  step "probe_${policy}_combined" /usr/bin/python3 -B "$base/probe_memory.py" --policy "$policy" --sensor-profile combined \
    --output "$base/probe_${policy}_combined" --trace --cases B_gazebo_left B_gazebo_right C_lead_stop_left
done
finish 0
