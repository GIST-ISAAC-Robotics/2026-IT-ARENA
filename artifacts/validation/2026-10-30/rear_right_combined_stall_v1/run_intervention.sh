#!/usr/bin/env bash
# 오프라인 개입 비교: 후보 생성 → 국소 반례 → 합성 B_gazebo 좌우(런타임/후보 × combined/ideal) → 저장 Gazebo 입력 대안 계산.
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/rear_right_combined_stall_v1
export PYTHONPATH="$PWD/$base:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/intervention_exit.txt"
step() {
  local name=$1; shift
  "$@" > "$base/$name.log" 2>&1
  local rc=$?
  printf '%s=%s\n' "$name" "$rc" >> "$base/intervention_exit.partial"
  return "$rc"
}
step make /usr/bin/python3 -B "$base/make_candidate.py" || { mv "$base/intervention_exit.partial" "$base/intervention_exit.txt"; exit 1; }
step tests /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_candidate_beside.py" || { mv "$base/intervention_exit.partial" "$base/intervention_exit.txt"; exit 1; }
for policy in runtime beside; do
  for profile in combined ideal; do
    step "probe_${policy}_${profile}" /usr/bin/python3 -B "$base/probe_beside.py" --policy "$policy" --sensor-profile "$profile" \
      --output "$base/probe_${policy}_${profile}" --trace --cases B_gazebo_left B_gazebo_right
  done
done
step shadow_gazebo_right /usr/bin/python3 -B "$base/shadow_gazebo.py" artifacts/validation/2026-10-30/rear_lane_measured_right_gazebo_v1/drive_right_on/follow_decisions.jsonl --output "$base/shadow_gazebo_right.json"
step shadow_gazebo_left /usr/bin/python3 -B "$base/shadow_gazebo.py" artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1/drive_on/follow_decisions.jsonl --output "$base/shadow_gazebo_left.json"
mv "$base/intervention_exit.partial" "$base/intervention_exit.txt"
