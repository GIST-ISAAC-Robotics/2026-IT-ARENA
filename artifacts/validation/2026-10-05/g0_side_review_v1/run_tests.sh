#!/usr/bin/env bash
# 별도 코드 검토의 수정 전/후 회귀. Gazebo·차량·Jetson을 실행하지 않는다.
set -e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
outdir='artifacts/validation/2026-10-05/g0_side_review_v1'
phase="$1"
case "$phase" in
  red) test_args=(tests/test_multi_vehicle_g0.py -k test_review_) ;;
  green) test_args=(tests/test_local_scene.py tests/test_g0_geometry.py tests/test_multi_vehicle_g0.py tests/test_r0_safety_contract.py tests/test_local_pursuit.py tests/test_lidar_safety.py tests/test_lidar_observation.py tests/test_lidar_motion.py tests/test_rotating_lidar.py) ;;
  *) exit 2 ;;
esac
if [ -e "$outdir/$phase.log" ]; then
  printf 'Refusing to overwrite existing evidence: %s\n' "$outdir/$phase.log"
  exit 3
fi
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "${test_args[@]}" > "$outdir/$phase.log" 2>&1
test_status=$?
set -e
printf '%s\n' "$test_status" > "$outdir/$phase.returncode"
tail -5 "$outdir/$phase.log"
exit "$test_status"
