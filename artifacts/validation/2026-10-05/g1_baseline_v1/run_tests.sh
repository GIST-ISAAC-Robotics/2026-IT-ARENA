#!/usr/bin/env bash
set -e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
outdir='artifacts/validation/2026-10-05/g1_baseline_v1'
test_args=(tests/test_local_scene.py tests/test_g0_geometry.py tests/test_multi_vehicle_g0.py tests/test_r0_safety_contract.py tests/test_local_pursuit.py tests/test_lidar_safety.py tests/test_lidar_observation.py tests/test_lidar_motion.py tests/test_rotating_lidar.py)
if [ -e "$outdir/green.log" ]; then
  printf 'Refusing to overwrite existing evidence: %s\n' "$outdir/green.log"
  exit 3
fi
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "${test_args[@]}" > "$outdir/green.log" 2>&1
test_status=$?
set -e
printf '%s\n' "$test_status" > "$outdir/green.returncode"
tail -8 "$outdir/green.log"
printf 'pytest_returncode=%s\n' "$test_status"
exit "$test_status"
