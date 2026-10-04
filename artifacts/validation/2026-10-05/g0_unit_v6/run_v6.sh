#!/usr/bin/env bash
# v6: PosePublisher 실제 토픽(/model/g0_actor/pose) 수정 후 단위 회귀 → Context probe → central_stop 1회.
set +e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
outdir='artifacts/validation/2026-10-05/g0_unit_v6'
mkdir -p "$outdir"

echo '=== targeted G0 + related regression tests ==='
/usr/bin/python3 -m pytest -q \
  tests/test_local_scene.py \
  tests/test_g0_geometry.py \
  tests/test_multi_vehicle_g0.py \
  tests/test_r0_safety_contract.py \
  tests/test_local_pursuit.py \
  tests/test_lidar_safety.py \
  tests/test_lidar_observation.py \
  tests/test_lidar_motion.py \
  tests/test_rotating_lidar.py \
  > "$outdir/pytest.log" 2>&1
pytest_rc=$?
printf '%s\n' "$pytest_rc" > "$outdir/pytest.returncode"
tail -5 "$outdir/pytest.log"
printf 'pytest_returncode=%s\n' "$pytest_rc"
if [ "$pytest_rc" -ne 0 ]; then exit "$pytest_rc"; fi

echo '=== ROS context check ==='
/usr/bin/python3 scripts/check_g0_ros_context.py > "$outdir/context.stdout.log" 2> "$outdir/context.stderr.log"
context_rc=$?
printf '%s\n' "$context_rc" > "$outdir/context.returncode"
cat "$outdir/context.stdout.log"
printf 'context_returncode=%s\n' "$context_rc"
if [ "$context_rc" -ne 0 ]; then exit "$context_rc"; fi

echo '=== single Gazebo central_stop after pose topic fix ==='
/usr/bin/python3 scripts/validate_multi_vehicle_g0.py \
  --mode gazebo \
  --case central_stop \
  --render-backend wsl_nvidia \
  --output artifacts/validation/2026-10-05/multi_vehicle_g0_gazebo_central_v3 \
  > "$outdir/gazebo.stdout.log" 2> "$outdir/gazebo.stderr.log"
gazebo_rc=$?
printf '%s\n' "$gazebo_rc" > "$outdir/gazebo.returncode"
cat "$outdir/gazebo.stdout.log"
printf 'gazebo_returncode=%s\n' "$gazebo_rc"
echo '=== leftover gz/bridge processes (report only) ==='
pgrep -a -f 'gz sim|parameter_bridge' | grep -v pgrep
exit "$gazebo_rc"
