#!/usr/bin/env bash
# v7: 시작 순서(첫 자세 시각 <= 시작 원시 시각) 수정 후 단위 회귀 → central_stop 단일 재검증
# → 성립할 때만 사전 등록 8사례 Gazebo 1회. 한도/시간은 config 그대로 두고 실패 시 재실행하지 않는다.
set +e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
outdir='artifacts/validation/2026-10-05/g0_unit_v7'
alldir='artifacts/validation/2026-10-05/g0_gazebo_all_v1'
mkdir -p "$outdir" "$alldir"

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
tail -3 "$outdir/pytest.log"
printf 'pytest_returncode=%s\n' "$pytest_rc"
if [ "$pytest_rc" -ne 0 ]; then exit "$pytest_rc"; fi

echo '=== single Gazebo central_stop after startup-order fix ==='
/usr/bin/python3 scripts/validate_multi_vehicle_g0.py \
  --mode gazebo --case central_stop --render-backend wsl_nvidia \
  --output artifacts/validation/2026-10-05/multi_vehicle_g0_gazebo_central_v4 \
  > "$outdir/gazebo.stdout.log" 2> "$outdir/gazebo.stderr.log"
central_rc=$?
printf '%s\n' "$central_rc" > "$outdir/gazebo.returncode"
cat "$outdir/gazebo.stdout.log"
printf 'central_returncode=%s\n' "$central_rc"
echo '=== leftover gz/bridge processes after central (report only) ==='
pgrep -a -f 'gz sim|parameter_bridge' | grep -v pgrep
if [ "$central_rc" -ne 0 ]; then exit "$central_rc"; fi

echo '=== eight preregistered Gazebo cases ==='
/usr/bin/python3 scripts/validate_multi_vehicle_g0.py \
  --mode gazebo --case all --render-backend wsl_nvidia \
  --output artifacts/validation/2026-10-05/multi_vehicle_g0_gazebo_all_v1 \
  > "$alldir/gazebo.stdout.log" 2> "$alldir/gazebo.stderr.log"
all_rc=$?
printf '%s\n' "$all_rc" > "$alldir/gazebo.returncode"
cat "$alldir/gazebo.stdout.log"
printf 'all_returncode=%s\n' "$all_rc"
echo '=== leftover gz/bridge processes after all (report only) ==='
pgrep -a -f 'gz sim|parameter_bridge' | grep -v pgrep
exit "$all_rc"
