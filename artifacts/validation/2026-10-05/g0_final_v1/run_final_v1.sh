#!/usr/bin/env bash
# 최종 소스 기준 합성 관측 재생성(analytic, Gazebo 없음)과 R0 해시 확인. 구동 출력 없음.
set +e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
outdir='artifacts/validation/2026-10-05/g0_final_v1'
mkdir -p "$outdir"
/usr/bin/python3 scripts/validate_multi_vehicle_g0.py --mode analytic \
  --output artifacts/validation/2026-10-05/multi_vehicle_g0_analytic_v3 \
  > "$outdir/analytic.stdout.log" 2> "$outdir/analytic.stderr.log"
analytic_rc=$?
printf '%s\n' "$analytic_rc" > "$outdir/analytic.returncode"
cat "$outdir/analytic.stdout.log"
printf 'analytic_returncode=%s\n' "$analytic_rc"
sha256sum scripts/validate_multi_vehicle_g0.py scripts/g0_geometry.py scripts/check_g0_ros_context.py \
  config/tests/multi_vehicle_g0.json config/tests/multi_vehicle_g0_r0_sha256.json \
  src/arena_autonomy/arena_autonomy/local_scene.py \
  tests/test_local_scene.py tests/test_g0_geometry.py tests/test_multi_vehicle_g0.py tests/test_r0_safety_contract.py \
  > "$outdir/final_sources.sha256"
cat "$outdir/final_sources.sha256"
exit "$analytic_rc"
