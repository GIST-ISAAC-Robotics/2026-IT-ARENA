#!/usr/bin/env bash
set +e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
outdir='artifacts/validation/2026-10-05/g0_unit_v4'
mkdir -p "$outdir"

echo '=== latest four-file unit tests ==='
/usr/bin/python3 -m pytest -q \
  tests/test_local_scene.py \
  tests/test_g0_geometry.py \
  tests/test_multi_vehicle_g0.py \
  tests/test_r0_safety_contract.py \
  > "$outdir/pytest.log" 2>&1
pytest_rc=$?
printf '%s\n' "$pytest_rc" > "$outdir/pytest.returncode"
cat "$outdir/pytest.log"
printf 'pytest_returncode=%s\n' "$pytest_rc"
if [ "$pytest_rc" -ne 0 ]; then
  echo 'Gazebo smoke skipped because targeted tests failed.' | tee "$outdir/gazebo_skipped.txt"
  exit "$pytest_rc"
fi

echo '=== single Gazebo central_stop smoke ==='
/usr/bin/python3 scripts/validate_multi_vehicle_g0.py \
  --mode gazebo \
  --case central_stop \
  --render-backend wsl_nvidia \
  --output artifacts/validation/2026-10-05/multi_vehicle_g0_gazebo_central_v1 \
  > "$outdir/gazebo.stdout.log" 2> "$outdir/gazebo.stderr.log"
gazebo_rc=$?
printf '%s\n' "$gazebo_rc" > "$outdir/gazebo.returncode"
cat "$outdir/gazebo.stdout.log"
if [ -s "$outdir/gazebo.stderr.log" ]; then
  cat "$outdir/gazebo.stderr.log" >&2
fi
printf 'gazebo_returncode=%s\n' "$gazebo_rc"
exit "$gazebo_rc"
