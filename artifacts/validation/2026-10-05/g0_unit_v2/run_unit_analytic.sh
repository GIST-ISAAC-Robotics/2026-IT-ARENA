#!/usr/bin/env bash
set +e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"

outdir='artifacts/validation/2026-10-05/g0_unit_v2'
mkdir -p "$outdir"

echo '=== targeted unit tests ==='
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

echo '=== analytic CLI help ==='
/usr/bin/python3 scripts/validate_multi_vehicle_g0.py --help \
  > "$outdir/analytic_help.log" 2>&1
help_rc=$?
printf '%s\n' "$help_rc" > "$outdir/analytic_help.returncode"
cat "$outdir/analytic_help.log"
printf 'analytic_help_returncode=%s\n' "$help_rc"

if [ "$help_rc" -ne 0 ]; then
  exit "$help_rc"
fi

echo '=== analytic run ==='
/usr/bin/python3 scripts/validate_multi_vehicle_g0.py \
  --mode analytic \
  --output artifacts/validation/2026-10-05/multi_vehicle_g0_analytic_v2 \
  > "$outdir/analytic.stdout.log" 2> "$outdir/analytic.stderr.log"
analytic_rc=$?
printf '%s\n' "$analytic_rc" > "$outdir/analytic.returncode"
cat "$outdir/analytic.stdout.log"
if [ -s "$outdir/analytic.stderr.log" ]; then
  cat "$outdir/analytic.stderr.log" >&2
fi
printf 'analytic_returncode=%s\n' "$analytic_rc"
exit "$analytic_rc"
