#!/usr/bin/env bash
set -u
cd -- "$(dirname -- "$0")/stage" || exit 2
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface"
python3 -m pytest tests/test_steering_calibration.py::SteeringCalibrationTests::test_launch_limit_uses_cad_and_legacy_remains_identical \
  -q -p no:cacheprovider -c /dev/null --confcutdir="$PWD/tests" \
  --basetemp="$PWD/../pytest_tmp_v2" --junitxml="$PWD/../junit_v2.xml" > ../pytest_v2.log 2>&1
r430_status=$?
printf '%s\n' "$r430_status" > ../pytest_v2.exitcode
cat ../pytest_v2.log
exit "$r430_status"
