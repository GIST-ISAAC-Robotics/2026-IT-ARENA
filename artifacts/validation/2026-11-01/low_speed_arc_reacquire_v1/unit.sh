#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests:$PYTHONPATH"
export PYTHONDONTWRITEBYTECODE=1
A=artifacts/validation/2026-11-01/low_speed_arc_reacquire_v1
nice -n 10 python3 -m pytest -q -p no:cacheprovider --tb=short tests/test_low_speed_arc_reacquire.py "$@" > "$A/pytest_unit.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" >> "$A/pytest_unit.log"
tail -30 "$A/pytest_unit.log" | cut -c1-170
exit $code
