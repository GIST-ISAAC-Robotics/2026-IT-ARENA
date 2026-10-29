#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/bump_traffic_review_v1
export PYTHONPATH="$PWD/$base:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/mixed_contracts_v1.log"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_mixed_clearance.py" tests/test_rolling_clearance_state.py tests/test_positive_clearance_state.py tests/test_anchor_clearance_probe.py tests/test_recovery_threat_capture.py > "$base/mixed_contracts_v1.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/mixed_contracts_v1.exit"
exit "$result"
